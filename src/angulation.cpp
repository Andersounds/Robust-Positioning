#include <iostream>
#include <opencv2/opencv.hpp>
#include "angulation.hpp"



/*
TODO: in dataBase2q, if there are unknown anchors. raise some flag. DONE. - inputouput array used as mask
At end of calculate-method, do pnp solve
pnp fits best here in angulation class
Then there is no repeated calculations
*/

/*constructor. Set max id of database, initializes it with the provided path to csv file containing id and coordinates
*/
ang::angulation::angulation(int maxId_,std::string path){
    maxId = maxId_;
    minAnchors = 2;
    k1_barrel = 0; //Initialize with coefficients 0 for barrel distortion compensation
    k2_barrel = 0;
    k3_barrel = 0;
    for(int i=0;i<maxId;i++){
        dataBase.push_back(cv::Mat_<float>::zeros(3,1));//Set empty mat
        activeAnchors.push_back(false);                 //Set all anchors to inactive
    }
    std::vector<int> ids;
    std::vector<cv::Mat_<float>> coordinates;
    if(!readToDataBase(path,ids,coordinates)){
        std::cout << "Could not read anchor database" << std::endl;
    }else{
        std::cout << "Read " << ids.size() << " anchors to aruco database."<< std::endl;
        int index = 0;
        for(int id:ids){
            coordinates[index].copyTo(dataBase[id]);//Assign coordinates to the right anchor ID
            activeAnchors[id] = true;
            index++;
        }
    }
}
/*Takes a path and reads the file into two vectors, vector<int> IDs and vector<mat> coordinates
 * Ignores any LEADING whitespaces and rows not containing exactly 4 data fields. delimiter is char ','
 * Each anchor shall be specified as id,x,y,z\n (Whit optional whitespaces after any comma)
 */
int ang::angulation::readToDataBase(std::string path,std::vector<int>& IDs, std::vector<cv::Mat_<float>>& coordinates){
    std::cout << "Reading aruco database from " << path << " ...";
    std::string line;
    std::string delim = ",";
    std::ifstream file;
    file.open(path);
    if(file.is_open()){
         while(getline(file,line)){
            std::vector<std::string> parsed = parse(line);
            if(parsed.size()==4){//Disregard any lines that are not exactly 4 elements long
                int id      =   std::stoi(parsed[0]);
                float x     =   std::stof(parsed[1]);
                float y     =   std::stof(parsed[2]);
                float z     =   std::stof(parsed[3]);
                cv::Mat_<float> coord = cv::Mat_<float>::zeros(3,1);
                coord(0,0) = x;
                coord(0,1) = y;
                coord(0,2) = z;
                IDs.push_back(id);
                coordinates.push_back(coord);
            }
         }
    }else{return 0;}
    file.close();
    std::cout << "Done." << std::endl;
    return 1;
}

//This function takes a line and parses it into a vector<string> using "," as deliminator and disregarding LEADING whitespaces
std::vector<std::string> ang::angulation::parse(std::string line){
    char delim = ',';
    std::vector<std::string> parsed;
    std::string::iterator it = line.begin();
    while(it!=line.end()){
        std::string word;
        while(it!=line.end()){
            if(isspace(*it)){it++;}//Remove leading whitespaces
            else{break;}
        }
        while(it!=line.end()){
            if(*it != delim){
                word+=*it;//Append the char to the temporary string
                it++;
            }//Go through until deliminator
            else{it++;
                break;}
        }
        parsed.push_back(word);//Push back the parsed word onto the return vector
    }
    return parsed;
}


int ang::angulation::maskOut(std::vector<cv::Mat_<float>>& vFull, std::vector<cv::Mat_<float>>& vMasked, std::vector<bool>& mask){
    //Use the mask to feed through
    vMasked.clear();
    std::vector<cv::Mat_<float>>::iterator it_v = vFull.begin();
    std::vector<bool>::iterator it_mask = mask.begin();
    while(it_mask != mask.end()){
        if(*it_mask){
            vMasked.push_back(*it_v);
        }
        it_v++;
        it_mask++;
    }
    return 1;
}

/*Converts a vector of camera pixel coordinates to a vector of uLOS vectors expressed as Mat_<float>
 *
 */
void ang::angulation::pix2uLOS(const std::vector<cv::Point2f>& points,std::vector<cv::Mat_<float>>& uLOS){
    //static int call = 0;
    uLOS.clear();              //Define vector to be returned
    //Loop through pixel points vector and build
    std::vector<cv::Point2f>::const_iterator it = points.begin();
    while(it!=points.end()){
        // Create a mat_<float> with pixel coordinates and z coordinte = 1
        cv::Mat_<float> point_mat = cv::Mat_<float>::ones(3,1);
        cv::Point2f undistortedPoint = unDistort(*it);//Compensate for the radial distortion
        point_mat(0,0) = undistortedPoint.x;
        point_mat(1,0) = undistortedPoint.y;
        //Transform to 3d image plane coordinates, then transfrom from image frame to uav frame
        //std::cout << "K inv: " << K_inv << std::endl;
        cv::Mat_<float> direction = T * K_inv * point_mat;
        //direction(2,0) = 0.001;
        std::cout << "################Z 0" << std::endl;
        float v_norm = (float) cv::norm(direction,cv::NORM_L2);
        direction /= v_norm;
        //Add to uLOS vector
        uLOS.push_back(direction);
        //Iterate
        it++;
    }
}
/*Overloaded version of pix2uLOS. If corner locations of anchors are given instead of points directly, the mean of each vector<point2f> is considered
 *  as the anchors location. This is calculated and then the original pix2uLOS is called
 */
void ang::angulation::pix2uLOS(const std::vector<std::vector<cv::Point2f>>& cornerLocations,std::vector<cv::Mat_<float>>& uLOS){
    std::vector<cv::Point2f> anchorLocations;
    for(int i=0;i<cornerLocations.size();i++){//Go through all anchors
        std::vector<cv::Point2f>::const_iterator cornerIt = cornerLocations[i].begin();
        cv::Point2f location(0,0);
        while(cornerIt != cornerLocations[i].end()){
            location += *cornerIt;
            cornerIt++;
        }
        location.x /= cornerLocations[i].size();
        location.y /= cornerLocations[i].size();
        anchorLocations.push_back(location);
    }
    //Do standard function call to calculate
    return pix2uLOS(anchorLocations,uLOS);
}
/* Takes IDs and returns vector of corresponding q vectors, if available.
    fills the inputoutputarray mask with values showing which anchors are known and which are unknown.
returns the amount of known anchors available
    q vector is global coordinate of anchor
 */
int ang::angulation::dataBase2q(const std::vector<int>& IDs,std::vector<cv::Mat_<float>>& q_vectors,std::vector<bool>& mask){
    int amountOfKnown = 0;
    mask.clear();//Clear vector to fill it with new values
    for(int i:IDs){
        q_vectors.push_back(dataBase[i]);
        bool active = activeAnchors[i];
        mask.push_back(active);
        if(active){
            amountOfKnown++;
        }



/*

        std::vector<cv::Mat_<float>>::iterator ptr = dataBase.begin();// + i;//i is not an incremented variable. i takes the value if IDs at an incremented position
        std::vector<bool>::iterator active = activeAnchors.begin();// + i;
        ptr+=i;//i is not an incremented variable. i takes the value if IDs at an incremented position
        active+=i;
        q_vectors.push_back(*ptr);
        mask.push_back(*active);//Fill mask with true or false
        if(*active){//If the current ID is active, i.e known
            amountOfKnown++;
        }
*/

    }
    return amountOfKnown;
}


/* method to perform in-place correction of barrel distortion of pixel coordinates
 * Using the following formula: https://docs.opencv.org/3.4.0/d4/d94/tutorial_camera_calibration.html
 */
cv::Point2f ang::angulation::unDistort(const cv::Point2f& point){
    return point;
    //Shift coordinate system center to middle of image
    float x = point.x - K(0,2);
    float y = point.y - K(1,2);
//    std::cout << "shift point: [" << x<< ", "<<y << "]."<< std::endl;
    //Perform undistortion
    float r2 = x*x + y*y;//Radius squared
//    float A = (1 + k1_barrel*r2 + k2_barrel*r2*r2 + k3_barrel*r2*r2*r2);
    float A = 1;
    //cv::Point2f undistortedPoint(x/A,y/A);//SHOULD IT BE MULTIPLIED? docs are not definitive. Different in diferent version of docs
    cv::Point2f undistortedPoint(x*A,y*A);//
    //Shift coordinates back to image
//    std::cout << "New point: [" << undistortedPoint.x<< ", "<<undistortedPoint.y << "]." <<std::endl;
    undistortedPoint.x += K(0,2);
    undistortedPoint.y += K(1,2);

    //return undistortedPoint;
  // Below is undistortion example using opencv.
/*    cv::Mat from = cv::Mat(1,1,CV_32FC2);
    from.at<cv::Point2f>(0,0)=point;
    cv::Mat to = cv::Mat(1,1,CV_32FC2);
    std::vector<double> coeffs;
    coeffs.push_back(0.2486);
    coeffs.push_back(-1.4526);
    coeffs.push_back(0);
    coeffs.push_back(0);
    coeffs.push_back(2.6388);
    cv::Mat R;
    cv::undistortPoints(from,to,K,coeffs,R,K);
    std::cout << "Raw point: [" << point.x<< ", "<<point.y << "]."<< std::endl;
    std::cout << "OCV point:" << to << std::endl;
    cv::Point2f newPoint = to.at<cv::Point2f>(0,0);
    return newPoint;
*/
}


/*Derotate ULOS vectors
*/
void ang::angulation::deRotateULOS(std::vector<cv::Mat_<float>>& ulos,float roll,float pitch){
    cv::Mat_<float> Rroll = getXRot(roll);//positive to rotate with the UAV
    cv::Mat_<float> Rpitch = getYRot(pitch);
    for(int i=0;i<ulos.size();i++){
        cv::Mat_<float> derotated = Rpitch*Rroll*ulos[i];
        derotated.copyTo(ulos[i]);
    }

}

/* Interface mathod used to set K mat and calculate its inverse
 */
void ang::angulation::setKmat(cv::Mat_<float>K_){
    K = K_;
    K_inv = K.inv();
}
/* Interface method used to set T mat. T is transformation fro  uav frame to camera frame
 */
void ang::angulation::setTmat(cv::Mat_<float> T_){
    T = T_;
    T_inv = T.inv();
}
/* Interface method to set distortion coefficients for barrel distortion compensation
*/
void ang::angulation::setDistortCoefficents(float k1, float k2, float k3){
    k1_barrel = k1;
    k2_barrel = k2;
    k3_barrel = k3;
}
/* Functions for defining roll, pitch, and yaw rotation matrices
 * Increase speed by passing reference and edit in place?
 */
cv::Mat ang::angulation::getXRot(float roll){
    float sinX = std::sin(roll);
    float cosX = std::cos(roll);
    cv::Mat_<float> R_x = cv::Mat_<float>::zeros(3,3);
    R_x(0,0) = 1;
    R_x(1,1) = cosX;
    R_x(1,2) = -sinX;
    R_x(2,1) = sinX;
    R_x(2,2) = cosX;
    return R_x;
}
cv::Mat ang::angulation::getYRot(float pitch){
    float sinY = std::sin(pitch);
    float cosY = std::cos(pitch);
    cv::Mat_<float> R_y = cv::Mat_<float>::zeros(3,3);
    R_y(0,0) = cosY;
    R_y(0,2) = sinY;
    R_y(1,1) = 1;
    R_y(2,0) = -sinY;
    R_y(2,2) = cosY;
    return R_y;
}
