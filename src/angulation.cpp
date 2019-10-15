#include <iostream>
#include <opencv2/opencv.hpp>
#include "angulation.hpp"
#include "azipe.hpp"


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


/* Performs the position and azimuth calculation
 * Discards all elements that are masked away and then calls azipe
 * Depending on how the roll and pitch data is available, maybe it can be given as cos terms directly?
 */
int ang::angulation::calculate(std::vector<cv::Mat_<float>>& q, std::vector<cv::Mat_<float>>& v,std::vector<bool>& mask, cv::Mat_<float>& pos,float& yaw, float roll,float pitch){
    //Use the mask to feed through
    std::vector<cv::Mat_<float>> v_m;
    std::vector<cv::Mat_<float>> q_m;
    std::vector<cv::Mat_<float>>::iterator it_v = v.begin();
    std::vector<cv::Mat_<float>>::iterator it_q = q.begin();
    std::vector<bool>::iterator it_mask = mask.begin();
    while(it_mask != mask.end()){
        if(*it_mask){
            v_m.push_back(*it_v);
            q_m.push_back(*it_q);
        }
        it_v++;
        it_q++;
        it_mask++;
    }
    cv::Mat_<float> t_opt;
    //return az::azipe(v_m,q_m,pos,yaw,roll,pitch,t_opt);//Gammal ordning på vinklar
    az::azipe(v_m,q_m,pos,yaw,pitch,roll,t_opt);
    if(v_m.size()>2 && false){
        int r = az::aipe(v_m,q_m,pos,yaw,pitch,roll,1,t_opt);
            //std::cout << "Roll: " << roll << ", pitch: " << pitch << std::endl;
        return r;
    }
    return 0;

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
        cv::Mat_<float> direction = T * K_inv * point_mat;
        float v_norm = (float) cv::norm(direction,cv::NORM_L2);
        direction /= v_norm;
        //cv::Mat_<float> direction =  K_inv * point_mat;
        //Normalize to length one
        //cv::Mat_<float> norm = cv::Mat_<float>::ones(1,3)*direction;//Norm is evaluated to 1x1 mat
        //direction /= std::sqrt(norm(0,0));
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
    //Shift coordinate system center to middle of image
    float x = point.x - K(0,2);
    float y = point.y - K(1,2);
    //Perform undistortion
    float r2 = x*x + y*y;//Radius squared
    float A = (1 + k1_barrel*r2 + k2_barrel*r2*r2 + k3_barrel*r2*r2*r2);
    cv::Point2f undistortedPoint(x/A,y/A);//SHOULD IT BE MULTIPLIED? docs are not definitive. Different in diferent version of docs
    //Shift coordinates back to image
    undistortedPoint.x += K(0,2);
    undistortedPoint.y += K(1,2);
    return undistortedPoint;
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
