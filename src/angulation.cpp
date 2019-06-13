#include <iostream>
#include <opencv2/opencv.hpp>
#include "angulation.hpp"
#include "azipe.hpp"


/*
TODO: in dataBase2q, if there are unknown anchors. raise some flag
At end of calculate-method, do pnp solve
pnp fits best here in angulation class
Then there is no repeated calculations
*/

/*constructor. Set max id of database, initializes it with the provided path to csv file containing id and coordinates
*/
ang::angulation::angulation(int maxId_,std::string path){
    maxId = maxId_;
    minAnchors = 2;
    //minAnchors = 2;             //At least visible anchors are needed to attempt AZIPE angulation
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
 * Depending on how the roll and pitch data is available, maybe it can be given as cos terms directly?
 */
bool ang::angulation::calculate(std::vector<cv::Point2f>& locations, std::vector<int>& IDs,cv::Mat_<float>& pos,float& yaw, float roll,float pitch){
    //Get locations of visible anchors
    std::vector<cv::Mat_<float>> q;
    if(dataBase2q(IDs,q)<minAnchors){return false;};// not enough known anchors

    //Calculate uLOS-vectors v from K,T
    std::vector<cv::Mat_<float>> v;
    pix2uLOS(locations,v);
    return az::azipe(v,q,pos,yaw,roll,pitch);

}

/*Overloaded version of calculate. It takes the mean value of the provided vector<point2f> for each ID and then
 * calls standard caluclate-method
 */
bool ang::angulation::calculate(std::vector<std::vector<cv::Point2f>>& cornerLocations, std::vector<int>& IDs,cv::Mat_<float>& pos,float& yaw, float roll,float pitch){
    std::vector<cv::Point2f> anchorLocations;
    for(int i=0;i<cornerLocations.size();i++){//Go through all anchors
        std::vector<cv::Point2f>::iterator cornerIt = cornerLocations[i].begin();
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
    return calculate(anchorLocations,IDs,pos,yaw,roll,pitch);
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
        point_mat(0,0) = it->x;
        point_mat(1,0) = it->y;
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
    /*std::cout << "directions_" << call << "= [" << std::endl;
    for(int i=0;i<uLOS.size();i++){
        std::cout << uLOS[i](0,0) << ", " << uLOS[i](1,0) << ", " << uLOS[i](2,0) << ";" << std::endl;
    }
    std::cout << "];" << std::endl;
    call++;*/
}
/* Takes IDs and returns vector of corresponding q vectors, if available.
    fills the inputoutputarray mask with values showing which anchors are known and which are unknown.
returns the amount of known anchors available
 */
int ang::angulation::dataBase2q(const std::vector<int>& IDs,std::vector<cv::Mat_<float>>& q_vectors,std::vector<bool>& mask){
    int amountOfKnown = 0;
    mask.clear();//Clear vector to fill it with new values
    for(int i:IDs){
        std::vector<cv::Mat_<float>>::iterator ptr = dataBase.begin() + i;//i is not an incremented variable. i takes the value if IDs at an incremented position
        std::vector<bool>::iterator active = activeAnchors.begin() + i;
        //ptr+=i;//i is not an incremented variable. i takes the value if IDs at an incremented position
        //active+=i;
        q_vectors.push_back(*ptr);
        mask.push_back(*active);//Fill mask with true or false
        if(*active){//If the current ID is active, i.e known
            amountOfKnown++;
        }
        //*active = true;
    }
    return amountOfKnown;
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
