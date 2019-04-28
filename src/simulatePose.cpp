#include <iostream>
#include <opencv2/opencv.hpp>
#include "simulatePose.hpp"

/*
This class generates a visual simulation environment to evaluate VO-algorithm

Features:
-   Provide a floor image separately or use the default chessboard pattern
-   Define parameters to relate to physical coordinates. Define camera position and pose in undistorted image case
        Example: Camera coordinate T and rotation R, with focal length f (or wide angle) gives undistorted projection
    The object the constructs the camera K that fulfills the specifications
-   For each new physical camera coordinate and pose [m],[rad], the object calculates the corresponding image warp
*/

    /*Sets the default chessboard pattern as base scene
    */
void simulatePose::setBaseScene(int blockSize,int rowsOfBoxes,int colsOfBoxes){
    baseScene = getChessboard(blockSize,rowsOfBoxes,colsOfBoxes);
    cx = ((float) baseScene.cols)/2;
    cy = ((float) baseScene.rows)/2;
    N = (float) baseScene.cols;
}
    /* Specify own image for basescene
     * TODO: Perform check of filetype and make sure that it is of type CV_8UC3
     */
void simulatePose::setBaseScene(cv::Mat baseScene_){
    baseScene = baseScene_;
    cx = ((float) baseScene.cols) /2;
    cy = ((float) baseScene.rows) /2;
    N = (float) baseScene.cols;
}

    /* This function is used to specify which camera pose that projects to
     * the undistorted base scene. It defines the R1, T1 that are initial camera pose,
     * and constructs the camera K that fulfills the specifications
     * TODO: check if v is defined correctly. R1.t() maybe??
     */
void simulatePose::setBasePose(std::vector<float> angles,std::vector<float> coordinates){
    if((angles.size()!=3) || (coordinates.size()!=3)){std::cout << "Not valid coordinates"<< std::endl;}
    //Base rotation of camera 1
    float yaw = angles[0];
    float pitch = angles[1];
    float roll = angles[2];
    cv::Mat_<float> Rz_ = getZRot(yaw);
    cv::Mat_<float> Ry_ = getYRot(pitch);
    cv::Mat_<float> Rx_ = getXRot(roll);
    R1 = Rx_*Ry_*Rz_;                                       //Set base rotation matrix
    //Base coordinate of camera 1
    float x = coordinates[0];
    float y = coordinates[1];
    float z = coordinates[2];
    cv::Mat_<float> t1_ = cv::Mat_<float>::zeros(3,1);
    t1_(0,0) = x;
    t1_(1,0) = y;
    t1_(2,0) = z;
    t1 = t1_;                                               //Set base coordinates
    //Base plane definition. This sets base plane in global xy-plane
    d = t1(2,0);                                            //Distance from camera 1 to plane NOTE ONLY VALID IF PLANE IS HORIZONTAL
    cv::Mat_<float> v_global = cv::Mat_<float>::zeros(3,1);
    v_global(2,0) = 1;                                      //Base plane normal in global coordinate system. positive z
    v = R1*v_global;                                        //Base plane normal expressed in camera 1 coordinate system
}

cv::Mat simulatePose::getChessboard(int blockSize,int rowsOfBoxes,int colsOfBoxes){
    int imageRows=blockSize*rowsOfBoxes;
    int imageCols=blockSize*colsOfBoxes;
    cv::Mat chessBoard(imageRows,imageCols,CV_8UC3,cv::Scalar::all(0));
    unsigned char color=0;
     for(int i=0;i<imageCols;i=i+blockSize){
      color=~color;
       for(int j=0;j<imageRows;j=j+blockSize){
       cv::Mat ROI=chessBoard(cv::Rect(i,j,blockSize,blockSize));
       ROI.setTo(cv::Scalar::all(color));
       color=~color;
      }
     }
     //Add coordinate system
     int scale = 30;
     cv::Point cntr = cv::Point(imageCols/2,imageRows/2);
     cv::Point x = cntr + cv::Point(1,0)*scale;
     cv::Point y = cntr +cv::Point(0,1)*scale;
     cv::arrowedLine(chessBoard,cntr,x,CV_RGB(255,0,0),2,cv::LINE_8,0,0.1);
     cv::arrowedLine(chessBoard,cntr,y,CV_RGB(255,0,0),2,cv::LINE_8,0,0.1);
    return chessBoard;
}
/*This function sets physical parameters of simulation environment one by one
 * TODO: add parameters x-y to shift global coordinate system??
 */
int simulatePose::setParam(std::string parameterName,float value){
    static int init = 0;
    if(!init){
        init=1;
        std::vector<std::vector<std::string>> validParameters_;
        //Init valid parameters as their names and description
        std::vector<std::string> param0{"sceneWidth",   "Physical width of scene in x-direction         [m]"};
        std::vector<std::string> param1{"d",            "Distance between camera and scene in base pose [m]"};
        std::vector<std::string> param2{"angle",        "Cameras angle of view in x-direction           [rad]"};
        std::vector<std::string> param3{"f_m",      "Focal length in meter                          [m]"};
        std::vector<std::string> param4{"f_p",      "Focal length in pixles                         [pixles]"};
        validParameters_.push_back(param0);
        validParameters_.push_back(param1);
        validParameters_.push_back(param2);
        validParameters_.push_back(param3);
        validParameters_.push_back(param4);
        validParameters = validParameters_; //Save as object attribute
        std::vector<std::vector<int>> validConfigurations_;
        std::vector<int> conf0{0,1};//sceneWidth and distance to base scene
        std::vector<int> conf1{0,2};//sceneWidth and viewing angle in x-direction
        std::vector<int> conf2{1,2};//distance to base scene and viewing angle in x direction
        std::vector<int> conf3{4,1};//focal length in pixles and distance to base scene
        std::vector<int> conf4{0,1,3};//sceneWidth and distance to base scene and focal length in meter
        //There are more configurations..
        validConfigurations_.push_back(conf0);
        validConfigurations_.push_back(conf1);
        validConfigurations_.push_back(conf2);
        validConfigurations_.push_back(conf3);
        validConfigurations_.push_back(conf4);
        validConfigurations = validConfigurations_;
    }
    if(parameterName == "sceneWidth"){
        sceneWidth = value;
    }
    else if(parameterName == "d"){
        d = value;
    }
    else if(parameterName == "angle"){
        angle = value;
    }
    else if(parameterName == "f_m"){
        f_m = value;
    }
    else if(parameterName == "f_p"){
        f_p = value;
    }
    else{
        std::cout << "Parameter name '" << parameterName << "' is not valid. Valid parameter configurations are:" << std::endl;
        std::cout << "Parameter name" << "\t\t" << "Description" << std::endl;
        for(int i=0;i<validConfigurations.size();i++){  //Print out all valid configurations
            std::cout << "---------Configuration " << i << "---------" << std::endl;
            for(int parameterIndex:validConfigurations[i]){//For all parameters in configuration i:
                std::cout << validParameters[parameterIndex][0] << "\t \t" << validParameters[parameterIndex][1] << std::endl;//Print out parameter name and its description
            }
        }
    }
    //Try to calculate K matrix with the available parameters
    if(!setKMat()){
        std::cout << "K matrix specified." << std::endl;
        return 1;
    }else{return 0;}

}
//Overloaded version to give info in case of erronous function call
int simulatePose::setParam(void){
    setParam("",0); //Dummy call of set param with invalid parameter name
    return 0;
}
/* This method initializes the simulation environment according to the configuration specified
 *
 */
int simulatePose::init(int configuration){
    switch(configuration){
        case 0:{
            if(d!=0 && sceneWidth!=0 && N!=0){//If the necessary attributes are defined
                //Set base pose
                std::vector<float> angles{3.1415,0,0};//Rotate around x to look down at base scene -> x-direction is not changed.
                std::vector<float> coordinates{0,0,d};
                setBasePose(angles,coordinates);
                setKMat();//Set K-mat according to default configuration, d,sceneWidth, N
            }else{
                return 0;
            }
            break;
        }
        case 1:{
            std::cout << "configuration not yet implemented" << std::endl;
            return 0;
            break;
        }
        case 2:{
            std::cout << "configuration not yet implemented" << std::endl;
            return 0;
            break;
        }
        case 3:{
            std::cout << "configuration not yet implemented" << std::endl;
            return 0;
            break;
        }
        case 4:{
            std::cout << "configuration not yet implemented" << std::endl;
            return 0;
            break;
        }
    }
    return 1;
}
/*Defines the K-matrix.
 TODO- give configuration as argument and build according to that
*/
int simulatePose::setKMat(void){
    cv::Mat_<float> K_ = cv::Mat_<float>::zeros(3,3);
    if(sceneWidth){
        float fx = d*N/sceneWidth;
        float fy = fx;
        K_(0,0) = fx;
        K_(1,1) = fy;
    }else{std::cout << "No scene width specified." << std::endl; return 0;}
    K_(0,2) = cx;//Use the specified
    K_(1,2) = cy;
    K_(2,2) = 1;// Should this be 1 as 1 focal length or fx as one focal lengths worth of pixles?
    K = K_;
    return 1;
}

/*Returns the warped baseScene. coordinates are to be given ass [roll,pitch,yaw], [x,y,z]
 *
 */
cv::Mat simulatePose::getWarpedImage(std::vector<float> angles,std::vector<float> t){
    cv::Mat out;
    if((angles.size()!=3) ||t.size()!=3){
        std::cout << "Not valid coordinates" << std::endl;
        return out;
    }
    float roll = angles[0];
    float pitch = angles[1];
    float yaw = angles[2];
    float x = t[0];
    float y = t[1];
    float z = t[2];
    cv::Mat R_x = getXRot(roll);                      //Rotations of camera pose 2
    cv::Mat R_y = getYRot(pitch);
    cv::Mat R_z = getZRot(yaw);
    cv::Mat_<float> t2 = cv::Mat_<float>::zeros(3,1); //Coordinate of camera pose 2
    t2(0,0) = x;
    t2(1,0) = y;
    t2(2,0) = z;

    //Define the pure rotation homography of roll,pitch http://people.scs.carleton.ca/~c_shu/Courses/comp4900d/notes/homography.pdf
    cv::Mat_<float> H_tilt = K * R_x*R_y * K.inv(); // Pure rotation (roll,pitch)
    //Define the yaw+translation homography according to 3.8 in Wadenbaeck.
    cv::Mat_<float> b = t1 - R1.t()*R_z*t2;
    cv::Mat_<float> A = R1.t()*R_z;
    cv::Mat_<float> H_trans = K * (A - b*v.t()/d) * K.inv();

    //Calculate complete homography and perform the perspective transform
    cv::Mat_<float> H = H_tilt*H_trans;
    cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,0);
    //cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,0);
    return out;
}
/* Functions for defining roll, pitch, and yaw rotation matrices
 *
 */
cv::Mat simulatePose::getXRot(float roll){
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

cv::Mat simulatePose::getYRot(float pitch){
    float sinY = std::sin(pitch);
    float cosY = std::cos(pitch);//std::sin(-pitch);
    cv::Mat_<float> R_y = cv::Mat_<float>::zeros(3,3);
    R_y(0,0) = cosY;
    R_y(0,2) = sinY;
    R_y(1,1) = 1;
    R_y(2,0) = -sinY; //Rätt?
    R_y(2,2) = cosY;
    return R_y;
}
cv::Mat simulatePose::getZRot(float yaw){
    float sinZ = std::sin(yaw);
    float cosZ = std::cos(yaw);//std::sin(-pitch);
    cv::Mat_<float> R_z = cv::Mat_<float>::zeros(3,3);
    R_z(0,0) = cosZ;
    R_z(0,1) = sinZ;
    R_z(1,0) = -sinZ;
    R_z(1,1) = cosZ;
    R_z(2,2) = 1;
    return R_z;
}
