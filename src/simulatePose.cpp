#include <iostream>
#include <opencv2/opencv.hpp>
#include "simulatePose.hpp"

/*
This class generates a visual simulation environment to evaluate VO-algorithm
-   Provide a floor image separately or use the default chessboard pattern
    This is locked to the global coordinate system. By default located in x-y-plane and z pointing downwards.
-   Define parameters which together with default scene defines the physical dimensions
        Example:    Define that base scene is sceneWidth m across in x-direction of image
                    Define that uav is located at d m in negative z-direction of global frame, i.e. at (x,y,-d), in base scene
                    Define location of global x-y origin by setting x and y in base pose
                    Define heading of UAV in base pose as a rotation around z-axis. (-pi/2 rad)
                    Assumption: Camera is looking down towards floor
        Example: Camera coordinate t and rotation R, with focal length f (or wide angle) gives undistorted projection
    The object the constructs the camera K that fulfills the specifications
-   For each new physical camera coordinate and pose [m],[rad], the object calculates the corresponding image warp
TODO:   -Let user define general global coordinate at base scene
        -
*/
simulatePose::simulatePose(void){
    //Variables for definition of K matrix
    N = 0;
    cx = 0;
    cy = 0;
    f_p = 0;
    //Relationship between global coordinate system and base pose. Default to being completely aligned
    x_base = 0;
    y_base = 0;
    z_base = 0;
    yaw_base  = 0;
    pitch_base= 0;
    roll_base = 0;
    T = cv::Mat_<float>::eye(3,4);              //Base conversion from UAV frame to camera frame
    //Physical scale variables
    sceneWidth = 0;//Maybe have dafault 1?
    angle = 0;
    f_m = 0;
    //gamma=0;
    //Define scene plane as expressed in base pose. perpendiculat to z-axis
    d  = 0;                             // 0 is non-accepted value. user must define via some configuration
    v = cv::Mat_<float>::zeros(3,1);
    v(2,0) = 1;
}

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
/*
This function will be used to define the relationship between the global coordinate system and the base pose
 */
void simulatePose::setBasePose(void){
    //Base rotation of camera 1. i.e the R matrix of the camera that should achieve the non-distorted base projection
    R1 = getTotRot(yaw_base,pitch_base,roll_base);                                    //Set base rotation matrix
    //Base coordinate of camera 1. i.e the t of the camera that should achieve the non-distorted base projection
    t1 = coord2CMat (x_base,y_base,z_base);
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
     cv::arrowedLine(chessBoard,cntr,y,CV_RGB(0,255,0),2,cv::LINE_8,0,0.1);
     cv::imshow("Chess board",chessBoard);
    cv::waitKey(0);
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
        std::vector<std::string> param1{"d",            "Distance to scene in base positive             [m]"};
        std::vector<std::string> param2{"f_m",          "Focal length in meter                          [m]"};
        std::vector<std::string> param3{"f_p",          "Focal length in pixles                         [pixles]"};

        std::vector<std::string> param4{"yaw",          "UAV yaw in base pose                           [rad]"};
        std::vector<std::string> param5{"pitch",        "UAV pitch in base pose                         [rad]"};
        std::vector<std::string> param6{"roll",         "UAV roll in base pose                          [rad]"};
        std::vector<std::string> param7{"x",            "UAV Base pose coordinate in x-direction        [m]"};
        std::vector<std::string> param8{"y",            "UAV Base pose coordinate in y-direction        [m]"};
        std::vector<std::string> param9{"z",            "UAV Base pose coordinate in z-direction        [m]"};


        validParameters_.push_back(param0);
        validParameters_.push_back(param1);
        validParameters_.push_back(param2);
        validParameters_.push_back(param3);
        validParameters_.push_back(param4);
        validParameters_.push_back(param5);
        validParameters_.push_back(param6);
        validParameters_.push_back(param7);
        validParameters_.push_back(param8);
        validParameters_.push_back(param9);
        validParameters = validParameters_; //Save as object attribute
        std::vector<std::vector<int>> validConfigurations_;
        std::vector<int> conf0{0,1};//sceneWidth and distance to base scene
        std::vector<int> conf1{1,2};//distance to base scene in base pose and focal length in meters
        std::vector<int> conf2{1,3};//distance to base scene in base pose and focal length in pixles
        std::vector<int> conf3{0,2};//scene width in [m] focal length in [m]
        std::vector<int> conf4{0,3};//scene width in [m] focal length in [pixles]
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
        d = abs(value);
    }
    /*else if(parameterName == "z"){
        z_base = value; //Only valid if base plane is horizontal and coplanar with base camera
    }
    else if(parameterName == "angle"){
        angle = value;
    }
    else if(parameterName == "f_m"){
        f_m = value;
    }
    else if(parameterName == "f_p"){
        f_p = value;
    }*/
    else if(parameterName == "y"){
        y_base = value; //Only valid if base plane is horizontal and coplanar with base camera
    }
    else if(parameterName == "x"){
        x_base = value; //Only valid if base plane is horizontal and coplanar with base camera
    }
    else if(parameterName == "z"){
        z_base = value; //Only valid if base plane is horizontal and coplanar with base camera
    }
    else if(parameterName == "yaw"){
        yaw_base = value; //Only valid if base plane is horizontal and coplanar with base camera
    }
    else if(parameterName == "pitch"){
        pitch_base = value; //Only valid if base plane is horizontal and coplanar with base camera
    }
    else if(parameterName == "roll"){
        roll_base = value; //Only valid if base plane is horizontal and coplanar with base camera
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
    return 1;
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
        /*
        Begin with implementing case 0. scenewidth, d must be set
        */
        case 0:{
            if(d!=0 && sceneWidth!=0 && N!=0){//If the necessary attributes are defined
                //Set base pose
                setBasePose();
                setKMat();//Set K-mat according to default configuration, d,sceneWidth, N
            }else{
                std::cout << "The required parameters for configuration "<< configuration << " are not set." << std::endl;
                return 0;
            }
            break;
        }
        default:{
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
    K_inv = K.inv();//Perform inversion
    return 1;
}

/* Re-calculates the requested UAV coordinates in global coordinate system to corresponding
 * translation and rotation matrix of the camera as realted to the base pose
 * Coordinates are to be given ass [yaw,pitch,roll], [z,y,x] of UAV expressed in global coordinate system
 * Finally the getWarpedImage method is called to calculate the corresponding warped view
 */
cv::Mat simulatePose::uav2BasePose(std::vector<float> angles,std::vector<float> t){
//Define the given coordinates and angles in matrix form
    float yaw = angles[0];
    float pitch = angles[1];
    float roll = angles[2];
    cv::Mat_<float> R_shifted = R1.t()*getTotRot(yaw,pitch,roll);
    float x = t[0];
    float y = t[1];
    float z = t[2];
    cv::Mat_<float> t_shifted = coord2CMat(x,y,z) - t1;

    cv::Mat out = getWarpedImage(R_shifted,t_shifted);
    return out;
}
/*Perform the warping of the image
 * Input: Coordinate and Rotation matrix of the camera as related to the base pose
 */
cv::Mat simulatePose::getWarpedImage(std::vector<float> angles,std::vector<float> t){
    cv::Mat out;
    if((angles.size()!=3) ||t.size()!=3){
        std::cout << "Not valid coordinates" << std::endl;
        return out;
    }
    float yaw = angles[0];
    float pitch = angles[1];
    float roll = angles[2];
    float x = t[0];
    float y = t[1];
    float z = t[2];
    cv::Mat R_z = getZRot(yaw);
    cv::Mat R_y = getYRot(pitch);
    cv::Mat R_x = getXRot(roll);                      //Rotations of camera pose 2

    cv::Mat_<float> t2 = cv::Mat_<float>::zeros(3,1); //Coordinate of camera pose 2
    t2(0,0) = x;
    t2(1,0) = y;
    t2(2,0) = z;

    //Define T-matrix according to 4.8 Wadenbaeck
    cv::Mat_<float> T = cv::Mat_<float>::eye(3,3) - t2*v.t();

    //Define Homography according to 4.7. Wadenbaeck. Apply the inverse.
    // x,y,z,yaw are positive. roll, pitch are negative
    cv::Mat_<float> H = K *R_x*R_y*R_z*T* K_inv;
    cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,0);

    // Define homography according to 4.7 but no inverse. Instead define it directly
    //No inverse. idea: put image coordinates first through zrot, yrot, xrot, then translation No quite sure why inverse though
    //cv::Mat_<float> H = K  *T.inv()*R_z.t()*R_y.t()*R_x.t()* K_inv;
    //cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,0);

    return out;
}
/* Overloaded version of warp perspective that accepts arguments in matrix form
*/
cv::Mat simulatePose::getWarpedImage(cv::Mat_<float> R2,cv::Mat_<float> t2){
    cv::Mat out;
    if((R2.rows!=3) || (R2.cols!=3) || (t2.rows!=3) || (t2.cols!=1)){
        std::cout << "Not valid size of pose Mats" << std::endl;
        return out;
    }
    //Define T-matrix according to 4.8 Wadenbaeck
    cv::Mat_<float> T = cv::Mat_<float>::eye(3,3) - t2*v.t();
    //Define Homography according to 4.7. Wadenbaeck. Apply the inverse.
    // x,y,z,yaw are positive. roll, pitch are negative
    cv::Mat_<float> H = K *R2*T* K_inv;
    cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,0);
    return out;
}
/*Gets a vector of float and returns a column vector in Mat_<float> form
*/
cv::Mat simulatePose::coord2CMat(float x, float y, float z){
    cv::Mat_<float> matrix = cv::Mat_<float>::zeros(3,1);
    matrix(0,0) = x;
    matrix(1,0) = y;
    matrix(2,0) = z;
    return matrix;
}
/* Define a complete rotation matrix from yaw,pitch,roll
    R_tot = Rx*Ry*Rz
    t_rot = R_tot*t
*/
cv::Mat simulatePose::getTotRot(float yaw, float pitch, float roll){
    cv::Mat_<float> R_tot = getXRot(roll)*getYRot(pitch)*getZRot(yaw);
    return R_tot;
}

/* Functions for defining roll, pitch, and yaw rotation matrices
 * Increase speed by passing reference and edit in place?
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
    float cosY = std::cos(pitch);
    cv::Mat_<float> R_y = cv::Mat_<float>::zeros(3,3);
    R_y(0,0) = cosY;
    R_y(0,2) = sinY;
    R_y(1,1) = 1;
    R_y(2,0) = -sinY;
    R_y(2,2) = cosY;
    return R_y;
}
cv::Mat simulatePose::getZRot(float yaw){
    float sinZ = std::sin(yaw);
    float cosZ = std::cos(yaw);
    cv::Mat_<float> R_z = cv::Mat_<float>::zeros(3,3);
    R_z(0,0) = cosZ;
    R_z(0,1) = sinZ;
    R_z(1,0) = -sinZ;
    R_z(1,1) = cosZ;
    R_z(2,2) = 1;
    return R_z;
}
