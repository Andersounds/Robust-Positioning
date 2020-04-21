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
TODO:   -Let user define general global coordinate at base scene DONE
        -Relate UAV to camera pose with T matrix. begin with only rotation around z
        -Let global coordinate system always be aligned with base image coordinate system.


Configurations:
        -0: Specify base image physical dimensions and camera base pose - K matrix is automatically calculated
        -1: Specify a camera matrix, base image physical dimensions (x or y) and provide base image
            base image can and should be high resolution. when generating a warped image, a new scaled camera matrix is used. returned image is then downscaled to correct resolution

*/
simulatePose::simulatePose(void){
    //Variables for definition of K matrix
    //N = 0;
    //cx = 0;
    //cy = 0;
    //f_p = 0;
    //Default T mat is no rotation around z.
    T_z = cv::Mat_<float>::eye(3,3);              //Base conversion from UAV frame to camera frame PURE ROTATION
    //Physical scale variables
    sceneWidth = 0;
    //angle = 0;
    //f_m = 0;
    //gamma=0;
    //Define scene plane as expressed in base pose. perpendiculat to z-axis
    //d  = 0;                             // 0 is non-accepted value. user must define via some configuration
    //base plane normal in global sys
    v = cv::Mat_<float>::zeros(3,1);
    v(2,0) = 1;
    yaw_offset = 0;
}

    /*Sets the default chessboard pattern as base scene
    */
void simulatePose::setBaseScene(int blockSize,int rowsOfBoxes,int colsOfBoxes){
    baseScene = getChessboard(blockSize,rowsOfBoxes,colsOfBoxes);
}
    /* Specify own image for basescene
     */
void simulatePose::setBaseScene(cv::Mat baseScene_){
        baseScene_.copyTo(baseScene);
}
void simulatePose::setBaseSceneWidth(float BaseSceneWidth_){
    sceneWidth = BaseSceneWidth_;
}
void simulatePose::setKMat(cv::Mat_<float>K_){
    K_.copyTo(K_specified);
    camera_res_x = 2*K_specified(0,2);
    camera_res_y = 2*K_specified(1,2);
}
void simulatePose::setTMat(cv::Mat_<float>T_){
    T_.copyTo(T_z);
}
void simulatePose::setYawOffset(float offset){
    yaw_offset = offset;
}
/*
This function will be used to define the relationship between the global coordinate system and the base pose
 */
void simulatePose::setBasePose(float x0,float y0,float z0, float yaw0){
    //Base rotation of camera 1. i.e the R matrix of the camera that should achieve the non-distorted base projection
    R1 = getTotRot(yaw0,0,0);    //Set base rotation matrix. no roll pitch
    //Base coordinate of camera 1. i.e the t of the camera that should achieve the non-distorted base projection
    t1 = coord2CMat (x0,y0,z0);
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

/* This method initializes the simulation environment according to the configuration specified
 *
 */
int simulatePose::init(void){
    //Check that K is defined, base scene is defined, scene width is defined
    if(!K_specified.empty() && !baseScene.empty() && sceneWidth!=0){
        //Pad basescene in either x or y direction to get x/y ratio as the specified camera matrix
        int baseSceneCols = baseScene.cols;
        int baseSceneRows = baseScene.rows;
        float s1 = ((float)baseSceneCols)/(2*K_specified(0,2)); //Scale factor from specified camera resolution to basescene resolution
        float s2 = ((float)baseSceneRows)/(2*K_specified(1,2));
        //If scale factors are not equal, apply the largest one
        if(s1<s2){// Widen base scene
            int baseSceneCols_new = (int)((2*K_specified(0,2))*s2);//New correct pixel count x direction
            sceneWidth *=((float)baseSceneCols_new)/((float)baseSceneCols); //Correct physical width of base scene [m]
            baseSceneCols = baseSceneCols_new;
        }else if(s2<s1){ //Heighten base scene
            baseSceneRows = (int)((2*K_specified(1,2))*s1);
        }
        if(s1!=s2){
            //Redefine the base scene by padding.
            cv::Mat baseScene_corrected = cv::Mat::zeros(cv::Size(baseSceneCols,baseSceneRows), baseScene.type());
            cv::Rect roi(0,0,baseScene.cols,baseScene.rows);//Roi covering whole provided base scene
            baseScene(roi).copyTo(baseScene_corrected(roi));
            baseScene.release();
            baseScene_corrected.copyTo(baseScene);
            //The basescene now has the ratio of specified camera
        }
        //Scale K_specified to the resolution of base scene
        float cx_specified = K_specified(0,2);
        float cx_sim =((float)baseSceneCols)/2;
        KmatScaleFactor = cx_sim/cx_specified;
        K = K_specified*KmatScaleFactor; //The K mat that is used in simulation
        K(2,2) = 1; //norm last element to 1 (!)
        K_inv = K.inv();

        //Calculate base pose of camera
        //Note that the assumes square pixles

        float x0 = sceneWidth/2;
        float pix2m = sceneWidth/((float)baseSceneCols); //scaling factor pixles to meters
        float y0 = baseSceneRows*pix2m/2;
        //distance to base scene in base pose using similar triangles half resolution in x, focal length in pixles, and half scene width in m gives distance in m
        d = x0*K(0,0)/K(0,2);
        float z0 = -d;
        float yaw0 = yaw_offset; //Image coordinate system is comletely aligned with base scene (global) coordinate system
        std::cout << "Simulatepose::init TODO: yaw0 is 0. what do?" << std::endl;
        std::cout << "x0: " << x0 << std::endl;
        std::cout << "y0: " << y0 << std::endl;
        std::cout << "z0: " << z0 << std::endl;
        //Set base pose of camera
        setBasePose(x0,y0,z0,yaw0);
    }else{
        std::cout << "The required parameters are not set. " << std::endl;
        std::cout << "Call the following functions before init()"<< std::endl;
        std::cout << "setBaseScene          - either provide image or chess board size" << std::endl;
        std::cout << "setBaseSceneWidth     - width of base scene (x dir) expressed in meters" << std::endl;
        std::cout << "setKMat               - K mat of simulated camera" << std::endl;
        return 0;
    }
    return 1;
}
/*
Takes global coordinates x,y,z and angles yaw,pitch,roll (in that order) OF UAV. and simulates a camera capture
*/
cv::Mat simulatePose::simulateView(std::vector<float> angles,std::vector<float>t){

    if((angles.size()!=3)||(t.size()!=3)){
        std::cout << "simulatepose::simulateView.Wrong input" << std::endl;
        cv::Mat X;
        return X;
    }
    //Warp image to simulate view
    cv::Mat fullScaleImage = uav2BasePose(angles,t);
    //Scale image to resolution of camera
    //KmatScaleFactor
    cv::Mat correctedResolutionImage = cv::Mat::zeros(cv::Size(camera_res_x,camera_res_y), baseScene.type());;
    cv::resize(fullScaleImage, correctedResolutionImage,correctedResolutionImage.size(),0,0,CV_INTER_AREA);
    return correctedResolutionImage;
}

/* Re-calculates the requested UAV coordinates in global coordinate system to corresponding
 * translation and rotation matrix of the camera as realted to the base pose
 * Coordinates are to be given ass [yaw,pitch,roll], [z,y,x] of UAV expressed in global coordinate system
 * Finally the getWarpedImage method is called to calculate the corresponding warped view

 Rename this function or separate from getwarpedimage?? what is best?
 */
cv::Mat simulatePose::uav2BasePose(std::vector<float> angles,std::vector<float> t){
//Define the given coordinates and angles in matrix form
    float yaw = angles[0];
    float pitch = angles[1];
    float roll = angles[2];
    /*  Hardcoded relationship between image coordinate system and UAV coordinate system
        UAV shall be rotated in order yaw->pitch->roll

        |_UAV_direction_|_Image_direction_|   -> |_UAV_Rotation_|_Image_rotation_|
        |       +X      |        -Y       |   -> |     Rroll    |    Ry(-roll)   |
        |       +Y      |        +X       |   -> |     Rpitch   |    Rx(pitch)   |
        |_______+Z______|________+Z_______|   -> |_____Ryaw_____|____Rz(yaw)_____|

        To apply a rotation Rz(yaw)*Rx(pitch)*Ry(-roll) = R_image on the image is
        equivalent with applying R_image.inv()=R_scene on the scene, which is what cv::warpPerspective does

        1. R_image.inv() = R_image.t()                 ...due to being a rotation matrix
        2. (Rz*Ry*Rx).t() = Rx.t()*Ry.t()*Rz.t()       ...basic algebra. note that order reverses
        Using property 1 again we can state that we shall apply

                            Rtot = R_roll.t()*R_pitch.t()*R_yaw.t()
                                 = Ry(roll)*Rx(-pitch)*Rz(-yaw)

        to perspective transformation.


        --Absolute first we should do is to rotate uav with R1.invers
        R1.inv*Rz*Rx*Ry.inv

        Ry*Rx.inv*Rz.inv*R1
    */
    cv::Mat R_yaw = getZRot(-yaw);
    cv::Mat R_pitch = getXRot(-pitch);
    cv::Mat R_roll = getYRot(roll);
    //cv::Mat R_shifted = R_roll*R_pitch*R_yaw;
    cv::Mat R_shifted = R_roll*R_pitch*R_yaw*R1;



    //cv::Mat_<float> R_shifted = R1.t()*getTotRot(yaw,pitch,roll);
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
 /*
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
    //cv::Mat_<float> T = cv::Mat_<float>::eye(3,3) - t2*v.t();
    cv::Mat_<float> T = cv::Mat_<float>::eye(3,3) - t2*v.t()/d*KmatScaleFactor;//Scale with initial d.

    //Define Homography according to 4.7. Wadenbaeck. Apply the inverse.
    // x,y,z,yaw are positive. roll, pitch are negative

    cv::Mat_<float> H = K *R_x*R_y*R_z*T* K_inv;
    cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,0);

    // Define homography according to 4.7 but no inverse. Instead define it directly
    //No inverse. idea: put image coordinates first through zrot, yrot, xrot, then translation No quite sure why inverse though
    //cv::Mat_<float> H = K  *T.inv()*R_z.t()*R_y.t()*R_x.t()* K_inv;
    //cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,0);
    return out;
}*/
/* Overloaded version of warp perspective that accepts arguments in matrix form
*/
cv::Mat simulatePose::getWarpedImage(cv::Mat_<float> R2,cv::Mat_<float> t2){
    cv::Mat out;
    if((R2.rows!=3) || (R2.cols!=3) || (t2.rows!=3) || (t2.cols!=1)){
        std::cout << "Not valid size of pose Mats" << std::endl;
        return out;
    }

    //Define T-matrix according to 4.8 Wadenbaeck
    //cv::Mat_<float> T = cv::Mat_<float>::eye(3,3) - t2*v.t();
    cv::Mat_<float> T = cv::Mat_<float>::eye(3,3) - t2*v.t()/d;//Scale with initial d.
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
    R_z(0,1) = -sinZ;
    R_z(1,0) = sinZ;
    R_z(1,1) = cosZ;
    R_z(2,2) = 1;
    return R_z;
}
