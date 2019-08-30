#ifndef SIMULATEPOSE_H
#define SIMULATEPOSE_H

//namespace sp{

class simulatePose{
public:
    simulatePose(void);                     //Constructor that sets default values to attributes
    void setBaseScene(int,int,int);         //Defines default chessboard pattern as base scene
    void setBaseScene(cv::Mat);  //Defines provided mat as base scene (And draws the global coordinate system?)
    void setBaseSceneWidth(float);//Set width in x dir of base scene [m]
    void setKMat(cv::Mat_<float>);  //Set K mat of camera
    void setTMat(cv::Mat_<float>);  //Set T mat of UAV/camera.
    int init(void);                          //Initialize physical simulation environment with the specified configuration



    cv::Mat simulateView(std::vector<float>,std::vector<float>);// give x,y,z,roll,pitch,yaw (order) of UAV. returned image is simulated and scaled
    cv::Mat simulateView(cv::Mat_<float>,cv::Mat_<float>); //Instead give coordinate as mat and rotations as rotationmatrix

private:
    cv::Mat baseScene;
    cv::Mat_<float> K;                      //K mat that satisfies specifications
    cv::Mat_<float> K_inv;                  //Inverse of K mat that satisfies specifications
    cv::Mat_<float> K_specified;            //Specified K matrix. this is scaled to fit given base scene during simulation. Output is scaled down to this K mat again
    cv::Mat_<float> R1;                     //Base rotation of CAMERA
    cv::Mat_<float> t1;                     //Base coordinate of CAMERA
    cv::Mat_<float> T_z;                    //T matrix. Camera orientation as related to UAV orientation
    float d;                                //d in base plane definition.
    float sceneWidth;                       //Scene width in x-direction       [m]
    cv::Mat_<float> v;                      //base plane normal. (as expressed in camera 1:s coordinate system) - always same per definition
    int camera_res_x;
    int camera_res_y;
    float KmatScaleFactor;                  //Scale factor between provided K mat and simulation K mat
    void setBasePose(float,float,float,float);                    //Define the base pose of the UAV in global coordinate system
    cv::Mat coord2CMat(float, float, float);   //Vector<float> to column Mat_<float>

    cv::Mat uav2BasePose(std::vector<float>,std::vector<float>);//Recalculates uav pose to camera pose in relation to base pose
    cv::Mat getWarpedImage(std::vector<float>,std::vector<float>);              //Perform perspective warp of base scene
    cv::Mat getWarpedImage(cv::Mat_<float>,cv::Mat_<float>);//Overloaded version with other arguments
    cv::Mat getXRot(float);            //Defines rotation matrices
    cv::Mat getYRot(float);
    cv::Mat getZRot(float);

    cv::Mat getChessboard(int,int,int);     //Creates a chessboard image of specified size
    cv::Mat getTotRot(float, float, float); //Defines total rotation matrix
};
//}
#endif
