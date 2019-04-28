#ifndef SIMULATEPOSE_H
#define SIMULATEPOSE_H

//namespace sp{

class simulatePose{
public:
    cv::Mat baseScene;
    cv::Mat_<float> K;                      //K mat that satisfies specifications
    cv::Mat_<float> K_inv;                  //Inverse of K mat that satisfies specifications
    cv::Mat_<float> T;                      //T matrix. Camera orientation as related to UAV orientation
    cv::Mat_<float> R1;                     //Base rotation of UAV
    cv::Mat_<float> t1;                     //Base coordinate of UAV
    simulatePose(void);                     //Constructor that sets default values to attributes
    void setBaseScene(int,int,int);         //Defines default chessboard pattern as base scene
    void setBaseScene(cv::Mat);  //Defines provided mat as base scene (And draws the global coordinate system?)
    int setParam(std::string,float);        //Define parameters one by one. can be given in any order
    int setParam(void);
    int init(int);                          //Initialize physical simulation environment with the specified configuration
    cv::Mat getWarpedImage(std::vector<float>,std::vector<float>);              //Perform perspective warp of base scene
private:
    float cx;                               //Pixel offset in x in K mat
    float cy;                               //Pixel offset in y in K mat
    float d;                                //d in base plane definition. -In this implementation, base plane is horizontal at z=0
    float z_base;                           //Base coordinate of camera 1 in z-direction
    float y_base;                           //Base coordinate of camera 1 in y-direction
    float x_base;                           //Base coordinate of camera 1 in x-direction
    float yaw_base;                         //Base yaw of camera 1
    float pitch_base;                       //Base pitch of camera 1
    float roll_base;                        //Base roll of camera 1
    cv::Mat_<float> v;                      //base plane normal. (as expressed in camera 1:s coordinate system)
    float N;                                //Camera resolution in x-direction [pixles]
    float sceneWidth;                       //Scene width in x-direction       [m]
    float angle;                            //Viewing angle of camera in x-direction
    float f_m;                              //Focal length of camera in meter  [m]
    float f_p;                              //Focal length of camera in pixles [-]
    float gamma;//NECESSARY? Pixel density of camera [pixles/meter]
    std::vector<std::vector<std::string>> validParameters;//The parameters that are valid to use to calculate the simulation camera
    std::vector<std::vector<int>> validConfigurations;      //The configurations of parameters that are valid to set up the complete simulation environment
    void setBasePose(std::vector<float>,std::vector<float>);                    //Define the base pose of the camera in global coordinate system
    cv::Mat getChessboard(int,int,int);     //Creates a chessboard image of specified size
    cv::Mat getXRot(float);            //Defines rotation matrices
    cv::Mat getYRot(float);
    cv::Mat getZRot(float);
    int setKMat(void);  //Uses some given parameters? creates a suitable K mat
};
//}
#endif
