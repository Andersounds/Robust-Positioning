#include <iostream>
#include <opencv2/opencv.hpp>
/*
Refactor this into h and c files with redefinition guards
*/

/*
This class generates a visual simulation environment to evaluate VO-algorithm

Features:
-   Provide a floor image separately or use the default chessboard pattern
-   Define parameters to relate to physical coordinates. Define camera position and pose in undistorted image case
        Example: Camera coordinate T and rotation R, with focal length f (or wide angle) gives undistorted projection
    The object the constructs the camera K that fulfills the specifications
-   For each new physical camera coordinate and pose [m],[rad], the object calculates the corresponding image warp
*/
class simulatePose{
public:
    cv::Mat baseScene;
    cv::Mat_<float> K;//Normalized K mat for image.
    simulatePose(cv::Mat_<float> K_){
        K=K_;
    }
    /*Sets the default chessboard pattern as base scene
    */
    void setBaseScene(int blockSize,int rowsOfBoxes,int colsOfBoxes){
        baseScene = getImage(blockSize,rowsOfBoxes,colsOfBoxes);
        float cx = ((float) baseScene.cols) /2;
        float cy = ((float) baseScene.rows) /2;
        //K = getKMat(cx,cy);
    }
    /* Specify own image for basescene
    */
    void setBaseScene(cv::Mat baseScene_){
        baseScene = baseScene_;
        float cx = ((float) baseScene.cols) /2;
        float cy = ((float) baseScene.rows) /2;
        //K = getKMat(cx,cy);
    }
    /* TODO */
    /* This function is used to specify which camera pose that projects to
     * the undistorted base scene. It defines the R1, T1 that are initial camera pose,
     * and constructs the camera K that fulfills the specifications
     */
    void setBasePose(void){


    }
    /*Returns the warped baseScene. coordinates are to be given ass [roll,pitch,yaw], [x,y,z]
     * Right now no z??
     */
    cv::Mat getWarpedImage(std::vector<float> angles,std::vector<float> t){
        cv::Mat out;
        if((angles.size()!=3) ||t.size()!=3){
            std::cout << "Not valid coordinates" << std::endl;
            return out;
        }
        float roll = angles[0];//0.001;
        float pitch = angles[1];
        float yaw = angles[2];
        float x = t[0];
        float y = t[1];
        float z = t[2];
        cv::Mat R_x = getXRot(roll);
        cv::Mat R_y = getYRot(pitch);
        cv::Mat R_z = getZRot(yaw);
        cv::Mat R_2 = R_x*R_y*R_z;
        cv::Mat R_1 = getXRot(3.1415);//Ursprunglig rot av kamera. väljer 180 grader runt X-x.riktning bevaras
        cv::Mat_<float> t1 = cv::Mat_<float>::zeros(3,1);
        t1(2,0) = 1;//1 meter i z-riktning ursprungligen
        cv::Mat_<float> t2 = cv::Mat_<float>::zeros(3,1);
        t2(0,0) = x;//
        t2(1,0) = y;//
        t2(2,0) = t1(2,0);// Vi antar planar motion. for now. bild 2 är på samma z-höjd som ursprungligen
        //Below is b and A with definition P=KR[I,-t]
        cv::Mat_<float> b = t1 - R_1.t()*R_2*t2;
        cv::Mat_<float> A = R_1.t()*R_2;
        //Below is b and A with definition P=K[R,t]
        //cv::Mat_<float> b = R_1.t()*(t2-t1);
        //cv::Mat_<float> A = R_1.t()*R_2;

        float d = t1(2,0);//Distance from camera 1 to plane (positive as plane is in front of camera 1, i think)
        cv::Mat_<float> v = cv::Mat_<float>::zeros(3,1);
        v(2,0) = 1;//Chackbrädets normal uttryckt i globalt koordinatsystem
        cv::Mat_<float> v_temp = R_1*v;//
        v=v_temp;



        cv::Mat_<float> H_tilt = K * R_x*R_y * K.inv(); // Pure rotation
        cv::Mat_<float> H_trans = K * (A - b*v.t()/d) * K.inv(); // According to 3.8
        //cv::Mat_<float> H = (A - b*v.t()/d) * K.inv();
        //cv::Mat_<float> H = (A - b*v.t()/d) ;
        //cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,0);
        cv::Mat_<float> H = H_tilt*H_trans;
        cv::warpPerspective(baseScene,out,H,baseScene.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,0);

        //cv::perspectiveTransform()
        return out;
    }
    /*Returns a linspace sequence starting from start with length no of steps of size step
     *
     */
    std::vector<float> getPath(float start,float step,float length){
        std::vector<float> path;
        for(float i=0;i<length;i++){
            path.push_back(step*i+start);
        }
        return path;
    }
private:
    cv::Mat getImage(int blockSize,int rowsOfBoxes,int colsOfBoxes){
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

    cv::Mat getXRot(float roll){
        float sinX = std::sin(roll);
        float cosX = std::cos(roll);//std::sin(-roll);
        cv::Mat_<float> R_x = cv::Mat_<float>::zeros(3,3);
        R_x(0,0) = 1;
        R_x(1,1) = cosX;
        R_x(1,2) = -sinX;
        R_x(2,1) = sinX;
        R_x(2,2) = cosX;
        return R_x;
    }

    cv::Mat getYRot(float pitch){
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
    cv::Mat getZRot(float yaw){
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
    /*Update this to correct
    */
    cv::Mat getKMat(float cx,float cy){
        cv::Mat_<float> K_ = cv::Mat_<float>::zeros(3,3);
        float fx = 1;//260;
        float fy = 1;//260;
        //float cx = 376;//Change pixel center according to focus area
        //float cy = 240;
        K_(0,0) = fx;
        K_(1,1) = fy;
        K_(0,2) = cx;
        K_(1,2) = cy;
        K_(2,2) = 1;//1.0;
        return K_;
    }
    cv::Mat getTransMat(float x,float y,float z){
        cv::Mat_<float> T = cv::Mat_<float>::eye(3,3);
        T(0,2) = x*z; //Scale x and y with z coordinate
        T(1,2) = y*z;
        T(2,2) = z;
        return T;
    }
};
