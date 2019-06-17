#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "vopos.hpp"


/*
Master positioning class constructor. Is called after the inherited classes are constructed
Inherited class constructors are called with relevant arguments after the ":" in the initialization list.
*/
pos::positioning::positioning(int opticalFlow_mode,
                                int visualOdometry_mode,
                                int arucoDictionary,                            //Example: cv::aruco::DICT_4X4_50
                                int maxID,
                                std::string anchorPath,
                                int flowGrid,
                                cv::Rect2f roi_rect,
                                cv::Mat_<float> K,
                                cv::Mat_<float> T):
        ang::angulation(maxID,anchorPath),                          //Angulation constructor
        of::opticalFlow(opticalFlow_mode,flowGrid,roi_rect.width),  //Optical flow constructor
        vo::planarHomographyVO(visualOdometry_mode),                //Homography constructor
        roi(roi_rect)                                               //Assign argument to positioning attribute
{
    //Set some settings for angulation object
    ang::angulation::setKmat(K);
    ang::angulation::setTmat(T);
    minAnchors = 2;//Descider wether to try angulation or not
    //Initialize the aruco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(arucoDictionary);
    //Set some settings for Optical Flow object
    of::opticalFlow::setDefaultSettings();
    //Set some settings for Visual Odometry object
    vo::planarHomographyVO::setKmat(K,roi_rect);
    vo::planarHomographyVO::setTmat(T);
    vo::planarHomographyVO::setDefaultSettings();
}
/*
Process the given data and update the position and yaw
*/
int pos::positioning::process(int mode,cv::Mat& frame, float dist,float roll, float pitch,float& yaw, cv::Mat_<float>& pos){
    //Aruco detect and draw
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);



    return 1;
}
/*
Process the given data and update position and yaw. Also illustrate by drawing on the outputFrame mat
dist - Distance from camera to the flow field plane.
*/
int pos::positioning::processAndIllustrate(int mode,cv::Mat& frame, cv::Mat& outputFrame,int illustrate_flag,float dist,float roll, float pitch,float& yaw, cv::Mat_<float>& pos){
    static cv::Mat subPrevFrame; //Static init of prev subframe for optical flow field
    //Aruco detect and draw
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
    //Draw markers
    cv::aruco::drawDetectedMarkers(outputFrame, corners, ids, CV_RGB(0,250,0));
    //Only do angulation if at least two known anchors are visible
    int status = 0;
    int returnMode = pos::RETURN_MODE_AZIPE;
    std::vector<cv::Mat_<float>> q;
    std::vector<cv::Mat_<float>> v;
    if(ids.size()>=minAnchors){
        std::vector<bool> mask;
        status = ang::angulation::calculateQV(corners,ids,mask,pos,yaw,roll,pitch,q,v);
    }
    if(status != ang::AZIPE_SUCCESS){//If not successful (Can fail due to either no known anchors or some azipe error)
        //Get flow field
        std::vector<cv::Point2f> features;
        std::vector<cv::Point2f> updatedFeatures; //The new positions estimated from KLT
        of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures);
        //Draw flow field arrows
        float scale = 10;
        cv::Point2f focusOffset(roi.x,roi.y);
        drawArrows(outputFrame,features,updatedFeatures,scale,focusOffset);
        cv::rectangle(outputFrame,roi,CV_RGB(255,0,0),2,cv::LINE_8,0);
        bool success = vo::planarHomographyVO::process(features,updatedFeatures,roll,pitch,dist,pos,yaw);
        //Is there at least one available anchor so that we can to projection fusing?
        if(success && ids.size()>0){
            projectionFusing(pos,q,v,mask,yaw,roll,pitch);
            returnMode = pos::RETURN_MODE_PROJ;
        }
        else if(success){returnMode = pos::RETURN_MODE_VO;}
        else{returnMode = pos::RETURN_MODE_INERTIA;}
    }


    frame(roi).copyTo(subPrevFrame);//Copy the newest subframe to subPrevFrame for use in next function call
    return returnMode;
}
/* Draws a closed loop between all given points
 */
void pos::positioning::drawLines(cv::Mat& img,std::vector<cv::Point2f> points,cv::Point2f offset){
    int size = points.size();
    int i = 0;
    while(i<(size-1)){
        cv::line(img,offset+points[i],offset+points[i+1],CV_RGB(255,0,0),2,cv::LINE_8,0);
        i++;
    }
    cv::line(img,offset+points[i],offset+points[0],CV_RGB(255,0,0),2,cv::LINE_8,0);//close loop
}

/* Fuses the VO position estimation with incomplete angulation measurement (Angular measurement to single anchor)
 * This is done by obtaining an expression for a possible 3D line where the vehicle can be positioned, assuming known pose, anchor position and angular measurement to anchor
 * (Pose obtained from IMU (Roll, Pitch), VO-algorithm (Yaw))
 * The initial 3d coordinate estimation from VO is projected onto the 3D line.
 * Theory wise will this prevent drift. But is sensitive of roll, pitch, yaw errors. Yaw may be pretty good estimated with VO. roll, pitch
 * is obtained from Gyro + Accelerometer to improve quality
 *

    cv::Mat_<float>& pos  : Inputoutputarray: Contains VO pos estimation. Will be updated with improved estimation
    std::vector<cv::Mat_<float>> q: Input array containing coordinate of known anchor.
    std::vector<cv::Mat_<float>> v; Input array containging uLOS vectors from vehicle to known anchor(s)
    std::vector<uchar> mask       : Input array containing a mask to choose which element of q to use. (Will choose first 1)
    float yaw, roll, pitch: Input floats :Pose info
 */
void pos::positioning::projectionFusing(cv::Mat_<float>& pos,std::vector<cv::Mat_<float>> q, std::vector<cv::Mat_<float>> v, std::vector<bool> mask,
                        float yaw, float roll,float pitch){
    //Create R mat describing the pose of the vehicle (Instead directly create inverted R-mat)
    //cv::Mat_<float> R = getXRot(roll)*getYRot(pitch)*getZRot(yaw);
    cv::Mat_<float> R_t = getZRot(-yaw)*getYRot(-pitch)*getXRot(-roll);
    //Find first element with known anchor
    std::vector<bool>::iterator it = std::find(mask.begin(), mask.end(), true);
    if(it!=mask.end()){
        int index = std::distance(mask.begin(), it);
        //Calculate v_tilde, which is the uLos vector expressed in global frame in negative direction. I.e vector pointing from anchor to vehicle
        cv::Mat_<float> v_tilde = -R_t*v[index];
        cv::Mat_<float> qt = t-q;//Vector from q to t (Anchor to estimated vehicle position)
        cv::Mat_<float> proj = q + qt.dot(v_tilde)/v_tilde.dot(v_tilde) * v_tilde;//Projected coordinate
        std::cout << "proj dim: " << proj.size() << std::endl;

    }

}
/* Functions for defining roll, pitch, and yaw rotation matrices
 * Increase speed by passing reference and edit in place?
 */
cv::Mat pos::positioning::getXRot(float roll){
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
cv::Mat pos::positioning::getYRot(float pitch){
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
cv::Mat pos::positioning::getZRot(float pitch){
    float sinY = std::sin(pitch);
    float cosY = std::cos(pitch);
    cv::Mat_<float> R_y = cv::Mat_<float>::zeros(3,3);
    R_y(0,0) = cosY;
    R_y(0,1) = -sinY;
    R_y(1,0) = sinY;
    R_y(1,1) = cosY;
    R_y(2,2) = 1;
    return R_y;
}
