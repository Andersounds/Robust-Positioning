#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "positioning.hpp"


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
    if(ids.size()>=minAnchors){
        bool success = ang::angulation::calculate(corners,ids,pos,yaw,roll,pitch);
    }

    //Get flow field
    std::vector<cv::Point2f> features;
    std::vector<cv::Point2f> updatedFeatures; //The new positions estimated from KLT
    of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures);
    bool odometerSuccess = vo::planarHomographyVO::process(features,updatedFeatures,roll,pitch,dist,pos,yaw);



    frame(roi).copyTo(subPrevFrame);//Copy the newest subframe to subPrevFrame for use in next function call
    return 1;
}
