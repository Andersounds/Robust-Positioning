#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "positioning.hpp"


/*
Master positioning class constructor. Is called after the inherited classes are constructed
Inherited class constructors are called with relevant arguments after the ":" in the initialization list.
*/
pos::positioning::positioning(int illustrate_mode,
                                int opticalFlow_mode,
                                int visualOdometry_mode,
                                int arucoDictionary,                            //Example: cv::aruco::DICT_4X4_50
                                int maxID,
                                std::string anchorPath,
                                int flowGrid,
                                cv::Rect2f roi_rect,
                                cv::Mat_<float> K,
                                cv::Mat_<float> T):
        ang::angulation(maxID,anchorPath),
        of::opticalFlow(opticalFlow_mode,flowGrid,roi_rect.width),
        vo::planarHomographyVO(K,T,visualOdometry_mode,roi_rect),
        roi(roi_rect) //Assign argument to positioning attribute
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
    vo::planarHomographyVO::setDefaultSettings();
}

int pos::positioning::process(cv::Mat& frame, float roll, float pitch,float& yaw, cv::Mat_<float>& pos){
    //Aruco detect and draw
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);



    return 1;
}

int pos::positioning::process(cv::Mat& frame, cv::Mat& outputFrame,float roll, float pitch,float& yaw, cv::Mat_<float>& pos){
    //Aruco detect and draw
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
    //Only do angulation if at least two known anchors are visible
    if(ids.size()>=minAnchors){
        bool success = ang::angulation::calculate(corners,ids,pos,yaw,roll,pitch);
    }

    //Draw markers
    cv::aruco::drawDetectedMarkers(outputFrame, corners, ids, CV_RGB(0,250,0));
    //std::cout << "success: " << success << std::endl;
    return 1;
}
