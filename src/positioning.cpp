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
                                cv::Rect2f roiSize,
                                cv::Mat_<float> K,
                                cv::Mat_<float> T):
        ang::angulation(maxID,anchorPath),
        of::opticalFlow(opticalFlow_mode,flowGrid,roiSize.width),
        vo::planarHomographyVO(K,T,visualOdometry_mode)
{
    //Initialize the aruco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(arucoDictionary);
    std::cout << "Edit the visualodometry-K-matrix to work with the roiSize here" << std::endl;
    //Or change the VO constructor so that it itself sets it upon initialization
}

int pos::positioning::process(cv::Mat frame, float roll, float pitch){
    //Aruco detect and draw
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
    return 1;
}
