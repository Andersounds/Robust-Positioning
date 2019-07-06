#include <iostream>
#include <opencv2/opencv.hpp>
#include "videoStream.hpp"
#include "videoStream.cpp"

/*
 *  This file if for rpi compilation only, as it needs raspicam installed and available
 *  It includes everything the mac-version by making a direct #include "videoStream.cpp"
 *  But in this file also include other funtion prototypes
 will this work? Will i not need the raspicam class in prototype??
 */

#include "../include/raspicam-0.1.6/src/raspicam_cv.h"

//Default constructor. does not do anything. onl called when Streamer does NOT use picam
robustPositioning::piCamStreamer::piCamStreamer(void){

}

robustPositioning::piCamStreamer::piCamStreamer(double prop){
    getImage(cv::Mat temp);
}

float robustPositioning::piCamStreamer::getImage(cv::Mat& frame){
    static bool initialized = false;
    static raspicam::RaspiCam_Cv RPICamera;
    if(!initialized){
        RPICamera.open();
        if(RPICamera.isOpened()){
            std::cerr<<"Error opening camera"<<std::endl;
            cv::Mat emptyFrame;
            emptyFrame.copyTo(frame);
            return -1;
        }else{
            cout<<"Connected to camera ="<<RPICamera.getId() <<endl;
            initialized = true;
            return -1;
        }
    }

    RPICamera.grab();
    RPICamera.retrieve(frame);

    return 1;
}
