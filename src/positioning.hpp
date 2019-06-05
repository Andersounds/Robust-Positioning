#ifndef POSITIONING_H
#define POSITIONING_H
//#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "../src/angulation.hpp"
#include "../src/opticalFlow.hpp"
#include "../src/homographyVO.hpp"
/*
This class implements a complete positioning functionality with fallback method, using Aruco as base and excluding Kalman fusing with IMU data

Init:
 - Specify which Aruco dictionary is to be used example cv::aruco::DICT_4X4_50
 - Read known locations of markers from specified csv-file
 -

Positioning:
1.      Get image and current readings or roll and pitch of vehicle
2.      Locate and decode visible ArUco-markers
2,5.    FUTURE: If not all are known, perform PnP estimation to add unknown
3.      Enough anchors? -> Perform Azipe
3.      Not enough anchors or Azipe failed? -> Perform fallback estimation
*/

namespace pos{
    //General setting flags
    const int ILLUSTRATE_ARUCO = 0;//Is used for extra vizualisation. In this mode the AruCo-markers will be drawn
    const int ILLUSTRATE_FLOW  = 1;//Is used for extra vizualisation. In this mode the Flow field will be drawn
    const int ILLUSTRATE_ALL   = 2;//Is used for extra vizualisation. In this mode the Flow field and aruco markers will be drawn
    const int ILLUSTRATE_NONE  = 3;//Is used in ASAP-mode. No illustrations, only necessary calculations
    //Flags for the optical flow
    const int OF_MODE_KLT  = 1;//VO is used as fallback method, optical flow obtained with KLT
    const int OF_MODE_CORR = 2;//VO is used as fallback method, optical flow obtained with phase correlation
    //Flags for the Visual Odometry
    const int VO_MODE_HOMOGRAPHY    = vo::USE_HOMOGRAPHY;
    const int VO_MODE_AFFINE        = vo::USE_AFFINETRANSFORM;

    //Needed additional settings:
    /*
        -camera K matrix
        -UAV-camera T-matrix
        -RoI size
        -Flow field size (x-by-x)
        -All KLT-settings
        -AruCo dictionary mode
        -Aruco marker max id. Should be able to contain all markers in the specified dictionary mode
    */
    class positioning: public ang::angulation, of::opticalFlow, vo::planarHomographyVO{
        cv::Ptr<cv::aruco::Dictionary> dictionary;//Pointer to Aruco dictionary
    public:
        positioning(int,int,int,int,        //Mode settings [Illustrate, optical flow, Visual odometry,aruco dictionary type]
                    int,std::string,        //Aruco init parameters [max marker ID, "Path-to-csv-file"]
                    int,cv::Rect2f,                //Optical flow parameters [flow grid, roi size]
                    cv::Mat_<float>,cv::Mat_<float> //Visual odometry paramters [K-mat, T-mat]
                );
        int process(cv::Mat,float,float);//Perform processing. return value indicates what kind of estimation is done
        //int init

    private:

    };
}
#endif
