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
    //Illustration settings flags
    const int ILLUSTRATE_ARUCO = 0;//Is used for extra vizualisation. In this mode the AruCo-markers will be drawn
    const int ILLUSTRATE_FLOW  = 1;//Is used for extra vizualisation. In this mode the Flow field will be drawn
    const int ILLUSTRATE_ALL   = 2;//Is used for extra vizualisation. In this mode the Flow field and aruco markers will be drawn
    //const int ILLUSTRATE_NONE  = 3;//Is used in ASAP-mode. No illustrations, only necessary calculations
    //Flags for the optical flow
    const int OF_MODE_KLT  = of::USE_KLT;//Optical flow obtained with KLT
    const int OF_MODE_CORR = of::USE_CORR;//Optical flow obtained with phase correlation
    //Flags for the Visual Odometry
    const int VO_MODE_HOMOGRAPHY    = vo::USE_HOMOGRAPHY;
    const int VO_MODE_AFFINE        = vo::USE_AFFINETRANSFORM;
    //Flags for Aruco dictionary. Can choose anone but this is an example
    const int ARUCO_DICT_DEFAULT    = cv::aruco::DICT_4X4_50;
    //Flags for process-function call.
    const int MODE_AZIPE_AND_VO = 0;//Positioning using AZIPE estimation with VO as fallback
    const int MODE_AZIPE = 1;       //Positioning estimation with only AZIPE angulation
    const int MODE_VO = 2;          //Positioning estimation with only VO method
    //Return flags for what positioning algorithm was used
    const int RETURN_MODE_AZIPE = 0;        //Azipe angulation was used to estimate position
    const int RETURN_MODE_VO    = 1;        //Visual odometry was used to estimate position (from last)
    const int RETURN_MODE_INERTIA=2;        //Visual odometry failed, assumed rotational speed kept
    const int RETURN_MODE_PROJ=3;           //Visual odometry successful and estimation is projected onto v_tilde

    class positioning: public ang::angulation, of::opticalFlow, vo::planarHomographyVO{
        cv::Ptr<cv::aruco::Dictionary> dictionary;//Pointer to Aruco dictionary
        cv::Rect2f roi;//The region of interest that is to be considered in
        int minAnchors;
    public:
        /* Constructor. constructs classes that are inherited from and sets default settings*/
        positioning(int,int,int,        //Mode settings [optical flow, Visual odometry,aruco dictionary type]
                    int,std::string,        //Aruco init parameters [max marker ID, "Path-to-csv-file"]
                    int,cv::Rect2f,                 //Optical flow parameters [flow grid, roi size]
                    cv::Mat_<float>,cv::Mat_<float> //Visual odometry paramters [K-mat, T-mat]
                );
        //ASAP processing. No illustration
        int process(int,cv::Mat&,float,float,float,float&,cv::Mat_<float>&);//Perform processing. return value indicates what kind of estimation is done
        //Illustration processing. Second mat argument is drawn upon.
        int processAndIllustrate(int,cv::Mat&,cv::Mat&,int,float,float,float,float&,cv::Mat_<float>&);//Perform processing, but also illustrate by drawing on the second argument matrix
    private:
        void drawLines(cv::Mat&,std::vector<cv::Point2f>,cv::Point2f);
        void projectionFusing(cv::Mat_<float>&,std::vector<cv::Mat_<float>>, std::vector<cv::Mat_<float>>, std::vector<uchar>,
                                float, float,float);
        void projectionFusing(cv::Mat_<float>&,std::vector<cv::Mat_<float>>,std::vector<cv::Mat_<float>>,std::vector<bool>,float,float,float);
        cv::Mat getXRot(float);
        cv::Mat getYRot(float);
        cv::Mat getZRot(float);
    };
}
#endif
