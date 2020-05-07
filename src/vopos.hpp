#ifndef POSITIONING_H
#define POSITIONING_H
//#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "angulation.hpp"
#include "opticalFlow.hpp"
#include "homographyVO.hpp"
#include "azipe.hpp"
#include "martonRobust.hpp"
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
    const int MODE_FALLBACK = 0;                //Positioning estimation with only VO method. For forcing fallback method
    const int MODE_AZIPE_AND_FALLBACK = 1;      //Positioning estimation with AZIPE angulation and fallback if it fails


    /*
        2 flags for function call.
            1 for switch/case statmennt in main function. assigns which function to call. azipe, azipe+VO, azipe+marton
            1 for giving to azipe+VO or aipe+marton call. states of we shall override and force fallback



    */


    //remove these
    const int MODE_AZIPE = 0;
    //const int MODE_FALLBACK = 1;                //Positioning estimation with only VO method. For forcing fallback method
    //const int MODE_AZIPE_AND_FALLBACK = 2;      //Positioning estimation with AZIPE angulation and fallback if it fails

    const int MODE_VO =3;
    const int MODE_AZIPE_AND_VO = 4;
    const int MODE_AZIPE_AND_MARTON = 5;
    const int MODE_MARTON = 6;


    //Return flags for what positioning algorithm was used
    const int RETURN_MODE_AZIPE = 1;        //Azipe angulation was used to estimate position
    const int RETURN_MODE_VO    = 2;        //Visual odometry was used to estimate position (from last)
    const int RETURN_MODE_MARTON = 3;
    //Error codes are above 10
    const int RETURN_MODE_AZIPE_FAILED = 10; //If only azipe is used and estimation fails
    //VO error codes
    const int RETURN_MODE_INERTIA = 20;        //Visual odometry failed, assumed rotational speed kept
    const int RETURN_MODE_PROJ = 21;           //Visual odometry successful and estimation is projected onto v_tilde
    //Marton error codes
    const int RETURN_MODE_MARTON_FAILED = 30;
    const int RETURN_MODE_MARTON_OLD = 31;
    const int RETURN_MODE_MARTON_ERR = 32;


    #define TSPAN_MAX 3000
    /* Convenience structs for passing to positioning functions*/
    struct argStruct {
        float dist;
        float roll;
        float pitch;
        float yaw;
    };
    struct VOargStruct {
        float dist;
        float roll;
        float pitch;
        float yaw;
    };
    struct MartonArgStruct{
        //float dist;
        float roll;
        float pitch;
        float yaw;
        float time;
        int bufferSize;
        float coneWeight;
    };

    class positioning: public ang::angulation, of::opticalFlow, public vo::planarHomographyVO{
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
        //int process(int,cv::Mat&,float,float,float,float&,cv::Mat_<float>&);//Perform processing. return value indicates what kind of estimation is done

        int process_AZIPE(cv::Mat& frame, cv::Mat& outputFrame,cv::Mat_<float>& pos, argStruct& arguments);
        int process_VO_Fallback(int mode,cv::Mat& frame, cv::Mat& outputFrame, cv::Mat_<float>& pos, VOargStruct& arguments);
        int process_Marton_Fallback(int mode,cv::Mat& frame, cv::Mat& outputFrame, cv::Mat_<float>& pos, MartonArgStruct& arguments);


        //Illustration processing. Second mat argument is drawn upon.
        //int processAndIllustrate(int,cv::Mat&,cv::Mat&,int,float,float&,float&,float&,cv::Mat_<float>&,float&);//Perform processing, but also illustrate by drawing on the second argument matrix
        //int processAz(int,cv::Mat&,cv::Mat&,int,float,float&,float&,float&,cv::Mat_<float>&,float&);//Only azipe aipe
        void illustrateDerotation(cv::Mat&,cv::Mat&,float,float&,float&,float&);//Only visualize derotation
        void illustrateYaw(cv::Mat&,float);
        cv::Mat_<float> T_vopos;
    private:
        void drawLines(cv::Mat&,std::vector<cv::Point2f>,cv::Point2f);
        void drawMarkers(cv::Mat&,std::vector<std::vector<cv::Point2f>>,std::vector<int>,std::vector<bool>);
        void projectionFusing(cv::Mat_<float>&,std::vector<cv::Mat_<float>>, std::vector<cv::Mat_<float>>, std::vector<uchar>,
                                float, float,float);
        void projectionFusing(cv::Mat_<float>&,std::vector<cv::Mat_<float>>,std::vector<cv::Mat_<float>>,std::vector<bool>,float,float,float);
        cv::Mat getXRot(float);
        cv::Mat getYRot(float);
        cv::Mat getZRot(float);
    };
}
#endif
