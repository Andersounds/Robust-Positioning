#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H
/*
This file defines all prototypes for optical flow calculation for use with the Visual Odometry-implementation
Two methods of calculating optical flow is implemented, and the choice between them is done by setting a flag
on instantiation

Method 1. Lukas Kanade pyramidal optical flow. Without corners. an x-by-x grid of points in region of interest is specified
    At each new frame, these points are regarded as "features to track". This method has in preliminary tests shown to be
    much more robust in low contrast environments than actual feature tracking. The inevitable noise is handled by RANSAC
    in the VO algorithm.
    key: USE_KLT
Method 2. Phase-correlation based optical flow. The RoI is divided into x-by-x squares. Each of these are phase correlated
    with the same square from the previous image, and from each square a single flow arrow is obtained
    This method is a variant of the one presented by Walter18
    key: USE_CORR
*/
namespace of{
    #define USE_KLT 1
    #define USE_CORR 2
    class opticalFlow{
    public:
        opticalFlow(int,int,int);
        int getFlow(const cv::Mat&, const cv::Mat&,std::vector<cv::Point2f>&,std::vector<cv::Point2f>&);
        void setDefaultSettings(void);
    private:
        bool init;
        int mode; //Either USE_CORR or USE_KLT
        int grid; //specifies size of flow grid (number of flow arrows in x and y direction). Total flow field will be grid^2 points arranged in a grid
        float h; //The distance between each flow arrow in x-and y direction. Also the width of each sub roi
        float grid_x; //x-ccordinate of first point in the used RoI.
        float grid_y; //See above
        std::vector<cv::Point2f> p1; //First points in point correspondances (In grid shape)
        //Parameters for phase correlation flow field
        cv::Mat hann; //Hanning window to reduce edge effects in phase correlation
        double corrQualityLevel;
        std::vector<cv::Rect2f> subRoI; //Coordinates of all subRects that are used for phase-correlation
        int corrFlow(const cv::Mat&, const cv::Mat&,std::vector<cv::Point2f>&,std::vector<cv::Point2f>&);
        //Parameters for KLT pyramidal flow field
        //KLT track parameters
        int flags;
        int windowSize;
        int maxLevel;
        cv::TermCriteria termcrit;
        int KLTFlow(const cv::Mat&, const cv::Mat&,std::vector<cv::Point2f>&,std::vector<cv::Point2f>&);

        //Parameters for KLT pyramidal flow field


    };




}
#endif
