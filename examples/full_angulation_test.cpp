#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
//#include <chrono> //For timestamps
#include "../src/vopos.hpp"

#include "../src/settingsParser.cpp"


#define PI 3.1416


#include <typeinfo>
int main(int argc, char** argv){

    //Initialize settings
    set::settings S(argc,argv);
    if(!S.success()){return 0;}


    // Read image

    cv::Mat frame = cv::imread(S.data.testImageRelPath,CV_LOAD_IMAGE_GRAYSCALE);

    //Initialize positioning object
    int maxIdAruco = 50;
    std::string anchorPath = S.data.anchorPath;//"anchors.txt";
    cv::Rect2f roiSize = S.data.ROI;
    cv::Mat_<float> K = S.data.K;
    cv::Mat_<float> T = S.data.T;//warper.getZRot(-PI/2);//UAV frame is x forward, camera frame is -y forward
    int flowGrid = 1;
    pos::positioning P(pos::OF_MODE_KLT,//pos::OF_MODE_CORR,//pos::OF_MODE_KLT,
                        pos::VO_MODE_AFFINE,//pos::VO_MODE_HOMOGRAPHY,//pos::VO_MODE_AFFINE,
                        cv::aruco::DICT_4X4_50,
                        maxIdAruco,anchorPath,flowGrid,roiSize,K,T);
    //Init values of position and yaw
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = S.data.x0;
    t(1,0) = S.data.y0;
    t(2,0) = S.data.z0;
    float yaw = S.data.yaw0;

    cv::Mat_<float> t2;
    t.copyTo(t2);
    float dist = 1.5;
    float roll = 0;
    float pitch = 0;
    //int mode = P.process(pos::MODE_AZIPE,frame,dist, roll, pitch, yaw, t2);
    cv::Mat colorFrame;
    cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
    int mode = P.processAndIllustrate(pos::MODE_AZIPE,frame,colorFrame,pos::ILLUSTRATE_ALL,dist,roll,pitch,yaw,t2);
    cv::imshow("processing",colorFrame);
    cv::waitKey(0);



    std::cout << "Diff: " << t.t() - t2.t() << std::endl;
    std::cout << "Xest: " << t2.t() << std::endl;

    return 1;
}





/*
std::cout << "V:" << std::endl;
for(cv::Mat_<float> i:v){
    std::cout  << ": "<< i.t() << std::endl;
}




*/
