#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "../src/positioning.hpp"
//#include "../src/simulatePose.hpp"
//#include "../src/angulation.hpp"
//#include "../src/save2file.cpp"



#define PI 3.1416


int main(void){
    int maxIdAruco = 50;
    std::string anchorPath = "anchors.txt";
    int flowGrid = 4;
    cv::Rect2f roiSize(0,0,150,150);
    cv::Mat_<float> K = cv::Mat_<float>::zeros(3,3);
    cv::Mat_<float> T = cv::Mat_<float>::zeros(3,3);
    pos::positioning P(pos::ILLUSTRATE_ARUCO,
                        pos::OF_MODE_KLT,
                        pos::VO_MODE_AFFINE,
                        cv::aruco::DICT_4X4_50,
                        maxIdAruco,anchorPath,flowGrid,roiSize,K,T);
    //P.setKmat(K);
    //P.setTmat(T);


}
