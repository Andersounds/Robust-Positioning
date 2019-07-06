#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
//#include <chrono> //For timestamps
#include "../src/logger.hpp"
#include "../src/timeStamp.cpp"


int main(int argc, char** argv){

    log::imageLogger imagebin;
    imagebin.init("","");
    imagebin.rename(cv::String("5_jul/"),cv::String("img_"));


    return 1;
}
