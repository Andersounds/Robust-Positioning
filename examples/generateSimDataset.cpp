#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include "../src/simulatePose.hpp"
#include "../src/logger.hpp"
#include "../src/timeStamp.cpp"
#include "../src/dataStream.hpp"
#define PI 3.1416


int main(int argc, char** argv){
timestamp::timeStamp_ms stamp;
double timeStamp;
stamp.get(timeStamp);


std::cout << "This file will be edited to be used to create simulation dataset" << std::endl;
std::cout << "1. Read a predefined true path with roll, pitch, yaw" << std::endl;
std::cout << "2. Simulation background. chessboard? backgroundimage?" << std::endl;
std::cout << "3. What are the physical dimensions of scene?" << std::endl;
std::cout << "4. Camera parameters? what are they?" << std::endl;
std::cout << "5. When saving the data, a complete settings file shall also be created (for k mat etc)" << std::endl;




log::imageLogger imagebin;
std::string pathToDir = "Generated-dataSets/";
std::string newDir = "5-jul-2";
imagebin.init(pathToDir,newDir);

robustPositioning::dataStreamer getData("Generated-dataSets/5_jul/truePath.csv");


    //Initialize settings
    //set::settings S(argc,argv);
    //if(!S.success){return 0;}
    //Initialize simulation
    // Define sim chessboard parameters
    int boxWidth = 11;
    int rowsOfBoxes = 30;
    int colsOfBoxes = 60;
// Define file objects to output data into
//    std::ofstream file_true;
//    std::ofstream file_estimated;
// Init simulation environment
    simulatePose warper;
    cv::Mat floor = cv::imread("spike/test2.png",cv::IMREAD_REDUCED_COLOR_4);
    if(floor.empty()){std::cout << "Could not read base image" << std::endl; return 0;}
    cv::Mat floor8U;
    cv::cvtColor(floor, floor8U, cv::COLOR_BGR2GRAY);
    warper.setBaseScene(floor);
    warper.setParam("d",2);             //Camera in base pose is 1 m from scene
    warper.setParam("sceneWidth",4);    //Scenewidth is 2m
    warper.setParam("yaw",-3.1415/2);   // Camera is rotated 90 deg cc in base pose
    warper.setParam("x",2);         //Set global origin at (-x,-y) from basepose
    float y_ = ((float)floor.rows)*2/((float)floor.cols);//Calculate y coordinate in base pose [m]
    warper.setParam("y",y_);
    warper.init(0);//Initialize with configuration 0




//Go through whole path
std::vector<float> data;
    while(getData.get(data)){
        //Get data
        std::vector<float> trueCoordinate{data[1],data[2],data[3]};
        std::vector<float> angles{data[4],data[5],data[6]};
    //Get new image
        cv::Mat rawFrame = warper.uav2BasePose(angles,trueCoordinate);
        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);
        cv::imshow("showit",rawFrame);

        //Dump image
        stamp.get(timeStamp);
        imagebin.dump(timeStamp,frame);
        //cv::waitKey(0);
        if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}
    }
    imagebin.rename(cv::String(pathToDir+newDir+"/"),cv::String("img_"));
    return 1;
}
