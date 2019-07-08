#include <iostream>
#include <opencv2/opencv.hpp>
#include "../src/videoStream.hpp"
#include "../src/logger.hpp"
#include "../src/timeStamp.cpp"
#define PI 3.1416



int main(int argc, char** argv){
timestamp::timeStamp_ms stamp;
//Can these two rows be written as stamp.get(double timeStamp); is timeStamp available in this scope then?
double timeStamp;
stamp.get(timeStamp); //initialize timestamp
std::cout << "Initial timestamp: " << timeStamp << "ms" << std::endl;
robustPositioning::Streamer VStreamer(robustPositioning::MODE_RPI_CAM);
cv::Mat frame, colorFrame;

//Read data until done
float timeStamp;
int counter = 1;
int noOfImages = 100;
    while(counter<=noOfImages){
        VStreamer.getImage(frame);
        std::cout << "Read image " << counter << std::endl;
        if(frame.empty()){std::cout << "Video stream done."<< std::endl; return 0;}
        //cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
        counter++;
    }
    stamp.get(timeStamp);
    std::cout << "Done." << std::endl;
    std::cout << "Collected " << noOfImages << " images in " << timeStamp << "ms." << std::endl;
    double fps = noOfImages*1000/timeStamp;
    std::cout << "Mean tempo: " << fps << " fps" << std::endl;
    return 1;
}
