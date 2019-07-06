#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep
#include "../src/videoStream.hpp"
#include "../src/argumentParser.cpp"

using namespace cv;
//using namespace std;



int main(int argc, char** argv){
  // check if inputs are correctly given
    streamArguments arguments;
    if(!argumentParser(argc,argv,arguments)){return 0;}
  //robustPositioning::Streamer C("images/","data.csv");
    robustPositioning::Streamer C("images/img_%03d.png");
    cv::Mat latestFrame;
    while(1){
        float ts = C.getImage(latestFrame);
        if(latestFrame.empty()){
            std::cout << "Stream done. "<< std::endl;
            return 0;
        }
        imshow("Showit",latestFrame);
        if( waitKey(1) == 27 ) break;                         // stop capturing by pressing ESC
    }
    return 1;
}
