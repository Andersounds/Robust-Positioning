#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep
#include "../src/videoStream.cpp"
#include "../src/argumentParser.cpp"
#include "featureTracker.cpp"


/*
Takes goodfeaturestotrack to find features, and uses Lukas Kanade Pyramid optical flow to track them
*/

using namespace cv;
using namespace std;



int main(int argc, char** argv){
  // check if inputs are correctly given
  std::cout << argv[1] << std::endl;
  streamArguments arguments;
  if(!argumentParser(argc,argv,arguments)){return 0;}

  Streamer* C;
  C = Streamer::make_streamer(arguments.streamMode,arguments.datasetPath,arguments.cam);

  cv::Mat freshFrame, prevFrame, nextFrame;

  int noOfCorners = 2;
  goodfeatureTracker F(noOfCorners);

while(1){
  C->getImage(freshFrame); //Initialize with a frame
  if(freshFrame.empty()){return 1;}/*If end of dataset then return*/

  cv::cvtColor(freshFrame, nextFrame, COLOR_BGR2GRAY);
  F.getFeaturesToTrack(nextFrame);
  F.initTracker(freshFrame);


  for(int i=0;i<100;i++)
  {
    C->getImage(nextFrame); //Initialize with a frame
    if(nextFrame.empty()){return 1;}//If end of dataset then return
    //cv::cvtColor(nextFrame, nextFrame, COLOR_BGR2GRAY);

    F.update(nextFrame);


    cv::imshow("Showit",nextFrame);
    if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}                          // stop capturing by pressing ESC
  }











}
}
