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

  Streamer* C;
  C = Streamer::make_streamer(arguments.streamMode,arguments.datasetPath,arguments.cam);


  cv::Mat latestFrame;
  while(1){
  C->getImage(latestFrame);
/*
Do something
*/



  imshow("Showit",latestFrame);
  if( waitKey(1) == 27 ) break;                         // stop capturing by pressing ESC
  }


}
