#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep
#include "../src/videoStream.cpp"
#include "../src/argumentParser.cpp"


/*
Takes goodfeaturestotrack to find features, and uses Lukas Kanade Pyramid optical flow to track them
*/

using namespace cv;
//using namespace std;



int main(int argc, char** argv){
  // check if inputs are correctly given
  streamArguments arguments;
  if(!argumentParser(argc,argv,arguments)){return 0;}

  Streamer* C;
  C = Streamer::make_streamer(arguments.streamMode,arguments.datasetPath,arguments.cam);

  cv::Mat freshFrame, prevFrame, nextFrame;


/*
Get features to track from a frame
*/
while(1){
  C->getImage(prevFrame); //Initialize with a frame
  if(prevFrame.empty()){return 1;}
  cv::cvtColor(prevFrame, prevFrame, COLOR_BGR2GRAY);
  int noOfCorners = 15;
  std::vector<Point2f> corners(noOfCorners);
  double qualityLevel= 0.5;
  double minDistance = 20;
  cv::Mat mask;
  int blockSize = 3;
  double k = 0.04;
  bool useHarris = false;
  cv::goodFeaturesToTrack(prevFrame, corners, noOfCorners, qualityLevel, minDistance,mask,blockSize,useHarris,k);

/*
Track previous features in new frame
*/
  for(int i = 1;i<40;i++){
    if(corners.empty()){break;}//Retry if no corners found
    C->getImage(freshFrame);
    cv::cvtColor(freshFrame, nextFrame, COLOR_BGR2GRAY);


    std::vector<Point2f> trackedCorners(noOfCorners);
    std::vector<uchar> status;
    std::vector<float> errors;
    cv::Size winSize(51,51);
    int maxLevel = 2;
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    cv::calcOpticalFlowPyrLK(prevFrame,nextFrame,corners,trackedCorners,status,errors,winSize,maxLevel,termcrit,0,0.001);
    std::cout << "       corners: " << corners[1] << std::endl;
    std::cout << "trackedcorners: " << trackedCorners[1] << std::endl;
/*
Draw points
*/
    for(int i=0;i<corners.size();i++){
      //std::cout << i << ":  " << corners.at(i)<<std::endl;
      circle(nextFrame, trackedCorners.at(i), 10,CV_RGB(0,255,0),2, 8,0);
      //circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    }


/*
Shift frames
*/
  nextFrame.copyTo(prevFrame);
  corners = trackedCorners;

    cv::imshow("Showit",nextFrame);
    if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}                          // stop capturing by pressing ESC
    //usleep(2000);
  }

}
}
