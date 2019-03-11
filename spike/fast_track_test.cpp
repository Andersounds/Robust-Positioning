
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep
#include "../src/VisPos.cpp"
#include "../src/videoStream.cpp"
#include "../src/argumentParser.cpp"

using namespace cv;
//using namespace std;

/*
Uses FAST algorithm to detect features. Code is copied not comletely sure what happens
*/

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


  std::vector<KeyPoint> keypointsD;
  Ptr<FastFeatureDetector> detector=FastFeatureDetector::create(12,true,FastFeatureDetector::TYPE_9_16);
  vector<Mat> descriptor;

  detector->detect(prevFrame,keypointsD,Mat());
  int size = keypointsD.size();
  std::vector< Point2f > corners(size);
  std::vector< int >(size,1);
  cv::KeyPoint::convert(keypointsD,corners);

//drawKeypoints(grayframe, keypointsD, grayframe);
/*
Track previous features in new frame
*/
  for(int i = 1;i<400;i++){
    if(corners.empty()){break;}//Retry if no corners found
    C->getImage(freshFrame);
    cv::cvtColor(freshFrame, nextFrame, COLOR_BGR2GRAY);


    std::vector<Point2f> trackedCorners(size);
    vector<uchar> status;
    vector<float> errors;
    cv::Size winSize(51,51);
    int maxLevel = 2;
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    cv::calcOpticalFlowPyrLK(prevFrame,nextFrame,corners,trackedCorners,status,errors,winSize,maxLevel,termcrit,0,0.001);

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
