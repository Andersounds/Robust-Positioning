#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "../src/videoStream.cpp"
#include "../src/KLT_ORB_Tracker.cpp"

#include <time.h>

//using namespace cv;
int main(int argc, char** argv){

  Streamer* C;
  C = Streamer::make_streamer(3,"",0);
  cv::Mat frame,prevFrame,mat4visual,freshFrame;
  int noOfClusters = 1;
  int clusterSize = 40;
  int minDistance = 100;




KLT_ORB_Tracker tracker;
std::vector< cv::KeyPoint > keypoints;
tracker.init();
int counter= 0;



clock_t tStart = clock();
while(1){
C ->getImage(freshFrame);
if(freshFrame.empty()){
  printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  std::cout<< "Number of frames processed: " << counter << std::endl;
      return 1;}
cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY);
frame.copyTo(mat4visual);
tracker.getFeatures(frame,keypoints);
//cv::Mat mat4visual;// = cv::Mat(keypointMat.size(),CV_8UC3);
//cv::cvtColor(frame, mat4visual, COLOR_GRAY2BGR,3);
std::vector<std::vector<cv::KeyPoint>> returnKeyPoints;
std::vector<cv::Rect> rectangles = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize, minDistance,returnKeyPoints);



/*TESTBLOCK*/
/*Denna nollas ju hela tiden. Det är ett problem.*/
/*Här skapas en vector av vector av point2f istället för keypoints. Som KLT-trackern ska ha*/
std::vector<std::vector< cv::Point2f >> returnPoints;
std::vector< cv::Point2f > p1;
cv::KeyPoint::convert(returnKeyPoints[0],p1);
returnPoints.push_back(p1);
/*TESTBLOCK*/



frame.copyTo(prevFrame);//Förskjut så att vi har både en frame och en prevFrame
while(1==1){
  C ->getImage(freshFrame);
  if(freshFrame.empty()){
    printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    std::cout<< "Number of frames processed: " << counter << std::endl;
        return 1;}
  cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY);
  frame.copyTo(mat4visual);

//std::cout << "HERE: " << rectangles[0].x << std::endl;
/*Denna nollas ju hela tiden. Det är ett problem.*/
/*std::vector<std::vector< cv::Point2f >> returnPoints;
std::vector< cv::Point2f > p1;
cv::KeyPoint::convert(returnKeyPoints[0],p1);
returnPoints.push_back(p1);*/


  std::vector< cv::KeyPoint > TestKeyPoint;
  //cv::KeyPoint::convert(returnKeyPoints[i],returnPoints);

  if(!tracker.trackOpticalFlow(prevFrame,frame,returnPoints[0],rectangles[0])){
    std::cout << "Rectangle lost. Position: [" << rectangles[0].x << "," << rectangles[0].y << "]" <<std::endl;
    //std::cout << "TrackedCorners:  [" << returnPoints[0][2].x << "," << returnPoints[0][2].y << "]" <<std::endl;
    //keypoints.clear();
    tracker.getFeatures(frame,keypoints);
    returnKeyPoints.pop_back();
    rectangles = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize, minDistance,returnKeyPoints);
    returnPoints.pop_back();
    cv::KeyPoint::convert(returnKeyPoints[0],p1);
    returnPoints.push_back(p1);
    }

  cv::KeyPoint::convert(returnPoints[0],TestKeyPoint);//Arg1: input. Arg2: output

  cv::drawKeypoints(mat4visual, TestKeyPoint, mat4visual,CV_RGB(200,0,255)); //Just keyPoints in rect
  cv::drawKeypoints(mat4visual, keypoints, mat4visual,CV_RGB(0,255,0));//All keyPoints



  //return 1;

  //cv::drawKeypoints(mat4visual, returnKeyPoints[i], mat4visual,CV_RGB(0,255,0));
  cv::rectangle(mat4visual,rectangles[0],CV_RGB(255,0,0),2,cv::LINE_8,0);

  //std::cout << "Point: " << rectangles[0].x << ", " << rectangles[0].y <<std::endl;

  std::string picNum = std::to_string(counter);
  counter++;

  cv::imshow("VideoStream",mat4visual);
  if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;
    printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    std::cout<< "Number of frames processed: " << counter << std::endl;return 1;}

    frame.copyTo(prevFrame);
}



}






}



//cv::Mat roi1 = cv::imread(path1,cv::IMREAD_GRAYSCALE);
//cv::rectangle(mat4visual,rect,CV_RGB(0,0,0),CV_FILLED,LINE_8,0);//DENNA FÖR ATT hitta flera kluster
/*
//Create mat where keypoint coordinates are set to non-zero/
cv::Mat keypointMat = cv::Mat::zeros(queryImg.size(), CV_8UC1);
//cv::Mat keypointMat = cv::Mat::zeros(queryImg.size(), CV_32FC1);
for(int i = 0; i < keypoints2.size(); ++i) {
    keypointMat.at<uchar>(keypoints2[i].pt) = 128;//Maybe use other value 1 is enough?
}



cv::Mat kernel = cv::Mat::ones(50,50,CV_8UC1);

cv::Mat filteredMat;
cv::filter2D(keypointMat, filteredMat, -1 , kernel, Point( -1, -1 ), 0, BORDER_DEFAULT );
double min, max;
cv::Point min_loc, max_loc;
cv::minMaxLoc(keypointMat, &min, &max, &min_loc, &max_loc);
std::cout<< max_loc << std::endl;
//Too slow? Try these optimizations. (Decomposition mainly)
// https://stackoverflow.com/questions/19533370/best-way-to-efficiently-find-high-density-regions
//https://stackoverflow.com/questions/31336186/opencv-computational-efficiency-of-filter2d-function
*/
/*
cv::Mat mat4visual;// = cv::Mat(keypointMat.size(),CV_8UC3);
cv::cvtColor(queryImg, mat4visual, COLOR_GRAY2BGR,3);

//cv::cvtColor(keypointMat, mat4visual, COLOR_GRAY2BGR,3);
////cv::Rect rect(max_loc.x-25,max_loc.y,50,50);
////cv::rectangle(mat4visual,rect,CV_RGB(0,255,0),2,LINE_8,0);
//cv::rectangle(mat4visual,rect,CV_RGB(0,0,0),CV_FILLED,LINE_8,0);//DENNA FÖR ATT hitta flera kluster


cv::Rect rectangle = tracker.findClusters(queryImg,keypoints2);
cv::rectangle(mat4visual,rectangle,CV_RGB(0,255,0),2,LINE_8,0);
cv::drawKeypoints(mat4visual, keypoints2, mat4visual,CV_RGB(0,255,0));
cv::imshow("POINTS",mat4visual);
cv::waitKey(0);
*/
//tracker.calcORBDescriptors(roi1,roikeypoints1, roidescriptors1);
//tracker.calcORBDescriptors(mask,keypoints2, descriptors2);







/*
Matching
*/
/*std::vector<cv::DMatch> matches;
tracker.featureMatching(roidescriptors1,descriptors2,matches);*/

/*cv::Mat imgOut;
tracker.drawTheMatches(roi1,roikeypoints1,mask,keypoints2,matches, imgOut);
cv::imshow("Matches",imgOut);*/
//cv::imshow("Keypoints", mask);
//cv::waitKey(0);






//}
