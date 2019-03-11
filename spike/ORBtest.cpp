#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "../src/videoStream.cpp"
#include "../src/KLT_ORB_Tracker.cpp"

#include <time.h>

using namespace cv;
int main(int argc, char** argv){


cv::Mat frame = cv::Mat::zeros(300,300,CV_8UC3);
cv::Mat mat4visual;
std::vector< cv::KeyPoint > keypoints;
int noOfClusters = 1;
int clusterSize = 40;
//cv::KeyPoint kp = cv::KeyPoint(100,100,-1,0,0,-1);
//cv::KeyPoint kp(100,100,-1,0,0,-1);
keypoints.push_back(cv::KeyPoint(100,100,5,1,0,-1));
keypoints.push_back(cv::KeyPoint(99+clusterSize,100,5,1,0,-1));
keypoints.push_back(cv::KeyPoint(99+clusterSize,99+clusterSize,5,1,0,-1));
keypoints.push_back(cv::KeyPoint(100,99+clusterSize,5,1,0,-1));


KLT_ORB_Tracker tracker;
tracker.init();
cv::Rect rectangle = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize);
//cv::cvtColor(frame, mat4visual, COLOR_GRAY2BGR,3);
cv::drawKeypoints(frame, keypoints, mat4visual,CV_RGB(0,255,0));
std::cout << "Keypoints: " << rectangle.x << std::endl;
cv::rectangle(mat4visual,rectangle,CV_RGB(255,0,0),2,LINE_8,0);

cv::imshow("Keypoints",mat4visual);
cv::waitKey(0);


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
