#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "../src/videoStream.cpp"
#include "../src/KLT_ORB_Tracker.cpp"

using namespace cv;
int main(int argc, char** argv){




std::string basePath, path1, path2;
//basePath = "/Users/Fredrik/Datasets/Euroc/V101/cam0/data/cam0_";
basePath = "/Users/Fredrik/Datasets/";

path1 = basePath + argv[1];
path2 = basePath + argv[2];

cv::Mat roi1 = cv::imread(path1);
cv::Mat queryImg = cv::imread(path2);
//std::cout << queryImg.rows << ", " << queryImg.cols << std::endl;
cv::Mat mask(queryImg, cv::Rect(610, 120, 100, 100) );
cv::imshow("MASK",mask);

cv::imshow(argv[1],roi1);
cv::imshow(argv[2],queryImg);
std::cout << "esc to close windows and move on" << std::endl;
cv::waitKey(0);
cv::destroyAllWindows();


int nfeatures = 100;
float scalefactor = 1.2;
int nlevels = 8;
int nThreshold = 20;//31
int firstLevel = 0;
int WTA_K = 2;
int scoreType = cv::ORB::HARRIS_SCORE;
int patchSize = 31;
int fastThreshold = 20;
cv::Ptr<cv::ORB> orbObject = cv::ORB::create(
                            nfeatures,
                            scalefactor,
                            nlevels,
                            nThreshold,
                            firstLevel,
                            WTA_K,
                            scoreType,
                            patchSize,
                            fastThreshold);


std::vector< cv::KeyPoint > roikeypoints1, keypoints2;
cv::Mat roidescriptors1, descriptors2;
orbObject->detect(roi1,roikeypoints1);
orbObject->detect(mask,keypoints2);
//cv::drawKeypoints(roi1, roikeypoints1, roi1,CV_RGB(0,255,0));
orbObject ->compute(roi1,roikeypoints1, roidescriptors1);
orbObject ->compute(mask,keypoints2, descriptors2);

//cv::imshow("Features",img1);
//cv::waitKey(0);



/*
Matching
*/
int normType = cv::NORM_HAMMING;
bool crosscheck = true;
cv::Ptr<cv::BFMatcher> matcherObject = cv::BFMatcher::create(normType, crosscheck);
std::vector<cv::DMatch> matches;


matcherObject ->match(roidescriptors1,descriptors2,matches);//OBS ordning på descriptors orsakar krasch
std::vector< char >  drawMask;
int rows = std::max(roi1.rows,queryImg.rows);
int cols = roi1.cols + queryImg.cols;
int type = roi1.type();
cv::Mat imgOut(rows,cols,type);
//std::cout << Scalar::all(-1);
//cv::drawMatches(img1,keypoints1,queryImg,keypoints2,matches,imgOut,-1,-1,mask,1);
//cv::drawMatches(img1,keypoints1,queryImg,keypoints2,matches,imgOut);
cv::drawMatches(roi1,roikeypoints1,mask,keypoints2,
               matches, imgOut, Scalar::all(-1), Scalar::all(-1),
               drawMask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


cv::imshow("Matches",imgOut);
cv::waitKey(0);






}
