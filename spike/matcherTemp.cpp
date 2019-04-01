#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "../src/videoStream.cpp"
#include "../src/KLT_ORB_Tracker.cpp"

#include <time.h>


int main(int argc, char** argv){
std::cout << "OpenCV version : " << CV_VERSION << std::endl;
/*Get images*/
  cv::Mat img_objectBig = imread("/Users/Fredrik/Datasets/picsForMatching/box.png", cv::IMREAD_GRAYSCALE );
  cv::Mat img_object = img_objectBig(cv::Rect(0,0,img_objectBig.cols,img_objectBig.rows));
  cv::imshow("small area",img_object);
  cv::waitKey(0);
  cv::Mat img_scene = imread("/Users/Fredrik/Datasets/picsForMatching/box_in_scene.png", cv::IMREAD_GRAYSCALE );
  cv::Mat mat4visual;
  img_scene.copyTo(mat4visual);
  cv::cvtColor(mat4visual, mat4visual, cv::COLOR_GRAY2BGR);
/*Some settings*/
  int noOfClusters = 1;
  int clusterSize = img_object.cols;


  int minDistance = 100;

/*Initialize variables*/
  std::vector< cv::KeyPoint > objectKeyPoints, queryKeyPoints;
  cv::Mat objectDescriptors, queryDescriptors;

  std::vector<cv::Rect> rectangles; //findclusters returnerar till denna
  rectangles.push_back(cv::Rect(0,0,img_object.cols,img_object.rows));
  std::vector<cv::DMatch> matches, goodmatches;
  std::vector<cv::Point2f> returnPoints, onlyGoodMatchesPoints;
  std::vector<cv::Point2f> src, dst;
  cv::Mat H;

/*Get data from object image*/
  KLT_ORB_Tracker tracker;
  tracker.init();
  tracker.getFeatures(img_object,objectKeyPoints);
  tracker.getFeatures(img_scene,queryKeyPoints);
  //rectangles = tracker.findClusters(img_object,keyPoints, noOfClusters, clusterSize, minDistance,returnKeyPoints, returnDescriptors);
  tracker.orbObject->compute(img_object,objectKeyPoints,objectDescriptors);// Calculate descriptors
  tracker.orbObject->compute(img_scene, queryKeyPoints,queryDescriptors);// Calculate descriptors
  //Get data from scene (whole scene)
  //tracker.getFeatures(img_object,returnKeyPoints);


  //tracker.orbObject->compute(img_scene,queryKeyPoints,queryDescriptors);// Calculate descriptors


  tracker.featureMatching(queryDescriptors,objectDescriptors, matches);



  tracker.trackMatches(matches, goodmatches,objectKeyPoints, queryKeyPoints, returnPoints, rectangles[0],onlyGoodMatchesPoints,src,dst,H);

  if(src.size()>0){
    //tracker.drawPoints(mat4visual,originalKeyPoints,mat4visual,CV_RGB(200,200,0));
    //cv::rectangle(mat4visual,originalRectangle,CV_RGB(200,0,0),2,cv::LINE_8,0);
    cv::line(img_object,src[0],src[1],CV_RGB(200,0,0),2,8);
    cv::line(img_object,src[1],src[2],CV_RGB(200,0,0),2,8);
    cv::line(img_object,src[2],src[3],CV_RGB(200,0,0),2,8);
    cv::line(img_object,src[3],src[0],CV_RGB(200,0,0),2,8);

    cv::line(mat4visual,dst[0],dst[1],CV_RGB(0,0,200),3,8);
    cv::line(mat4visual,dst[1],dst[2],CV_RGB(0,0,200),3,8);
    cv::line(mat4visual,dst[2],dst[3],CV_RGB(0,0,200),3,8);
    cv::line(mat4visual,dst[3],dst[0],CV_RGB(0,0,200),3,8);
    std::cout << "Src: " << src << std::endl;
    std::cout << "Dst: " << dst << std::endl;
    std::cout << "H= " << H << std::endl;
    cv::Mat img_matches;

    ///drawMatches( img_object, objectKeyPoints, img_scene, queryKeyPoints,
    ///             goodmatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
    ///             std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    tracker.drawPoints(img_scene,onlyGoodMatchesPoints,img_matches,CV_RGB(255,0,0));
    //tracker.drawPoints(img_scene,onlyGoodMatchesPoints,img_matches,CV_RGB(200,200,0));
    //cv::imshow("Matches",img_matches);
    cv::imshow("Show",img_matches);
    cv::waitKey(0);

  }else{std::cout << "No matches" << std::endl;}

  cv::imshow("New rect shape",mat4visual);
  cv::waitKey(0);


return 1;
}

/*
  tracker.getQueryFeatures(frame, rectangles[0], queryKeyPoints, queryDescriptors);
  tracker.featureMatching(returnDescriptors[0], queryDescriptors, matches);

  tracker.trackMatches(matches, originalKeyPoints, queryKeyPoints, returnPoints[0], originalRectangle,onlyGoodMatchesPoints,src,dst);//queryeypoints == Scene keypoints
  tracker.drawPoints(mat4visual,onlyGoodMatchesPoints,mat4visual,CV_RGB(0,200,0));//Draw points of only good matches
  std::cout << "src size: " << src.size() << std::endl;
  if(src.size()>0){
    tracker.drawPoints(mat4visual,originalKeyPoints,mat4visual,CV_RGB(200,200,0));
    cv::rectangle(mat4visual,originalRectangle,CV_RGB(200,0,0),2,cv::LINE_8,0);
    cv::line(mat4visual,src[0],src[1],CV_RGB(200,0,0),2,8);
    cv::line(mat4visual,src[1],src[2],CV_RGB(200,0,0),2,8);
    cv::line(mat4visual,src[2],src[3],CV_RGB(200,0,0),2,8);
    cv::line(mat4visual,src[3],src[0],CV_RGB(200,0,0),2,8);

    cv::line(mat4visual,dst[0],dst[1],CV_RGB(0,0,200),3,8);
    cv::line(mat4visual,dst[1],dst[2],CV_RGB(0,0,200),3,8);
    cv::line(mat4visual,dst[2],dst[3],CV_RGB(0,0,200),3,8);
    cv::line(mat4visual,dst[3],dst[0],CV_RGB(0,0,200),3,8);
    std::cout << "Src: " << src << std::endl;
    std::cout << "Dst: " << dst << std::endl;
    cv::rectangle(mat4visual,rectangles[0],CV_RGB(255,0,0),2,cv::LINE_8,0);
*/
