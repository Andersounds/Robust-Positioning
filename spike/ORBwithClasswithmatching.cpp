#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "../src/videoStream.cpp"
#include "../src/KLT_ORB_Tracker.cpp"

#include <time.h>


int main(int argc, char** argv){
std::cout << "OpenCV version : " << CV_VERSION << std::endl;
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
    cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY,1);
    frame.copyTo(mat4visual);


    std::vector<cv::KeyPoint> queryKeyPoints; //OBS måste nolla dessa efter varje användning?
    cv::Mat queryDescriptors;
    std::vector<cv::DMatch> matches;

//cv::Mat mat4visual;// = cv::Mat(keypointMat.size(),CV_8UC3);
//cv::cvtColor(frame, mat4visual, COLOR_GRAY2BGR,3);
    std::vector<std::vector<cv::KeyPoint>> returnKeyPoints;
    std::vector<cv::Mat> returnDescriptors;
    tracker.getFeatures(frame,keypoints);
    std::vector<cv::Rect> rectangles = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize, minDistance,returnKeyPoints, returnDescriptors);
    /*Här skapas en vector av vector av point2f istället för keypoints. Som KLT-trackern ska ha*/
    std::vector<std::vector< cv::Point2f >> returnPoints;
    std::vector< cv::Point2f > p1;
    cv::KeyPoint::convert(returnKeyPoints[0],p1);
    returnPoints.push_back(p1);


    frame.copyTo(prevFrame);//Förskjut så att vi har både en frame och en prevFrame
    while(1==1){
      C ->getImage(freshFrame);
      if(freshFrame.empty()){
        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        std::cout<< "Number of frames processed: " << counter << std::endl;
        return 1;}
      cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY,1);
      frame.copyTo(mat4visual);

      std::vector< cv::KeyPoint > TestKeyPoint;

      if(!tracker.trackOpticalFlow(prevFrame,frame,returnPoints[0],rectangles[0])){
        std::cout << "Rectangle lost. Position: [" << rectangles[0].x << "," << rectangles[0].y << "]" <<std::endl;
        tracker.getFeatures(frame,keypoints);
        returnKeyPoints.pop_back();
        returnDescriptors.pop_back();
        rectangles = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize, minDistance,returnKeyPoints, returnDescriptors);
        cv::KeyPoint::convert(returnKeyPoints[0],p1);
        returnPoints.pop_back();
        returnPoints.push_back(p1);
        frame.copyTo(prevFrame);
        break;//
        }


      tracker.getQueryFeatures(frame, rectangles[0], queryKeyPoints, queryDescriptors);
      cv::drawKeypoints(mat4visual, queryKeyPoints, mat4visual,CV_RGB(50,50,255));//QueryKeyPoints


      //tracker.featureMatching(returnDescriptors[0], queryDescriptors, matches);
      tracker.featureMatching(queryDescriptors,returnDescriptors[0], matches);// Crashes if wrong order. Maybe use the overloaded match instead which only gives one descriptor argument

      //tracker.trackMatches(matches,returnPoints[0],rectangles[0]);

      //Here sort out the matches and only keep good ones
      std::vector <cv::Point2f> scene, obj;
      for( int i = 0; i < matches.size(); i++ ){
        scene.push_back( returnKeyPoints[0][ matches[i].trainIdx ].pt );
        obj.push_back( queryKeyPoints[ matches[i].queryIdx ].pt );
      }


      if(scene.size()){
        cv::Mat H = cv::findHomography(obj,scene,cv::RHO);
        scene.clear();
        obj.clear();
      }

      tracker.drawPoints(mat4visual, returnPoints[0], mat4visual,CV_RGB(200,0,255)); //Just keyPoints in rect
      tracker.drawPoints(mat4visual, keypoints, mat4visual,CV_RGB(0,255,0));//All keyPoints
      cv::rectangle(mat4visual,rectangles[0],CV_RGB(255,0,0),2,cv::LINE_8,0);

      std::string picNum = std::to_string(counter);
      counter++;

      cv::imshow("VideoStream",mat4visual);
      if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;
        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        std::cout<< "Number of frames processed: " << counter << std::endl;return 1;
      }
      frame.copyTo(prevFrame);
    }
  }
}
