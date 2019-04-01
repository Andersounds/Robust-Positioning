#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "../src/videoStream.hpp"
#include "../src/KLT_ORB_Tracker.hpp"
#include "../src/houghDetector.hpp"

#include <time.h>


int main(int argc, char** argv){
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    Streamer* C;
    C = Streamer::make_streamer(3,"",0);
    cv::Mat frame,prevFrame,mat4visual,freshFrame;
    int noOfClusters = 1;
    int clusterSize = 70;
    int minDistance = 100;


    KLT_ORB_Tracker tracker;
    std::vector< cv::KeyPoint > keypoints;
    tracker.init();
    int counter= 0;
    int laps = 0;
    int totlaps=0;
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
        std::vector<cv::DMatch> matches,goodmatches;


        std::vector<cv::Point2f> onlyGoodMatchesPoints;// Denna är tillfällig för att bara få ut alla bra matches
        std::vector<cv::Point2f> src;
        std::vector<cv::Point2f> dst;
        cv::Rect originalRectangle;
        std::vector<cv::KeyPoint> originalKeyPoints;


        std::vector<std::vector<cv::KeyPoint>> returnKeyPoints;
        std::vector<cv::Mat> returnDescriptors;
        tracker.getFeatures(frame,keypoints);
        std::vector<cv::Rect> rectangles = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize, minDistance);
        originalRectangle = rectangles[0];
        originalKeyPoints = returnKeyPoints[0];
        /*Här skapas en vector av vector av point2f istället för keypoints. Som KLT-trackern ska ha*/
        std::vector<std::vector< cv::Point2f >> returnPoints;
        std::vector< cv::Point2f > p1;
        cv::KeyPoint::convert(returnKeyPoints[0],p1);
        returnPoints.push_back(p1);


        frame.copyTo(prevFrame);//Förskjut så att vi har både en frame och en prevFrame
        while(1==1){
            laps ++ ;
            totlaps++;

            C ->getImage(freshFrame);
            if(freshFrame.empty()){
                printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
                std::cout<< "Number of frames processed: " << counter << std::endl;
                return 1;
            }
            cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY,1);
            frame.copyTo(mat4visual);

      //std::cout << "Rectangle x: " << rectangles[0].x << std::endl;
      if(!tracker.trackOpticalFlow(prevFrame,frame,returnPoints[0],rectangles[0])){
        std::cout << "Rectangle lost. Position: [" << rectangles[0].x << "," << rectangles[0].y << "]" <<std::endl;
        tracker.getFeatures(frame,keypoints);// find features over whole frame
        returnKeyPoints.pop_back();
        returnDescriptors.pop_back();
        rectangles = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize, minDistance);
//------
        //returnKeyPoints = keypoints;
        tracker.getQueryFeatures(frame, rectangles[0], keypoints, returnDescriptors[0]);

        //-----
        originalRectangle = rectangles[0];
        originalKeyPoints = returnKeyPoints[0];

        cv::KeyPoint::convert(returnKeyPoints[0],p1);
        returnPoints.pop_back();
        returnPoints.push_back(p1);
        frame.copyTo(prevFrame);

        break;//
        }



      //cv::drawKeypoints(mat4visual, queryKeyPoints, mat4visual,CV_RGB(50,50,255));//QueryKeyPoints

      if(laps>50){
      tracker.getQueryFeatures(frame, rectangles[0], queryKeyPoints, queryDescriptors);
      if(queryKeyPoints.size()>1){//Guard here. if only one keypoinnt it will cause crash
          tracker.featureMatching(returnDescriptors[0], queryDescriptors, matches);
      }
        /*cv::line(mat4visual,src[0],src[1],CV_RGB(200,0,0),2,8);
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

      laps = 0;
      }

      //tracker.featureMatching(returnDescriptors[0], queryDescriptors, matches);

      //tracker.trackMatches(matches,returnPoints[0],rectangles[0]);

      /*if(scene.size()){
        cv::Mat H = cv::findHomography(obj,scene,cv::RHO);
        scene.clear();
        obj.clear();
      }*/
      //tracker.drawPoints(mat4visual,onlyGoodMatchesPoints,mat4visual,CV_RGB(0,200,0));//Draw points of only good matches
      //tracker.drawPoints(mat4visual,returnPoints[0],mat4visual,CV_RGB(200,0,0)); //Draw current points that are being tracked

      //tracker.drawPoints(mat4visual,queryKeyPoints,mat4visual,CV_RGB(200,0,0));//Draw matches

      //tracker.drawPoints(mat4visual, returnPoints[0], mat4visual,CV_RGB(200,0,255)); //Just keyPoints in rect
      //tracker.drawPoints(mat4visual, keypoints, mat4visual,CV_RGB(0,255,0));//All keyPoints
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
