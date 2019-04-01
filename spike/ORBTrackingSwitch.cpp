#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "../src/videoStream.hpp"
#include "../src/KLT_ORB_Tracker.hpp"
#include "../src/houghDetector.hpp"
#include "../src/trackedObjectClass.hpp"

#include <time.h>

/*
Problem:
-Ibland hittas kp utanför roi????? varför?
-måste ha ett reproducerbat sätt att relatera rekangeln till keypoint. Nu skapas rektangeln där alla kp får plats, men sen placeras den enl ngt annat
-Trackopticalflow är inte riktigt placerad rätt. Rita upp tänkt flöde för main och rätta till
*/
int main(int argc, char** argv){
//Settings
    int noOfClusters = 1;
    int clusterSize = 70;
    int minDistance = 100;
    int roiFeatures = 200;
    int sceneFeatures = 1000;
    int noOfAnchors = 20;
//Initialize data holder variables on the bottom of the stack
    cv::Mat frame,prevFrame,freshFrame, mat4visual; // Opencv mat
    std::vector<cv::Rect_<float>> rectangles;       // Rectangles tracked by KLT
    std::vector<cv::Point2f> roIPoints;             // Points tracked by KLT
    std::vector<trackedObject> anchors;             // Anchors tracked
    int totlaps=0;                                  // Counter of every frame. Can theoretically overflow
    int state = 0;                                  // State holder for Switch statement
    to::trackedObjectList objectList(noOfAnchors);  // List containing all tracked general anchors
//Initialize objects
    //Clock
    clock_t tStart = clock();
    //Data streamer
    Streamer* C;
    C = Streamer::make_streamer(3,"",0);//"/Users/Fredrik/Datasets/Euroc/V102/cam0/data/cam0_%04d.png"
    //Trackers
    KLT_ORB_Tracker tracker, roiTracker;
    roiTracker.setNfeatures(roiFeatures);   //Finds fewer keypoints in region of interest
    tracker.setNfeatures(sceneFeatures);    //Finds many keypoints in whole scene
    tracker.init();
    roiTracker.init();
    //Matcher
    cv::Ptr<cv::BFMatcher> matcherObject = cv::BFMatcher::create(cv::NORM_HAMMING,true);


while(1){
// All runs start with a new image
    C ->getImage(freshFrame);
    if(freshFrame.empty()){
        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        std::cout<< "Number of frames processed: " << totlaps << std::endl;
        return 1;
    }
    cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY,1);
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    freshFrame.copyTo(mat4visual);
//Reinitialize some temporary vectors on the stack
    std::vector< cv::KeyPoint > keypoints;//(2000);
    std::vector< cv::KeyPoint > roIKeypoints;//(100);
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    cv::Mat sceneDescriptors;
    cv::Mat objectDescriptors;
    //std::cout << "State: " << state << std::endl;

    switch (state) {
        case 0:{//Get new regions to track
            tracker.getFeatures(frame,keypoints);//Can this be moved out?
            rectangles = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize, minDistance);//Get cluster
            tracker.getMask(rectangles[0],mask);
            roIKeypoints = roiTracker.getFeatures(frame,mask);  //Get new features only within cluster rectangle
            cv::KeyPoint::convert(roIKeypoints, roIPoints);     //Convert to Points to be used by KLT-tracker
            if(roIKeypoints.size()>1&&keypoints.size()>1){      //Must be at least one pair
                roiTracker.orbObject->compute(frame, roIKeypoints, objectDescriptors);//Calculate the keypoint descriptors from the current frame
                objectList.add(new trackedObject(roIKeypoints, objectDescriptors,rectangles[0]));//Add new region to list of tracked objects
            }
        state = 2; //Do KLT tracking next
        break;
        }
        case 1:{//Locate tracked objects in new frame
            tracker.getFeatures(frame,keypoints);//Can this be moved out?
            tracker.orbObject->compute(frame, keypoints, sceneDescriptors);//Calculate descriptors of all KP in scene
            int index = 0;
            while(index < objectList.no_of_tracked){//Try to match with all tracked objects until match found
                matcherObject ->match(objectList.list[index]->originalDescriptors,sceneDescriptors,matches);//Perform matching
                hd::getGoodMatches(matches, good_matches);//Sort out the best 30%
                std::vector<cv::Point> xWinningPairs, yWinningPairs;
                std::vector<float> status;
                // Do Hough detection
                status = hd::detect(good_matches, objectList.list[index]->originalKeyPoints,keypoints, xWinningPairs, yWinningPairs);

                if(status[0]){//Match!
                    std::cout << index << std::endl;
                    std::vector<cv::DMatch> winningMatches = hd::getBestMatches(good_matches,xWinningPairs,yWinningPairs);
                    roIPoints.clear(); //Clear vector to remove garbage data
                    //
                    roIPoints = objectList.list[index]->getTrackablePoints(status[1],status[2],winningMatches,keypoints,rectangles[0]);
                    state = 2;//Do KLT tracking next
                    break;// Break after first match is found
                }
                else{std::cout << "-";}//Not match
                index++;
            }
        if(state==1){state=0;std::cout <<std::endl;}//If no match has been found, go to state 0 next
        break;
        }
        case 2:{//KLT track points from prev image in new image

            int success = tracker.trackOpticalFlow(prevFrame,frame,roIPoints,rectangles[0]); //Do KLT tracking
            if(!success){//If tracking was not successful
                state = 1; //Go to state 1
            }
        break;
        }
    }
    tracker.drawPoints(mat4visual,roIPoints,mat4visual,CV_RGB(255,0,0));
    cv::rectangle(mat4visual,rectangles[0],CV_RGB(255,0,0),2,cv::LINE_8,0);
    cv::imshow("VideoStream",mat4visual);
    frame.copyTo(prevFrame); //Shift new image to prev
    if( cv::waitKey(1) == 27 ) {
        std::cout << "Bryter"<< std::endl;
        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        std::cout<< "Number of frames processed: " << totlaps << std::endl;
        objectList.clear();
        return 1;
    }

}
objectList.clear();
return 1;
}
