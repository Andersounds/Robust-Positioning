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
-måste ha ett reproducerbat sätt att relatera rektangeln till keypoint. Nu skapas rektangeln där alla kp får plats, men sen placeras den enl ngt annat
*/
int main(int argc, char** argv){
//Settings
    int noOfClusters = 1;
    int clusterSize = 70;
    int minDistance = 100;
    int roiFeatures = 200;
    int sceneFeatures = 1000;
    int noOfAnchors = 20;
    int noOfParallell = 1;
//Initialize data holder variables on the bottom of the stack
    cv::Mat frame,prevFrame,freshFrame, mat4visual; // Opencv mat
    std::vector<cv::Rect_<float>> rectangles;       // Rectangles tracked by KLT
    std::vector<cv::Point2f> roIPoints;             // Points tracked by KLT
    std::vector<trackedObject> anchors;             // Anchors tracked
    int totlaps=0;                                  // Counter of every frame. Can theoretically overflow
    int state = 0;                                  // State holder for Switch statement
    to::trackedObjectList objectList(noOfAnchors,noOfParallell);  // List containing all tracked general anchors
    std::vector<int> stateSequence{2,1,0};// In which order to process all active anchors (based on state)
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
//Inf loop
while(1){
// All runs start with a new image
    C ->getImage(freshFrame);
    if(freshFrame.empty()){break;}
    cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY,1);
    cv::Mat roIMask = cv::Mat::zeros(frame.size(), CV_8UC1);                    //Mask used to find KP in only region of interest
    cv::Mat invertedMask = cv::Mat(frame.size(), CV_8UC1,cv::Scalar(255));      //Mask used to find KP only outside of all active RoIs
    freshFrame.copyTo(mat4visual);                                              //3 channel frame used for visualization

    for(int substate:stateSequence){//All active anchors need to be processed in an order depending on their state
        for(int currentAnchor = 0;currentAnchor<objectList.no_of_parallel;currentAnchor++){//Loop through all indexes of active anchors
            if(objectList.activeStates[currentAnchor] != substate){continue;}//Only run the switch statement if the state of current anchor is correct
//Reinitialize some temporary vectors on the stack
            std::vector< cv::KeyPoint > keypoints;
            std::vector< cv::KeyPoint > roIKeypoints;
            std::vector<cv::DMatch> matches;
            std::vector<cv::DMatch> good_matches;
            cv::Mat sceneDescriptors;
            cv::Mat objectDescriptors;

            state = objectList.activeStates[currentAnchor];//Take the current state of the current active anchor
            switch (state) {
                case 0:{//Get new regions to track
                    tracker.getFeatures(frame,keypoints);//Detect ORBs in whole scene
                    rectangles = tracker.findClusters(frame,invertedMask,keypoints, noOfClusters, clusterSize, minDistance);//Get cluster
                    tracker.getMask(rectangles[0],roIMask);
                    roIKeypoints = roiTracker.getFeatures(frame,roIMask);  //Get new features only within cluster rectangle
                    cv::KeyPoint::convert(roIKeypoints, roIPoints);     //Convert to Points to be used by KLT-tracker
                    if(roIKeypoints.size()>1&&keypoints.size()>1){      //Must be at least one pair
                        roiTracker.orbObject->compute(frame, roIKeypoints, objectDescriptors);//Calculate the keypoint descriptors from the current frame
                        objectList.add(new trackedObject(roIKeypoints, objectDescriptors,rectangles[0]));//Add new region to list of tracked objects
                    }
                state = 2; //Do KLT tracking next
                break;
                }
                case 1:{//Locate tracked objects in new frame
                    tracker.getFeatures(frame,keypoints);//Detect ORBs in whole scene
                    tracker.orbObject->compute(frame, keypoints, sceneDescriptors);//Calculate descriptors of all KP in scene
                    int index = 0;
                    for(int index=0;index < objectList.no_of_tracked;index++){//Try to match with all tracked objects (that are not active) until match found
                        int anchorID = objectList.list[index]->ID;
                        //If the current object is already active, then dont try to match with it
                        if(objectList.activeIDs.find(ID)){continue;}
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
                            //posibly only black out if there is at least one active anchor in state 0. Otherwise its not needed.
                            cv::rectangle(invertedMask,rectangles[0],1,CV_FILLED,cv::LINE_8,0);// Black out the inverted mask where the anchor is found
                            state = 2;//Do KLT tracking next
                            break;// Break after first match is found
                        }
                        else{std::cout << "-";}//Not match
                    }
                if(state==1){state=0;std::cout << std::endl;}//If no match has been found, go to state 0 next. Print endline only also to show in terminal
                break;
                }
                case 2:{//KLT track points from prev image in new image
                    int success = tracker.trackOpticalFlow(prevFrame,frame,roIPoints,rectangles[0]); //Do KLT tracking
                    //posibly only black out if there is at least one active anchor in state 0. Otherwise its not needed.
                    cv::rectangle(invertedMask,rectangles[0],1,CV_FILLED,cv::LINE_8,0);// Black out the inverted mask where the anchor is found
                    if(!success){//If tracking was not successful
                        // Remove the anchor from list of active anchors.
                        state = 1; //Go to state 1
                    }
                break;
                }
            }//End of switch statement
            objectList.activeStates[currentAnchor] = state;//Set the new state of the current active anchor
        }//End of for loop
    }//End of sequence loop
    tracker.drawPoints(mat4visual,roIPoints,mat4visual,CV_RGB(255,0,0));
    cv::rectangle(mat4visual,rectangles[0],CV_RGB(255,0,0),2,cv::LINE_8,0);
    cv::imshow("VideoStream",mat4visual);
    frame.copyTo(prevFrame); //Shift new image to prev
    if( cv::waitKey(1) == 27 ) {break;}
}//End of while loop

std::cout << "Bryter"<< std::endl;
printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
std::cout<< "Number of frames processed: " << totlaps << std::endl;
objectList.clear();// Deallocate memory on heap
return 1;
}
