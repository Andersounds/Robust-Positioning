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
    int clusterSize = 100;
    int minDistance = 100;
    int roiFeatures = 200;
    int sceneFeatures = 1000;
    int noOfAnchors = 20;
    int noOfParallell = 2;
//Initialize data holder variables on the bottom of the stack
    cv::Mat frame,prevFrame,freshFrame, mat4visual; // Opencv mat
    float totlaps=0;                                  // Counter of every frame. Can theoretically overflow
    int state = 0;                                  // State holder for Switch statement
    bool getNewRegions = true;                      // A persistent variable that dictates if the inverted mask should be calculated. Basically just to save time
    to::trackedObjectList objectList(noOfAnchors,noOfParallell);  // List containing all tracked general anchors
    std::vector<int> stateSequence{2,1,0};          // In which order to process all active anchors (based on state)
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
    //cv::Mat roIMask = cv::Mat(frame.size(), CV_8UC1,cv::Scalar(0));//Mask used to find KP in only region of interest
    cv::Mat invertedMask = cv::Mat(frame.size(), CV_8UC1,cv::Scalar(255));                  //Mask used to find KP only outside of all active RoIs
    freshFrame.copyTo(mat4visual);                                              //3 channel frame used for visualization

    for(int substate:stateSequence){//All active anchors need to be processed in an order depending on their state
        for(int currentAnchor = 0;currentAnchor<objectList.no_of_parallel;currentAnchor++){//Loop through all indexes of active anchors
            state = objectList.getState(currentAnchor);//Take the current state of the current active anchor
            if(state != substate){continue;}//Only run the switch statement if the state of current anchor is correct
//Reinitialize some temporary vectors on the stack
            std::vector< cv::KeyPoint > keypoints;
            //std::vector< cv::KeyPoint > roIKeypoints;
            std::vector<cv::DMatch> matches;
            std::vector<cv::DMatch> good_matches;
            cv::Mat sceneDescriptors;
            switch (state) {
                case 0:{//Get new regions to track for all anchors with state 0
                    std::vector<int> zeroIndexes = objectList.getZeroStateIndexes();//Get indexes of all active anchors with state 0
                    int noOfClusters = zeroIndexes.size();
                    tracker.getFeatures(frame,keypoints);//Detect ORBs in whole scene
                    std::vector<cv::Rect_<float>> rectangles;       //  tracked by KLT
                    rectangles = tracker.findClusters(frame,invertedMask,keypoints, noOfClusters, clusterSize, minDistance);//Get cluster
                    //Repeated for all rectangles
                    for(int index =0;index<noOfClusters;index++){
                        std::vector<cv::Point2f> roIPoints;
                        std::vector< cv::KeyPoint > roIKeypoints;
                        roIKeypoints = roiTracker.getFeatures(frame,rectangles[index]);//Här kraschar ibland
                        cv::Mat temp;
                        cv::KeyPoint::convert(roIKeypoints, roIPoints);     //Convert to Points to be used by KLT-tracker
                        if(roIKeypoints.size()>1&&keypoints.size()>1){      //Must be at least one pair
                            cv::Mat objectDescriptors;
                            roiTracker.calcORBDescriptors(frame, roIKeypoints, objectDescriptors);//Här kraschar ibland
                            int objectID = objectList.add(new trackedObject(roIKeypoints, objectDescriptors,rectangles[index], roiFeatures));//Add new region to list of tracked objects
                            objectList.assignAnchor(zeroIndexes[index],objectID);
                            objectList.updateAnchor(zeroIndexes[index],rectangles[index],roIPoints);
                            objectList.setState(zeroIndexes[index],2);//Do KLT tracking next
                        }
                    }
                break;
                }
                case 1:{//Locate tracked objects in new frame
                    tracker.getFeatures(frame,keypoints);//Detect ORBs in whole scene
                    tracker.calcORBDescriptors(frame,keypoints,sceneDescriptors);
                    int index = 0;
                    for(int index=0;index < objectList.no_of_tracked;index++){//Try to match with all tracked objects (that are not active) until match found
                        int anchorID = objectList.list[index]->ID;
                        if(objectList.isActive(anchorID)){continue;}//If the current object is already active, then dont try to match with it
                        matcherObject ->match(objectList.list[index]->originalDescriptors,sceneDescriptors,matches);//Perform matching
                        hd::getGoodMatches(matches, good_matches);//Sort out the best 30%
                        std::vector<cv::Point> xWinningPairs, yWinningPairs;
                        std::vector<float> status;
                        // Do Hough detection
                        status = hd::detect(good_matches, objectList.list[index]->originalKeyPoints,keypoints, xWinningPairs, yWinningPairs);
                        if(status[0]){//Match!
                            std::cout << index << std::endl;
                            std::vector<cv::DMatch> winningMatches = hd::getBestMatches(good_matches,xWinningPairs,yWinningPairs);
                            std::vector<cv::Point2f> roIPoints;
                            //
                            cv::Rect_<float> newRect;
                            roIPoints = objectList.list[index]->getTrackablePoints(status[1],status[2],winningMatches,keypoints,newRect);
                            if(getNewRegions){//Only black out if there is at least one active anchor in state 0. Otherwise its not needed.
                                tracker.blackOutMask(invertedMask,newRect,minDistance);
                                //cv::rectangle(invertedMask,newRect,0,CV_FILLED,cv::LINE_8,0);// Black out the inverted mask where the anchor is found
                            }
                            objectList.assignAnchor(currentAnchor,index);//Assign the object with index index to active anchor currentAnchor
                            objectList.updateAnchor(currentAnchor, newRect, roIPoints);
                            objectList.setState(currentAnchor,2);//Do KLT tracking next
                            //state = 2;//Do KLT tracking nex
                            break;// Break after first match is found
                        }
                        else{std::cout << "-";}//Not match
                    }
                if(objectList.getState(currentAnchor)==1){               //If no match has been found
                    objectList.setState(currentAnchor,0);                //The anchor goes into state 0
                    getNewRegions = true;   //At least one new region needs to be found next lap
                    std::cout << std::endl; //Print endline to show in terminal
                }
                break;
                }
                case 2:{//KLT track points from prev image in new image
                    int currentID = objectList.activeIDs[currentAnchor];
                    cv::Rect_<float> newRect = objectList.list[currentID]->trackedRect;
                    std::vector<cv::Point2f> roIPoints = objectList.list[currentID]->trackedPoints;
                    int success = tracker.trackOpticalFlow(prevFrame,frame,roIPoints,newRect); //Do KLT tracking
                    if(getNewRegions&&success){//Only black out if there is at least one active anchor in state 0. Otherwise its not needed.
                        tracker.blackOutMask(invertedMask,newRect,minDistance);
                    }
                    if(success){//If tracking was successful
                        objectList.updateAnchor(currentAnchor,newRect,roIPoints);
                    }
                    else{//If not successful
                        objectList.setState(currentAnchor,1);//Go to state 1
                    }
                break;
                }
            }//End of switch statement
        }//End of for loop
    }//End of sequence loop
    //first

    tracker.drawPoints(mat4visual,objectList.list[objectList.activeIDs[0]]->trackedPoints,mat4visual,CV_RGB(255,0,0));
    cv::rectangle(mat4visual,objectList.list[objectList.activeIDs[0]]->trackedRect,CV_RGB(255,0,0),2,cv::LINE_8,0);
    //second
    tracker.drawPoints(mat4visual,objectList.list[objectList.activeIDs[1]]->trackedPoints,mat4visual,CV_RGB(0,255,0));
    cv::rectangle(mat4visual,objectList.list[objectList.activeIDs[1]]->trackedRect,CV_RGB(0,255,0),2,cv::LINE_8,0);
    //Third
    //tracker.drawPoints(mat4visual,objectList.list[objectList.activeIDs[2]]->trackedPoints,mat4visual,CV_RGB(0,0,255));
    //cv::rectangle(mat4visual,objectList.list[objectList.activeIDs[2]]->trackedRect,CV_RGB(0,0,255),2,cv::LINE_8,0);
    //Show
    cv::imshow("VideoStream",mat4visual);
    frame.copyTo(prevFrame); //Shift new image to prev
    if( cv::waitKey(1) == 27 ) {break;}
    totlaps++;
}//End of while loop

std::cout << "Bryter"<< std::endl;
float time = (float)((clock() - tStart)/CLOCKS_PER_SEC);
printf("Time taken: %.2fs\n", time);
std::cout<< "Number of frames processed: " << totlaps << std::endl;
std::cout<< "Avarage fps: " << totlaps/time << std::endl;
objectList.clear();// Deallocate memory on heap
return 1;
}
