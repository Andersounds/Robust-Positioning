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
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    Streamer* C;
    C = Streamer::make_streamer(3,"",0);//"/Users/Fredrik/Datasets/Euroc/V102/cam0/data/cam0_%04d.png"
    cv::Mat frame,prevFrame,freshFrame;

    C ->getImage(freshFrame);
    if(freshFrame.empty()){return 1;}
    cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY,1);
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat mat4visual = cv::Mat(frame.size(),CV_8UC3);
    freshFrame.copyTo(mat4visual);

    int noOfClusters = 1;
    int clusterSize = 70;
    int minDistance = 100;
    int roiFeatures = 200;
    int sceneFeatures = 1000;

    KLT_ORB_Tracker tracker, roiTracker;
    roiTracker.setNfeatures(roiFeatures);
    tracker.setNfeatures(sceneFeatures);
    tracker.init();
    roiTracker.init();
//Matcher
    cv::Ptr<cv::BFMatcher> matcherObject = cv::BFMatcher::create(cv::NORM_HAMMING,true);
    cv::Mat obj;

    int laps = 0;
    int totlaps=0;
    clock_t tStart = clock();

    std::vector<cv::Rect_<float>> rectangles;
    std::vector<cv::Point2f> roIPoints;
    std::vector<trackedObject> anchors;

    int init = 0;
    int success = 0;
    int getNewRegions = 1;

    int stride = 0;

// Multiple tracked objects
    to::trackedObjectList objectList(20);


int pause = 0;
    while(1==1){
        std::vector< cv::KeyPoint > keypoints;//(2000);
        std::vector< cv::KeyPoint > roIKeypoints;//(100);
        std::vector<cv::DMatch> matches;
        std::vector<cv::DMatch> good_matches;
        cv::Mat sceneDescriptors;
        cv::Mat objectDescriptors;

        laps ++ ;
        totlaps++;

        C ->getImage(freshFrame);
        if(freshFrame.empty()){
            printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
            std::cout<< "Number of frames processed: " << totlaps << std::endl;
            return 1;
        }
        cv::cvtColor(freshFrame, frame, cv::COLOR_BGR2GRAY,1);
        freshFrame.copyTo(mat4visual);

        // Get new regions to track if KLT tracker says no success.
        if(!success || laps>100){
            laps=0;
            tracker.getFeatures(frame,keypoints);
            // First try to match. in this version match with every tracked object or until match is found
            if(init){
                tracker.orbObject->compute(frame, keypoints, sceneDescriptors);
                int index = 0;
                while(index < objectList.no_of_tracked){
                    matcherObject ->match(objectList.list[index]->originalDescriptors,sceneDescriptors,matches);
                    hd::getGoodMatches(matches, good_matches);
                    std::vector<cv::Point> xWinningPairs, yWinningPairs;
                    std::vector<float> status;
                    status = hd::detect(good_matches, objectList.list[index]->originalKeyPoints,keypoints, xWinningPairs, yWinningPairs);
                    if(status[0]){
                        std::cout << index << std::endl;
                        std::vector<cv::DMatch> winningMatches = hd::getBestMatches(good_matches,xWinningPairs,yWinningPairs);
                        roIPoints.clear();
                        roIPoints = objectList.list[index]->getTrackablePoints(status[1],status[2],winningMatches,keypoints,rectangles[0]);
                        getNewRegions = 0;
                        break;// Break after first match is found
                    }
                    else{
                        std::cout << "-";//One dash for every no-match
                        getNewRegions = 1;
                    }
                    index++;
                }
///////////////// IMAGES::::::
                    //std::cout << "STATUS: " << status[0] << std::endl;
                /*    cv::Mat img_matches;
                    cv::destroyAllWindows();
                    std::cout << "Keypoint sizes: "<< objectList.list[index]->originalKeyPoints.size() << ", " << keypoints.size() << std::endl;
                    cv::drawMatches(obj, objectList.list[index]->originalKeyPoints, frame, keypoints,
                               matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                               cv::imshow("All Matches", img_matches);
                    cv::waitKey(0);
                    cv::destroyAllWindows();
                }
                cv::drawMatches(obj, FIRSTroIKeypoints, frame, keypoints,
                               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                cv::imshow("Good Matches", img_matches);
                cv::waitKey(0);
                cv::destroyAllWindows();
                std::vector<cv::DMatch> best_matches = hd::getBestMatches(good_matches,xWinningPairs, yWinningPairs);
                cv::drawMatches(obj, FIRSTroIKeypoints, frame, keypoints,
                               best_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                cv::imshow("Best Matches", img_matches);
                cv::waitKey(0);
                cv::destroyAllWindows();*/
///////////////// IMAGES::::::

//If success: getNewRegions = 1 else 0
// Need to be able to mask out currently tracked clusters when finding new ones
            }
            if(getNewRegions){
                rectangles = tracker.findClusters(frame,keypoints, noOfClusters, clusterSize, minDistance);
                tracker.getMask(rectangles[0],mask);
                //cv::imshow("VideoStream",mask);
                //cv::waitKey(0);
                roIKeypoints = roiTracker.getFeatures(frame,mask);
                cv::KeyPoint::convert(roIKeypoints, roIPoints);
                //Have to have this loop here because I cant trust that getFeatures respects the mask
                /*std::vector< cv::Point2f > newRoIPoints;
                for(int i = 0;i<roIPoints.size();i++){
                    if(rectangles[0].contains(roIPoints[i])){
                        newRoIPoints.push_back(roIPoints[i]);
                    }
                }
                roIPoints = newRoIPoints;*/
            //}
            //if(!init){
/////
                if(roIKeypoints.size()>1&&keypoints.size()>1){//Must be at least one pair
                    roiTracker.orbObject->compute(frame, roIKeypoints, objectDescriptors);//Calculate the keypoint descriptors from the current frame
                    objectList.add(new trackedObject(roIKeypoints, objectDescriptors,rectangles[0]));
                    std::cout <<"Created new"<< std::endl;
                    if(init){pause = 1;}
                }
////////
                //anchors.push_back(trackedObject(roIKeypoints, objectDescriptors,rectangles[0]));
                frame.copyTo(obj);
                //if(!init){
                //    frame.copyTo(prevFrame);//Förskjut så att vi har både en frame och en prevFrame
                //    init=1;
                //}
            }
        }
        //This must be carefully placed. so that new keypoints are tracked
        success = tracker.trackOpticalFlow(prevFrame,frame,roIPoints,rectangles[0]);
        tracker.drawPoints(mat4visual,roIPoints,mat4visual,CV_RGB(255,0,0));
        cv::rectangle(mat4visual,rectangles[0],CV_RGB(255,0,0),2,cv::LINE_8,0);
        cv::imshow("VideoStream",mat4visual);
        if(pause){pause = 0;cv::waitKey(0);}
        if( cv::waitKey(1) == 27 ) {
            std::cout << "Bryter"<< std::endl;
            printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
            std::cout<< "Number of frames processed: " << totlaps << std::endl;
            objectList.clear();
            return 1;
        }
        frame.copyTo(prevFrame);
        //matches.clear();
        //best_matches.clear();
        //usleep(20000);
    }
objectList.clear();
return 1;
}
