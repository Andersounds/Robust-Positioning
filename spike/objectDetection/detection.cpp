#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath> //For sleep
//#include <random>//For index sampling
#include "../../src/videoStream.hpp"
#include "../../src/argumentParser.cpp"
#include "../../src/KLT_ORB_Tracker.hpp"
#include "../../src/houghDetector.hpp"

//using namespace cv;
//using namespace std;
using std::cout;
using std::endl;

int main(int argc, char** argv){
  // check if inputs are correctly given
  streamArguments arguments;
  if(!argumentParser(argc,argv,arguments)){return 0;}
  //String path = "/Users/Fredrik/Datasets/picsForMatching/objectDetection/images/scenes/scene_%04d.png";
  cv::String scenestr = "scenes/scene_%04d.png";
  cv::String objstr = "objects/obj_%04d.png";
  cv::String Tempscenestr = "scenes/scene_0051.png";
  cv::String Tempobjstr = "objects/obj_0052.png";
  int noOfClusters = 1;
  int clusterSize = 40;
  int minDistance = 100;


  KLT_ORB_Tracker sceneTracker, objectTracker;
  objectTracker.setNfeatures(100);
  sceneTracker.setNfeatures(300);
  objectTracker.init();
  sceneTracker.init();


  cv::Ptr<cv::BFMatcher> matcherObject = cv::BFMatcher::create(cv::NORM_HAMMING,true);
  std::vector<cv::DMatch> matches;
  std::vector< cv::DMatch > good_matches;
  cv::Mat frame;
  cv::Mat object;

  cv::VideoCapture cap;
  cv::VideoCapture cop;
  cap.open(arguments.datasetPath+scenestr);
  cop.open(arguments.datasetPath+objstr);
  //cap.read(frame);//Offset with one image
  int counter = 0;
  while(1){
    cv::Mat img_matches;

    // Get new scene and object pair
    cap.read(frame);
    cop.read(object);
    if(frame.empty()||object.empty()){std::cout << "Done." << std::endl;return 1;}
    //Get keypoints and descriptors from both scene and object
    std::vector< cv::KeyPoint > sceneKeypoints, objectKeypoints;
    objectTracker.getFeatures(object,objectKeypoints);
    sceneTracker.getFeatures(frame,sceneKeypoints);
    //objectTracker.drawPoints(object,objectKeypoints,object,CV_RGB(0,200,0));
    //objectTracker.drawPoints(frame,sceneKeypoints,frame,CV_RGB(0,200,0));

    if(objectKeypoints.size()>1&&sceneKeypoints.size()>1){//Must be at least one pair
        cv::Mat sceneDescriptors;
        cv::Mat objectDescriptors;
//--
cv::destroyAllWindows();
sceneTracker.drawPoints(frame, sceneKeypoints,frame,CV_RGB(200,0,255));
cv::imshow("Scen keypoints", frame);
objectTracker.drawPoints(object, objectKeypoints,object,CV_RGB(200,0,255));
cv::imshow("object keypoints", object);
cv::waitKey(0);


//------

        sceneTracker.orbObject->compute(frame, sceneKeypoints, sceneDescriptors);
        objectTracker.orbObject->compute(object, objectKeypoints, objectDescriptors);//Här kraschar den
        matcherObject ->match(objectDescriptors,sceneDescriptors,matches);
        //getGoodMatches(matches, good_matches, objectKeypoints, sceneKeypoints);
        hd::getGoodMatches(matches, good_matches);
        std::vector<cv::Point> xWinningPairs, yWinningPairs;
        //int status = detect(object, frame, good_matches, objectKeypoints,sceneKeypoints, xWinningPairs, yWinningPairs);

/*//-------------
        cv::destroyAllWindows();
        cv::drawMatches( object, objectKeypoints, frame, sceneKeypoints,
                       good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                       std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::imshow("All Matches", img_matches);
        cv::waitKey(0);
        cv::destroyAllWindows();
        cv::drawMatches( object, objectKeypoints, frame, sceneKeypoints,
                       good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                       std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::imshow("Good Matches", img_matches);
        cv::waitKey(0);
        cv::destroyAllWindows();

//------------*/
        std::vector<float> status;
        status = hd::detect(good_matches, objectKeypoints,sceneKeypoints, xWinningPairs, yWinningPairs);

        if(status[0]){cout << "Match!" << endl;}else{cout << "No match" << endl;}

        std::vector<cv::DMatch> best_matches = hd::getBestMatches(good_matches,xWinningPairs, yWinningPairs);

        cv::drawMatches( object, objectKeypoints, frame, sceneKeypoints,
                       best_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                       std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::imshow("Best Matches", img_matches);
        cv::waitKey(0);
    }else{std::cout << "No keypoints. skipping this." << std::endl;}



    counter++;

    cout << counter << endl;

  }
}
