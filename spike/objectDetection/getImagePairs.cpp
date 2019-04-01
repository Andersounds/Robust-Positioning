#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep
#include "../../src/videoStream.cpp"
#include "../../src/argumentParser.cpp"
#include "../../src/KLT_ORB_Tracker.cpp"

using namespace cv;
//using namespace std;



int main(int argc, char** argv){
  // check if inputs are correctly given
  streamArguments arguments;
  if(!argumentParser(argc,argv,arguments)){return 0;}


  int noOfClusters = 1;
  int clusterSize = 100;
  int minDistance = 100;
  String fileEnding = ".png";
  String path = "spike/objectDetection/images/";
  String objectName = "objects/obj_";
  String sceneName = "scenes/scene_";




  KLT_ORB_Tracker sceneTracker, objectTracker;
  objectTracker.setNfeatures(100);
  objectTracker.init();
  sceneTracker.init();

  Streamer* C;
  C = Streamer::make_streamer(arguments.streamMode,arguments.datasetPath,arguments.cam);


  int counter= 0;
  int basecounter = 0;
  while(1){
  std::vector< cv::KeyPoint > keypoints;
  cv::Mat frame;
  std::vector<cv::Mat> returnDescriptors;
  C->getImage(frame);
  if(frame.empty()){return 1;}

  sceneTracker.getFeatures(frame,keypoints);                              // Get all keypoints in scene
  std::cout << "-1";
  std::vector<std::vector<cv::KeyPoint>> returnKeyPoints;                 // Initialize returnkeypoints again
  //std::cout << "-2";
  std::vector<cv::Rect> rectangles = sceneTracker.findClusters(frame,          //Get rect which contains the object
                                                          keypoints,
                                                          noOfClusters,
                                                          clusterSize,
                                                          minDistance,
                                                          returnKeyPoints,
                                                          returnDescriptors);//And the keypoints in scene where object is

    std::cout << "4";
    if(counter>50){

////
      cv::Rect imgRect = cv::Rect(cv::Point(0,0), frame.size());
      cv::Rect rectsIntersecion = imgRect & rectangles[0];
      if(rectsIntersecion.area() == rectangles[0].area()){
/////
        cv::Mat drawnImage;
        frame.copyTo(drawnImage);
        cv::rectangle(drawnImage,rectangles[0],CV_RGB(255,0,0),2,cv::LINE_8,0);
        cv::imshow("Found rect", drawnImage);
        //cv::waitKey(0);



        //std::cout << "-4";
        cv::Mat RoI = cv::Mat(frame,rectangles[0]);// get only RoI
        //std::cout << "-5" << std::endl;
        //Write image pair to disk
        String nmbr = std::to_string(basecounter);
        String strNmbr;
        if(nmbr.size() == 1){strNmbr = "000" + nmbr;}
        if(nmbr.size() == 2){strNmbr = "00" + nmbr;}
        if(nmbr.size() == 3){strNmbr = "0" + nmbr;}
        if(nmbr.size() == 4){strNmbr = nmbr;}
        String obj = "";
        obj+= (path + objectName + strNmbr + fileEnding);
        String scene = "";
        scene += (path + sceneName + strNmbr + fileEnding);
        int a = cv::imwrite(obj, RoI);
        cv::imwrite(scene, frame);
        //imshow("Showit",latestFrame);
        //cv::waitKey(0);
        //if( cv::waitKey(1) == 27 ) break;                         // stop capturing by pressing ESC
        std::cout << "wrote image to disk: " << a << std::endl;
        std::cout << obj << std::endl;
        counter = 0;
        basecounter++;
      }
    }
    counter++;
    //if(basecounter >4){return 1;}
  }
}
