#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep
#include "../src/videoStream.hpp"
#include "../src/argumentParser.cpp"
#include <opencv2/aruco.hpp>

using namespace cv;
//using namespace std;



int main(int argc, char** argv){
    // check if inputs are correctly given
    streamArguments arguments;
    if(!argumentParser(argc,argv,arguments)){return 0;}

    Streamer* C;
    C = Streamer::make_streamer(arguments.streamMode,arguments.datasetPath,arguments.cam);

    cv::Mat colorFrame;
    while(1){
    C->getImage(colorFrame);
    if(colorFrame.empty()){std::cout << "Image empty. " << std::endl;return 1;}

    cv::Mat frame;
    cv::cvtColor(colorFrame, frame, cv::COLOR_BGR2GRAY,1);
    //Aruco magic
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
    cv::aruco::drawDetectedMarkers(colorFrame, corners, ids, CV_RGB(0,250,0));
    std::cout << "size: " << corners.size() << std::endl;





    imshow("Showit",colorFrame);
    if( waitKey(1) == 27 ) break;                         // stop capturing by pressing ESC
    }


}
