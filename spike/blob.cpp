#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep
#include "../src/VisPos.cpp"
#include "../src/videoStream.cpp"

using namespace cv;
using namespace std;




int main(int argc, char** argv){
  // check if inputs are correctly given

  if ( (argc != 2) and (argc != 3) ) {
    cerr << "Usage: out <mode> [<path to dataset>]" << endl;
    cerr << "Press enter..." << endl;
    cin.get();
    return 0;
  }
  string mode = argv[1];


  Mat latestFrame;
  if(mode == "1"){                                          //If internal camera
    int camNmbr = stoi(argv[2]);
    usbCamStreamer C(camNmbr);
    while(1){
      C.getImage(latestFrame);
      //cvtColor(latestFrame, latestFrame, COLOR_BGR2GRAY);
      //threshold(latestFrame, latestFrame,128,255,THRESH_BINARY);//bör ha dynamisk threshhold


      // Set up the detector with default parameters.
      SimpleBlobDetector detector;

      // Detect blobs.
      std::vector<KeyPoint> keypoints;
      detector.detect(latestFrame, keypoints);

      // Draw detected blobs as red circles.
      // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    //  Mat im_with_keypoints;
      //drawKeypoints(latestFrame, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

      // Show blobs
    //  imshow("keypoints", im_with_keypoints );


      if( waitKey(1) == 27 ) break;                         // stop capturing by pressing ESC
    }


  }
}
