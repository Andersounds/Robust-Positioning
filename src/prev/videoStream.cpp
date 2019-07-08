#include <iostream>
#include <opencv2/opencv.hpp>
#include "videoStream.hpp"


/*Factory method for abstract streamer class with default settings*/
Streamer *Streamer::make_streamer(int choice, std::string path, int cam)
{
  if      (choice == 1)
  {
    if(cam==0){return new usbCamStreamer(0);}//Default cam
    else {return new usbCamStreamer(cam);}//
  }
  else if (choice == 3)
  {
    if(path==""){return new datasetStreamer("/Users/Fredrik/Datasets/Euroc/V101/cam0/data/cam0_%04d.png");}//Default path
    else {return new datasetStreamer(path);}
  }
  else
  {
    std::cout << "No valid mode given. Defaulting to internal cam 0" << std::endl;
    return new usbCamStreamer(cam);
  }
}

/*
 *Constructor that saves camera number
 */
usbCamStreamer::usbCamStreamer(int nmbr){
    cameraNmbr = nmbr;
    initialized = 0;
    std::cout << "Mode 1. USB camera streamer object created." << std::endl;
}
/*
 *Get next image interface. Initializes streamer object upon first call
 */
int usbCamStreamer::getImage(cv::Mat& frame){
  static cv::VideoCapture cap;
  if(!initialized){
    if(!cap.open(cameraNmbr)){std::cerr << "Internal camera number not valid:" << cameraNmbr <<std::endl;return 0;}
    initialized=1;
  }
  cap.read(frame);
  if( frame.empty() ){std::cout << "Stream done." << std::endl; return 0;}//If stream is done return 0
  return 1;
}
int usbCamStreamer::setSettings(int test){
  return 1;
}




/*
 *Constructor that saves path to dataset
 */
datasetStreamer::datasetStreamer(std::string path){
    pathToDataset = path;
    initialized = 0;
    std::cout << "Mode 3. Datastreamer object created." << std::endl;
}
/*
 *Get next image interface. Initializes streamer object upon first call
 */
int datasetStreamer::getImage(cv::Mat& frame){
  static cv::VideoCapture cap;
  if(!initialized){
    if(!cap.open(pathToDataset)){std::cerr << "Path to dataset not valid:" << pathToDataset <<std::endl;return 0;}
    initialized=1;
  }
  cap.read(frame);
  if( frame.empty() ){std::cout << "Stream done." << std::endl; return 0;}//If stream is done return 0
  return 1;
}

int datasetStreamer::setSettings(int test){
  return 1;
}
