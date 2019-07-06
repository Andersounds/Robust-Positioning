/*
 * These classes are written to achieve a common interface to the three video streaming
 * possibilities:
 * - 1. Physical camera of computer
 * - 2. RPi camera module
 * - 3. Stream from dataset
 */
#ifndef VIDEOSTREAM_H
#define VIDEOSTREAM_H
#include <iostream>
#include <opencv2/opencv.hpp>


namespace robustPositioning{
    //Different modes for what kind of stream that is to be used
    const int MODE_USB_CAM = 0;
    const int MODE_RPI_CAM = 1;
    const int MODE_IMG_STREAM = 2;
    const int MODE_IMG_TIMESTAMP_STREAM = 3;


/*
 *Physical camera
 */
class usbCamStreamer{// : public Streamer{
  int initialized;
  int cameraNmbr;
  public:
    usbCamStreamer(void);
    usbCamStreamer(int);      /*Constructor*/
    float getImage(cv::Mat&);   /*Get next image interface*/   //in-place return of Mat
    int setSettings(int);     /*Settings*/
};
/*
 *Dataset stream
 */
class datasetStreamer{// : public Streamer{
  int initialized;
  int sequence;
  int datasetSize;
  bool useTimeStampFile;
  std::string pathToDataset;//Path to directory where image files are located. Including "/"
  std::string dataFile;
  std::vector<std::string> timeStamps_s;
  std::vector<float> timeStamps_f;
  std::vector<std::string> imgNames;
  std::vector<std::string> parseRow(std::string);
  public:
    datasetStreamer(void);//Default constructor. This one is called from the Streamer constructor if datasetstreamer is NOT used.
    datasetStreamer(std::string); //This constructor is for just cv::cap stream
    datasetStreamer(std::string,std::string,bool);
    float getImageViaCapStream(cv::Mat&);
    float getImageViaTimestampList(cv::Mat&);
    float getImage(cv::Mat&); //This method returns the image via inputoutput and timestamp via return if timestamp mode is chosen
    float peek(void);
    int setSettings(int);
    int readTimeStampData(void);
};
/*
Raspberry pi camera module stream
*/
class piCamStreamer{// : public Streamer{
  int mode;
  //std::string pathToDataset;
  public:
    piCamStreamer(void);
    float getImage(cv::Mat&);
    int setSettings(int);
};




/*
 * Master streamer class
 */
class Streamer{
  public:
/*    static Streamer *make_streamer(int choice);//Just choose between usb or rpi. if dataset must be chosen with path
    static Streamer *make_streamer(int choice,int cam);//For choosing usb cam and another internal cam than 0
    static Streamer *make_streamer(std::string datasetPath);//For choosing dataset. Just reading images
    static Streamer *make_streamer(int choice, std::string datasetPath);//For choosing dataset but also passing the choice
    static Streamer *make_streamer(std::string basePath, std::string dataFile);//For choosing dataset stream with data file and timestamps
    static Streamer *make_streamer(int choice, std::string basePath, std::string dataFile);//same as above but with explicit choice
*/
    Streamer(int choice);//Just choose between usb or rpi. if dataset must be chosen with path
    Streamer(int choice,int cam);//For choosing usb cam and another internal cam than 0
    Streamer(std::string datasetPath);//For choosing dataset. Just reading images
    Streamer(int choice, std::string datasetPath);//For choosing dataset but also passing the choice
    Streamer(std::string basePath, std::string dataFile);//For choosing dataset stream with data file and timestamps
    Streamer(int choice, std::string basePath, std::string dataFile);//same as above but with explicit choice
    float getImage(cv::Mat&); //Equals zero to declare it and create vtable
    float peek(void);
private:
    int chosenMode;
    usbCamStreamer usbcamSTR;
    datasetStreamer datasetSTR;
    piCamStreamer picamSTR;
};



}
#endif
