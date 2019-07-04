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
 * Abstract base videoStream class
 */
class Streamer{
  public:
    static Streamer *make_streamer(int choice);//Just choose between usb or rpi. if dataset must be chosen with path
    static Streamer *make_streamer(int choice,int cam);//For choosing usb cam and another internal cam than 0
    static Streamer *make_streamer(std::string datasetPath);//For choosing dataset. Just reading images
    static Streamer *make_streamer(int choice, std::string datasetPath);//For either choosing just dataset or dataset with timestamps
    virtual int getImage(cv::Mat&) = 0; //Equals zero to declare it and create vtable
    virtual int setSettings(int) = 0; //General settings method
};


/*
 *Physical camera
 */
class usbCamStreamer : public Streamer{
  int initialized;
  int cameraNmbr;
  public:
    usbCamStreamer(int);      /*Constructor*/
    int getImage(cv::Mat&);   /*Get next image interface*/   //in-place return of Mat
    int setSettings(int);     /*Settings*/
};
/*
 *Dataset stream
 */
class datasetStreamer : public Streamer{
  int initialized;
  bool useTimeStampFile;
  std::string pathToDataset;
  std::vector<std::string> timeStamps_s;
  std::vector<float> timeStamps_f;
  std::vector<std::string> imgNames;
  public:
    datasetStreamer(std::string,bool);
    float getImage(cv::Mat&); //This method returns the image via inputoutput and timestamp via return if timestamp mode is chosen
    int setSettings(int);
    int readTimeStampData(void);
};
/*
Raspberry pi camera module stream
*/
class piCamStreamer : public Streamer{
  int mode;
  std::string pathToDataset;
  public:
    piCamStreamer(void);
    int getImage(cv::Mat&);
    int setSettings(int);
};


}
#endif
