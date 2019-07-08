/*
 * These classes are written to achieve a common interface to the three video streaming
 * possibilities:
 * - 1. Physical camera of computer
 * - 2. RPi camera module
 * - 3. Stream from dataset
 */
#ifndef VIDEOSTREAM_H
#define VIDEOSTREAM_H
/*
 * Abstract base videoStream class
 */
class Streamer{
  public:
    static Streamer *make_streamer(int choice, std::string path, int cam);//Choose another internal camera
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
  std::string pathToDataset;
  public:
    datasetStreamer(std::string);
    int getImage(cv::Mat&);
    int setSettings(int);
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
#endif
