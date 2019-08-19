#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
//#include "../src/save2file.cpp"
#include "../src/videoStream.hpp"
#include "../src/dataStream.hpp"
#include "../src/settingsParser.cpp"

#include "../src/logger.hpp"
#include "../src/timeStamp.cpp"
#include "../src/i2c_slave.cpp"

//#include "../include/raspicam-0.1.6/src/raspicam_cv.h" //Can we just include <raspicam/raspicam_cv.h>? since it is installed?
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>

#include <unistd.h> //For sleep
#define PI 3.1416



int main(int argc, char** argv){
    timestamp::timeStamp_ms stamp;
    //Can these two rows be written as stamp.get(double timeStamp); is timeStamp available in this scope then?
    double timeStamp; //Can it be float? can i just give something else?
    stamp.get(timeStamp);//Initialize .. Do i need to even?

    //Initialize imagebin. It automatically creates a directory 'images' in the given path
    robustPositioning::imageLogger imagebin;
    imagebin.init("../TryOuts/dataColl/","32_jul");
    //Initialize databin
    robustPositioning::dataLogger databin_LOG;
    if(!databin_LOG.init("../TryOuts/dataColl/32_jul/imuData.csv",std::vector<std::string>{"Timestamp [ms]","dist [mm]","height [mm]","pitch [rad]","roll [rad]","watchdog [ms]"})) return 0;

    //Initialize settings
    set::settings S(argc,argv);
    if(!S.success()){return 0;}

    //Initialize video stream
    robustPositioning::Streamer VStreamer(robustPositioning::MODE_RPI_CAM);
    cv::Mat frame;
    //Initialize data stream
    //initialize i2c slave object with the inherited encode/decode class
    //const int slaveAddress = 0x04;
    robustpositioning::i2cSlave_decode i2cComm(0x04);

    std::vector<float> data{0,0,0,0};

    //Read data until done
    float timeStamp_data;
    double timeStamp_image;
    int counter = 0;
    while(counter<100){
        i2cComm.clearRxBuffer();                            //Clear data so that new can be recieved
        stamp.get(timeStamp_data);                          //Set data timestamp
        stamp.get(timeStamp_image);                         //Set image timestamp
        VStreamer.getImage(frame);
        int watchdog=0;//Wait maximal 0.5s on imu data
        while(i2cComm.readAndDecodeBuffer(data)<0 && watchdog<500){          //Try to read until we get the requested data
            usleep(1000);//Wait an additional ms
	    watchdog++;
        }
        float dist = -data[0];//This is used as a subst as actual height is not in dataset
        float height = data[1];//This may drift significantly. When processing, offset it to correct value when possible
        float pitch = data[2];
        float roll = data[3];


        //Log data
	//std::cout << "Roll: " << roll << ", pitch: " << pitch << std::endl;
        std::vector<float> logData{timeStamp_data, dist, height,pitch, roll,watchdog};
        databin_LOG.dump(logData);
        imagebin.dump(timeStamp_image,frame);

        counter++;
    }
    return 0;
}
