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

#include <unistd.h> //For sleep
#define PI 3.1416



int main(int argc, char** argv){
    timestamp::timeStamp_ms stamp;
    //Can these two rows be written as stamp.get(double timeStamp); is timeStamp available in this scope then?
    double timeStamp; //Can it be float? can i just give something else?
    stamp.get(timeStamp);//Initialize .. Do i need to even?

    //Initialize imagebin. It automatically creates a directory 'images' in the given path
    robustPositioning::imageLogger imagebin;
    imagebin.init("Generated_datasets/","25_jul");
    //Initialize databin
    robustPositioning::dataLogger databin_LOG;
    if(!databin_LOG.init("Generated_datasets/25_jul/imuData.csv",std::vector<std::string>{"Timestamp [ms]","height [m]","pitch [rad]","roll [rad]"})) return 0;

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

    std::string dataFile = S.data.dataStreamFile;
    int skipLines = 1;
    robustPositioning::dataStreamer getData(dataFile,skipLines);
    std::vector<float> data;



    //Read data until done
    float timeStamp_data;
    double timeStamp_image;
    int counter = 0;
    while(counter<100){
        i2cComm.clearRxBuffer();                            //Clear data so that new can be recieved
        stamp.get(timeStamp_data);                          //Set data timestamp
        VStreamer.getImage(frame);
        stamp.get(timeStamp_image);                          //Set image timestamp
        int watchdog=0;//Wait maximal 0.5s on imu data
        while(i2cComm.readAndDecodeBuffer(data)<0 && watchdog<500){          //Try to read until we get the requested data
            usleep(1000);//Wait an additional ms
        }
        float height = -data[0];//This is used as a subst as actual height is not in dataset
        float pitch = data[1];
        float roll = data[2];

        //Log data
        std::vector<float> logData{timeStamp_data, height, pitch, roll};
        databin_LOG.dump(logData);
        imagebin.dump(timeStamp_image,frame);

        counter++;
    }
    return 0;
}
