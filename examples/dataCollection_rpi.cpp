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

#include <regex>

#include <unistd.h> //For sleep
#define PI 3.1416

/*
-p relative path to directory where new directory is to be saved (optional. if skipped then it is created in pwd)
-d new directory name
-f wanted filename of the data logger
*/
int parsePaths(std::vector<std::string>& paths,int argc, char** argv){
    std::string basePath = "";
    std::string newDirName = "";
    std::string csvDataName = "imudata.csv";
    int laps = 100;
    bool gotDirName = false;
    for(int i=0;i<argc;i++){
        std::string flag = argv[i];
        std::string arg;
        if((i+1)<argc){
            arg = argv[i+1];
            if(flag == "-p"){
                //Make sure that dir ends with /
                if(std::regex_match(arg,std::regex("(.*)(/)"))){
                    basePath = arg;
                }else{
                    basePath = arg + "/";
                }
                std::cout << "Set base path to: " << basePath << std::endl;
            }else if(flag == "-d"){
                newDirName = arg;
                gotDirName = true;
                std::cout << "Set dir name to: " << newDirName << std::endl;
            }else if(flag == "-f"){
                //Make sure that file ending is there
                if(std::regex_match(arg,std::regex("(.*)(.csv)"))){
                    csvDataName = arg;
                }else{
                    csvDataName = arg + ".csv";
                }
                std::cout << "Set file name to: " << csvDataName << std::endl;
            }else if(flag == "-l"){
                try{
                    laps = stoi(arg);
                } catch(const std::invalid_argument& ia){
                    std::cout << "Gave invalid number of laps to run (specified with flag -l): " << arg << std::endl;
                }
                std::cout << "Set number of laps to: " << laps << std::endl;
            }
        }
    }
    if(!gotDirName){
        std::cout << "Usage:" << std::endl;
        std::cout << "Flag Remark      Purpose" << std::endl;
        std::cout << "-d   mandatory   Name of new directory in which data is logged" << std::endl;
        std::cout << "-f   optional    Name of csv file in which i2c data is to be saved. default imudata.csv" << std::endl;
        std::cout << "-p   optional    Relative path to directory in which new dir is to be created. Default to pwd. \"\" gives top of hierarchy"  << std::endl;
        std::cout << "-l   optional    Number of laps to do. default to 100." << std::endl;
        return 0;
    }else{
        std::cout << "Logging to directory " << basePath << newDirName << std::endl;
        std::cout << "Logging csv data to file " << csvDataName << std::endl;
        paths.clear();
        paths.push_back(basePath);
        paths.push_back(newDirName);
        paths.push_back(basePath + newDirName +"/" + csvDataName);
    }
    return laps;
}


int main(int argc, char** argv){
    timestamp::timeStamp_ms stamp;
    //Can these two rows be written as stamp.get(double timeStamp); is timeStamp available in this scope then?
    double timeStamp; //Can it be float? can i just give something else?
    stamp.get(timeStamp);//Initialize .. Do i need to even?
    std::vector<std::string> paths;
    int laps = parsePaths(paths,argc,argv);
    if(!laps){return 0;}
    //Initialize imagebin. It automatically creates a directory 'images' in the given path
    robustPositioning::imageLogger imagebin;
    imagebin.init(paths[0],paths[1]);
    //Initialize databin
    robustPositioning::dataLogger databin_LOG;
    if(!databin_LOG.init(paths[2],std::vector<std::string>{"Timestamp [ms]","dist [m]","height [m]","pitch [rad]","roll [rad]","watchdog [ms]"})) return 0;

    //Initialize settings
    //set::settings S(argc,argv);
    //if(!S.success()){return 0;}

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
    float counter = 0;
    while(counter<laps){
        i2cComm.clearRxBuffer();                            //Clear data so that new can be recieved
        stamp.get(timeStamp_data);                          //Set data timestamp
        stamp.get(timeStamp_image);                         //Set image timestamp
        VStreamer.getImage(frame);			    //Get image
	imagebin.dump(timeStamp_image,frame);		    //Log image (By doing this now we give some extra time  for i2c)
	// Read i2c message
        float watchdog=0;//Wait maximal 0.5s on imu data
        int recv_amount = i2cComm.readAndDecodeBuffer(data);;//Number of recieved and decoded floats
        while(recv_amount<0 && watchdog<500){          //Try to read until we get the requested data
            std::cout << "      WD: "<< watchdog << ", Recieved floats: " << recv_amount << ", roll: "<< data[3]<<std::endl;
	        usleep(2000);//Wait 2 ms
	        watchdog++;
	        recv_amount = i2cComm.readAndDecodeBuffer(data);
        }
	float ts_temp;
	stamp.get(ts_temp);
	float time = ts_temp-timeStamp_data;
	std::cout << "DONE. WD: "<< watchdog << ",("<<time <<"ms), Recieved floats: " << recv_amount << ", roll: " << data[3] <<std::endl;
	float dist = -data[0];//This is used as a subst as actual height is not in dataset
        float height = data[1];//This may drift significantly. When processing, offset it to correct value when possible
        float pitch = data[2];
        float roll = data[3];

        //Log data
	//    std::cout << "Roll: " << roll << ", pitch: " << pitch << std::endl;
        //std::cout << "Watchdog: " << watchdog << " [ms]" << std::endl;
        std::vector<float> logData{timeStamp_data, dist, height,pitch, roll,watchdog};
        databin_LOG.dump(logData);
        counter++;
    }
    stamp.get(timeStamp_data);
    float total_time_s = (timeStamp_data-timeStamp)/1000;
    float fps = counter/total_time_s;
    std::cout << "Data collection done." << std::endl;
    std::cout << counter << " frames in " << total_time_s << " seconds. (" << fps << " fps)" << std::endl;
    return 0;
}
