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
int parsePaths(std::vector<std::string>& paths,std::vector<int>& params_,int argc, char** argv){
    std::string basePath = "";
    std::string newDirName = "";
    std::string csvDataName = "imudata.csv";
    int cp1;
    bool cp1Set = false;
    int cp2;
    std::string fileEnding = ".bmp";//default to png
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
            }else if(flag == "-cp1"){
                try{
                    cp1 = stoi(arg);
                    cp1Set = true;
                } catch(const std::invalid_argument& ia){
                    std::cout << "Gave invalid int for cp1: " << arg << std::endl;
                }
            }else if(flag == "-cp2"){
                try{
                    cp2 = stoi(arg);
                    if(cp1Set){
                        params_.push_back(cp1);
                        params_.push_back(cp2);
                        cp1Set = false;
                    }else{
                        std::cout << "cp2 set without specifying cp1.." << std::endl;
                    }
                } catch(const std::invalid_argument& ia){
                    std::cout << "Gave invalid int for cp2: " << arg << std::endl;
                }
            }else if(flag == "-cp3"){
                fileEnding = arg;
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
        std::cout << "-cp1 optional    imwrite compression parameter 1 (name) " << std::endl;
        std::cout << "-cp2 optional    imwrite compression parameter 1 (name) " << std::endl;
        std::cout << "-cp3 optional    imwrite file ending" << std::endl;
        return 0;
    }else{
        std::cout << "Logging to directory " << basePath << newDirName << std::endl;
        std::cout << "Logging csv data to file " << csvDataName << std::endl;
        paths.clear();
        paths.push_back(basePath);
        paths.push_back(newDirName);
        paths.push_back(basePath + newDirName +"/" + csvDataName);
        paths.push_back(fileEnding);
    }
    return laps;
}


int main(int argc, char** argv){
    timestamp::timeStamp_ms stamp;
    std::vector<std::string> paths;
    std::vector<int> imWriteParams;
    int laps = parsePaths(paths,imWriteParams,argc,argv);
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
    ///////// TESTING VIDEO WRITER
    Size frame_size(640, 480);
    double frames_per_second = 30;
    cv::VideoWriter oVideoWriter("../TryOuts/MyVideo.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),frames_per_second, frame_size, false);
   if (oVideoWriter.isOpened() == false){
       cout << "Cannot save the video to a file" << endl;
       return 0;
   }


    ///////// TESTING VIDEO WRITER

    //Initialize video stream
    robustPositioning::Streamer VStreamer(robustPositioning::MODE_RPI_CAM,CV_8UC1);
    imagebin.params = imWriteParams;
    imagebin.imgFormatStr = paths[3];
    cv::Mat frame;
    //Initialize data stream
    //initialize i2c slave object with the inherited encode/decode class
    //const int slaveAddress = 0x04;
    robustpositioning::i2cSlave_decode i2cComm(0x04);

    std::vector<float> data{0,0,0,0,0};
    float dist = 0;
    float height = 0;
    float pitch = 0;
    float roll = 0;
    float batt = 0;
    //Read data until done
    float timeStamp_data;
    double timeStamp_image;
    float counter = 0;
    double timeStamp; //
    stamp.get(timeStamp);//Initialize to get start value
    while(counter<laps){
        i2cComm.clearRxBuffer();                            //Clear data so that new can be recieved
        stamp.get(timeStamp_data);                          //Set data timestamp
        stamp.get(timeStamp_image);                         //Set image timestamp
        VStreamer.getImage(frame);			    //Get image
//	    imagebin.dump(timeStamp_image,frame);		    //Log image (By doing this now we give some extra time  for i2c)
    oVideoWriter.write(frame);
    // Read i2c message
        float watchdog=0;//Wait maximal 0.5s on imu data
        int recv_amount = i2cComm.readAndDecodeBuffer(data);;//Number of recieved and decoded floats
        while(recv_amount<0 && watchdog<10){          //Try to read until we get the requested data. max 1/10 s
	        usleep(2000);//Wait 2 ms
	        watchdog++;
	        recv_amount = i2cComm.readAndDecodeBuffer(data);
        }
	//Only update the variables that  have been recieved. And do not try to access outside bounds of data vector.
	switch(recv_amount){
	    case 5:batt = data[4];
	    case 4:height = -data[3];
	    case 3:dist = data[2];
	    case 2:roll = data[1];
	    case 1:{pitch = data[0];
			std::cout <<counter <<". WD: "<< watchdog << ". Recieved " << recv_amount <<" decoded floats. roll: "<< roll<< ", batt: " << batt << std::endl;
			break;}
	    case 0:{std::cout << "No available data in rx buffer" << std::endl;break;}
	    default:{std::cout << "Recieved more than 5 decoded floats? " << recv_amount << std::endl;}
	}
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
    oVideoWriter.release();
    return 0;
}
