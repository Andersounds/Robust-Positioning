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

#include "../src/vopos.hpp"

//#include "../include/raspicam-0.1.6/src/raspicam_cv.h" //Can we just include <raspicam/raspicam_cv.h>? since it is installed?
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>

#include <regex>

#include <unistd.h> //For sleep
#define PI 3.1416

/*
TODO::
 - Read settings from settings file instead
 - specify resolution via settings file



*/



/*
-p relative path to directory where new directory is to be saved (optional. if skipped then it is created in pwd)
-d new directory name
-f wanted filename of the data logger
*/
int parsePaths(int argc, char** argv){
    bool gotLapNumber = false;
    int laps = 100;
    for(int i=0;i<argc;i++){
        std::string flag = argv[i];
        std::string arg;
        if((i+1)<argc){
            arg = argv[i+1];
            if(flag == "-l"){
                try{
                    laps = stoi(arg);
                } catch(const std::invalid_argument& ia){
                    std::cout << "Gave invalid number of laps to run (specified with flag -l): " << arg << std::endl;
                }
                std::cout << "Set number of laps to: " << laps << std::endl;
            }else if(flag == "-w"){
	        try{
                    waitSeconds = stoi(arg);
                } catch(const std::invalid_argument& ia){
                    std::cout << "Gave invalid int for -w wait [s]: " << arg << std::endl;
                }
   	        }
        }
    }
    if(!gotLapNumber){
        std::cout << "Usage:" << std::endl;
        std::cout << "Flag Remark      Purpose" << std::endl;
        std::cout << "-l   optional    Number of laps to do. default to 100." << std::endl;
        std::cout << "-w   optional    wait before starting data collection [s]" << std::endl;
        std::cout << "TODO:  choose to ignore atsam data. and just give some values to vo algorithm" << std::endl;
        return 0;
    }
    return laps;
}


int main(int argc, char** argv){
    //Initialize settings
    set::settings S(argc,argv);
    if(!S.success()){return 0;}
    int laps = parsePaths(argc,argv);
    if(!laps){return 0;}

    timestamp::timeStamp_ms stamp;
    //Initialize positioning object
    int maxIdAruco = 50;
    std::string anchorPath = S.data.anchorPath;//"anchors.txt";
    cv::Rect2f roiSize = S.data.ROI;
    cv::Mat_<float> K = S.data.K;
    cv::Mat_<float> T = S.data.T;//warper.getZRot(-PI/2);//UAV frame is x forward, camera frame is -y forward
    int OpticalFlowMode = S.data.MODE_OpticalFlow;
    int VisualOdometryMode = S.data.MODE_VisualOdometry;
    int flowGrid = S.data.optical_flow_grid;
    pos::positioning P(OpticalFlowMode,
                        flowGrid,
                        cv::aruco::DICT_4X4_50,
                        maxIdAruco,anchorPath,flowGrid,roiSize,K,T);
    P.setDistortCoefficents(S.data.dist_k1,S.data.dist_k2,S.data.dist_k3);//Set distortion coefficients
    //Init values of position and yaw
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = S.data.x0;
    t(1,0) = S.data.y0;
    t(2,0) = S.data.z0;
    float yaw = S.data.yaw0;

    //Initialize databin
    robustPositioning::dataLogger databin_LOG;
    if(!databin_LOG.init(paths[2],std::vector<std::string>{"Timestamp [ms]","dist [m]","height [m]","pitch [rad]","roll [rad]","x","y","z","yaw [rad]"})) return 0;
    //Initialize video stream
    robustPositioning::Streamer VStreamer(robustPositioning::MODE_RPI_CAM,CV_8UC1);
    imagebin.params = imWriteParams;
    imagebin.imgFormatStr = paths[3];
    cv::Mat frame;
    //Initialize data stream
    //initialize i2c slave object with the inherited encode/decode class
    //const int slaveAddress = 0x04;
    robustpositioning::i2cSlave_decode i2cComm(0x04);
    std::vector<float> data{0,0,0,0,0,0,0};
    float dist = 0;
    float height = 0;
    float pitch = 0;
    float roll = 0;
    float batt = 0;
    //Read data until done
    float timeStamp_data;
    float counter = 0;
    double timeStamp; //
    stamp.get(timeStamp);//Initialize to get start value
    int estimationMode = S.data.MODE_Positioning;
    while(counter<laps){
        i2cComm.clearRxBuffer();                            //Clear data so that new can be recieved
        stamp.get(timeStamp_data);                          //Set data timestamp
        stamp.get(timeStamp_image);                         //Set image timestamp
        VStreamer.getImage(frame);			    //Get image
        //imagebin.dump(timeStamp_image,frame);		    //Log image (By doing this now we give some extra time  for i2c)
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
    //Positioning...
        int mode = P.process(estimationMode,frame,height, roll, pitch, yaw, t);


        //Log data
        std::vector<float> logData{timeStamp_data, dist, height,pitch, roll,t(0,0),t(0,1),t(0,2),yaw};
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
