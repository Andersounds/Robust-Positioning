#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
//#include <chrono> //For timestamps
#include "../src/vopos.hpp"
#include "../src/simulatePose.hpp"
//#include "../src/save2file.cpp"
#include "../src/videoStream.hpp"
#include "../src/dataStream.hpp"
#include "../src/settingsParser.cpp"

#include "../src/logger.hpp"
#include "../src/timeStamp.cpp"
#define PI 3.1416


#include <typeinfo>
int main(int argc, char** argv){
timestamp::timeStamp_ms stamp;
//Can these two rows be written as stamp.get(double timeStamp); is timeStamp available in this scope then?
double timeStamp;
stamp.get(timeStamp);

/*
//Initialize imagebin. It automatically creates a directory 'images' in the given path
robustPositioning::imageLogger imagebin;
imagebin.init("","5_jul");

robustPositioning::dataLogger databin_LOG;
//if(!databin_EST.init("estPath.csv",std::vector<std::string>{"Timestamp [ms]","x[m]","y[m]","z[m]","yaw[rad]","mode"})) return 0;
if(!databin_LOG.init("5_jul/truePath.csv",std::vector<std::string>{"Timestamp [ms]","x [m]","y [m]","z [m]","yaw [rad]","pitch [rad]","roll [rad]"})) return 0;


*/



    //Initialize settings
    set::settings S(argc,argv);
    if(!S.success()){return 0;}


    //Initialize video stream
    std::string basePath = S.data.imageStreamBasePath;
    std::string imageInfo = S.data.imageStreamInfoFile;
    robustPositioning::Streamer VStreamer(basePath,imageInfo);
    cv::Mat frame, colorFrame;
    //Initialize data stream
    std::string dataFile = S.data.dataStreamFile;
    int skipLines = 1;
    robustPositioning::dataStreamer getData(dataFile,skipLines);
    std::vector<float> data;
    //Initialize data logger
    robustPositioning::dataLogger databin_LOG;
    if(!databin_LOG.init("log.csv",std::vector<std::string>{"timestamp [ms]","X [m]","Y [m]","Z [m]","Yaw [rad]"})) return 0;


    //Initialize positioning object
    int maxIdAruco = 50;
    std::string anchorPath = S.data.anchorPath;//"anchors.txt";
    int flowGrid = S.data.optical_flow_grid;
    cv::Rect2f roiSize = S.data.ROI;
    cv::Mat_<float> K = S.data.K;
    cv::Mat_<float> T = S.data.T;//warper.getZRot(-PI/2);//UAV frame is x forward, camera frame is -y forward
    pos::positioning P(pos::OF_MODE_KLT,//pos::OF_MODE_CORR,//pos::OF_MODE_KLT,
                        pos::VO_MODE_AFFINE,//pos::VO_MODE_HOMOGRAPHY,//pos::VO_MODE_AFFINE,
                        cv::aruco::DICT_4X4_50,
                        maxIdAruco,anchorPath,flowGrid,roiSize,K,T);
    //Init values of position and yaw
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = S.data.x0;
    t(1,0) = S.data.y0;
    t(2,0) = S.data.z0;
    float yaw = S.data.yaw0;



//Read data until done
float timeStamp_data;
float timeStamp_image;
int counter = 0;
double timeStamp_start;
stamp.get(timeStamp_start);
float rad2Grad = 57.2958;
    while(getData.get(data)){
        timeStamp_data = data[0];
        float dist = data[S.data.distColumn];//This is used as a subst as actual height is not in dataset
        float pitch = 0;//data[S.data.pitchColumn];
        float roll = 0;//data[S.data.rollColumn];

        //####TEMP EDIT. give correct

//Get new image
        if(VStreamer.peek()<=timeStamp_data){
            VStreamer.getImage(frame);
            if(frame.empty()){std::cout << "Video stream done."<< std::endl; return 0;}
//            int mode = P.process(pos::MODE_AZIPE_AND_VO,frame,dist, roll, pitch, yaw, t);
            //int mode = P.process(pos::MODE_AZIPE,frame,dist, roll, pitch, yaw, t);
            cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
            int mode = P.processAndIllustrate(pos::MODE_AZIPE,frame,colorFrame,pos::ILLUSTRATE_ALL,dist,roll,pitch,yaw,t);
            //Log data
            if(true){
                std::vector<float> logData{timeStamp_data,t(0,0),t(1,0),t(2,0),yaw};
                databin_LOG.dump(logData);
            }
            std::cout << "X: "<< t(0,0) << ", Y: "<< t(1,0) << ", Z: " << t(2,0) <<", roll: " << roll*rad2Grad<<", pitch: " << pitch*rad2Grad << "yaw: " << yaw<< std::endl;
            cv::imshow("showit",colorFrame);
            //cv::waitKey(0);
            if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}

            std::cout << "Lap " << counter << std::endl;
            counter++;
      }


    //std::cout << "t: " << t.t() << std::endl;
        //Maybe convert to color for illustartion?
        //cv::cvtColor(colorFrame, frame, cv::COLOR_BGR2GRAY);

        //std::cout << "Mode: " << mode << std::endl;

    /*    //Write to file
        stamp.get(timeStamp);
        std::vector<float> truePath{(float)timeStamp, xPath[i],yPath[i],zPath[i],yawPath[i],pitchPath[i],rollPath[i]};
        databin_LOG.dump(truePath);
        imagebin.dump(timeStamp,frame);

*/
    //    std::vector<float> estimation{(float)timeStamp,t(0,0),t(1,0),t(2,0),yaw,(float)mode};
    //    databin_EST.dump(estimation);

//if(i>100) return 0;
        //std::string str = std::to_string(i);
        //std::cout << str << std::endl;
        //cv::putText(colorFrame,str,cv::Point(10,colorFrame.rows/2),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(118, 185, 0),2);


    }
    double timeStamp_end;
    stamp.get(timeStamp_end);
    double tot_time_s = (timeStamp_end-timeStamp_start)/1000;
    double fps = ((double)counter)/tot_time_s;
    std::cout << "Processed " << counter << "images in " << tot_time_s << " s. (" << fps << " fps)" << std::endl;
    return 1;
}
