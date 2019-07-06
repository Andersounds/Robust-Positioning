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



int main(int argc, char** argv){
timestamp::timeStamp_ms stamp;
//Can these two rows be written as stamp.get(double timeStamp); is timeStamp available in this scope then?
double timeStamp;
stamp.get(timeStamp);

/*
//Initialize imagebin. It automatically creates a directory 'images' in the given path
log::imageLogger imagebin;
imagebin.init("","5_jul");

log::dataLogger databin_LOG;
//if(!databin_EST.init("estPath.csv",std::vector<std::string>{"Timestamp [ms]","x[m]","y[m]","z[m]","yaw[rad]","mode"})) return 0;
if(!databin_LOG.init("5_jul/truePath.csv",std::vector<std::string>{"Timestamp [ms]","x [m]","y [m]","z [m]","yaw [rad]","pitch [rad]","roll [rad]"})) return 0;


*/

    //Initialize settings
    set::settings S(argc,argv);
    if(!S.success()){return 0;}


    //Initialize video stream
    std::string basePath = "Generated-dataSets/5_jul/";
    std::string imageInfo = "data.csv";
    robustPositioning::Streamer VStreamer(basePath,imageInfo);
    cv::Mat frame, colorFrame;
    //Initialize data stream
    std::string dataFile = "Generated-dataSets/5_jul/truePath.csv";
    int skipLines = 1;
    robustPositioning::dataStreamer getData(dataFile,skipLines);
    std::vector<float> data;


    //Initialize positioning object
    int maxIdAruco = 50;
    std::string anchorPath = S.data.anchorPath;//"anchors.txt";
    int flowGrid = 4;
    cv::Rect2f roiSize(245,125,150,150);
    cv::Mat_<float> K = S.data.K;

    cv::Mat_<float> T = S.data.T;//warper.getZRot(-PI/2);//UAV frame is x forward, camera frame is -y forward
    pos::positioning P(pos::OF_MODE_KLT,
                        pos::VO_MODE_AFFINE,
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
    while(getData.get(data)){
        timeStamp_data = data[0];
        float height = -data[3];//This is used as a subst as actual height is not in dataset
        float pitch = data[5];
        float roll = data[6];

//Get new image
        if(VStreamer.peek()<=timeStamp_data){
            VStreamer.getImage(frame);
            if(frame.empty()){std::cout << "Video stream done."<< std::endl; return 0;}
            cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
            int mode = P.processAndIllustrate(pos::MODE_AZIPE_AND_VO,frame,colorFrame,pos::ILLUSTRATE_ALL,height,roll,pitch,yaw,t);
            cv::imshow("showit",colorFrame);
            //cv::waitKey(0);
            if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}
        }

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
    return 1;
}
