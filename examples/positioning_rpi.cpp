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


    //Initialize settings
    set::settings S(argc,argv);
    if(!S.success()){return 0;}


    //Initialize video stream
    std::string basePath = S.data.imageStreamBasePath;
    std::string imageInfo = S.data.imageStreamInfoFile;
//    robustPositioning::Streamer VStreamer(robustPositioning::MODE_RPI_CAM);
    robustPositioning::Streamer VStreamer(basePath,imageInfo);
    cv::Mat frame, colorFrame;
    //Initialize data stream
    std::string dataFile = S.data.dataStreamFile;
    int skipLines = 1;
    robustPositioning::dataStreamer getData(dataFile,skipLines);
    std::vector<float> data;


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
                        VisualOdometryMode,
                        cv::aruco::DICT_4X4_50,
                        maxIdAruco,anchorPath,flowGrid,roiSize,K,T);
    P.setDistortCoefficents(S.data.dist_k1,S.data.dist_k2,S.data.dist_k3);//Set distortion coefficients
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
    double avgProcessTime = 0; //Avarage process time in ms
    stamp.get(timeStamp_start);
    int estimationMode = S.data.MODE_Positioning;
    while(getData.get(data)){
        timeStamp_data = data[0];
        float height = data[1];//This is used as a subst as actual height is not in dataset
        float pitch = data[2];
        float roll = data[3];

//Get new image
        if(VStreamer.peek()<=timeStamp_data){
            counter++;
            VStreamer.getImage(frame);
            if(frame.empty()){std::cout << "Video stream done."<< std::endl; break;}
            double from;
            double to;
            stamp.get(from);
            int mode = P.process(estimationMode,frame,height, roll, pitch, yaw, t);
            stamp.get(to);
            avgProcessTime += (to-from);
            std::cout << "Lap " << counter << std::endl;
        }
    }
    double timeStamp_end;
    stamp.get(timeStamp_end);
    double tot_time_s = (timeStamp_end-timeStamp_start)/1000;
    double fps = ((double)counter)/tot_time_s;
    std::cout << "Processed " << counter << "images in " << tot_time_s << " s. (" << fps << " fps)" << std::endl;
    avgProcessTime/=((double)counter);
    std::cout << "Avarage processing time of VOPOS: " << avgProcessTime << " ms." << std::endl;
    return 1;
}
