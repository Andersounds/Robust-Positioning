/*First version of Optical Flow-based Visual odometry
    Uses IMU measurements to derotate flow field
    Uses single point range sensor for scaling
*/
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep
#include "../spike/gimLeeStreamer.cpp"



int main(int argc, char** argv){
// Init streamer
    cv::Mat frame;
    dataSetStreamer dss("/Users/Fredrik/Datasets/GimLee/3LoopsDown/imagesUndistort_3loopsDown/",
        "/Users/Fredrik/Datasets/GimLee/3LoopsDown/imagesUndistort_3loopsDown.txt",
        "/Users/Fredrik/Datasets/GimLee/3LoopsDown/imu_3loopsDown.txt",
        "/Users/Fredrik/Datasets/GimLee/3LoopsDown/vicon_3loopsDown.txt");

        float roll;
        float pitch;
        float height;
        float viconX;
        float viconY;
        float viconZ;
    while(1){
        std::vector<float> imuData;
        std::vector<float> viconData;
        std::string framePath;
        std::vector<int> dataStatus = dss.getData(imuData,viconData,framePath);
        //Process IMU data..
        if(dataStatus[1]){
            roll = viconData[3];//right values? right signs?
            pitch = viconData[4];
            height = viconData[0];
            viconX = viconData[0];
            viconY = viconData[1];
            viconZ = viconData[2];
            //std::cout << "Data: " << viconData[0]<< ", "<<viconData[1] << ", "<<viconData[2] << std::endl;
        }

        // Process image if available..
        if(dataStatus[2]==0){continue;}

        //Image process..
        frame = cv::imread(framePath,cv::IMREAD_GRAYSCALE);//Get new image
        if(frame.empty()){std::cout << "Image path not valid: " << framePath << std::endl;return 1;}


        cv::imshow("Showit",frame);
        cv::waitKey(0);
        if(cv::waitKey(1) == 27 ) break;                         // stop capturing by pressing ESC

    }
    return 1;
}
