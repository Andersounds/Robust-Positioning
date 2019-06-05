#include <iostream>
#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include "../src/simulatePose.hpp"
#include "../src/angulation.hpp"
#include "../src/save2file.cpp"
#include <opencv2/aruco.hpp>

#define PI 3.1416
/*
This code is a basic stream of sim warped images
*/


/*Returns a linspace sequence starting from start with length no of steps of size step
 *
 */
std::vector<float> linSpace(float start,float stop,float length){
    std::vector<float> path;
    float step = (stop-start)/length;
    for(float i=0;i<length;i++){
        path.push_back(step*i+start);
    }
    return path;
}
std::vector<float> sinVector(std::vector<float> t,float scale){
    std::vector<float> sin_;
    for(float i:t){
        sin_.push_back(std::sin(i)*scale);
    }
    return sin_;
}


int main(void){


    int maxId = 50;
    std::string anchorPath = "anchors.txt";
// Define sim chessboard parameters
    int boxWidth = 11;
    int rowsOfBoxes = 30;
    int colsOfBoxes = 60;
// Define file objects to output data into
    std::ofstream file_true;
    std::ofstream file_estimated;

// Init simulation environment
    simulatePose warper;
    cv::Mat floor = cv::imread("/Users/Fredrik/Datasets/FloorTextures/test2.png",cv::IMREAD_REDUCED_COLOR_4);
    cv::Mat floor8U;
    cv::cvtColor(floor, floor8U, cv::COLOR_BGR2GRAY);
    warper.setBaseScene(floor);


    warper.setParam("d",2);             //Camera in base pose is 1 m from scene
    warper.setParam("sceneWidth",4);    //Scenewidth is 2m
    warper.setParam("yaw",-3.1415/2);   // Camera is rotated 90 deg cc in base pose
    warper.setParam("x",2);         //Set global origin at (-x,-y) from basepose
    float y_ = ((float)floor.rows)*2/((float)floor.cols);//Calculate y coordinate in base pose [m]
    warper.setParam("y",y_);
    warper.init(0);//Initialize with configuration 0

    //Create path of camera and save to output file
    #include "../spike/testPath.cpp"
    //std::vector<float> xPath = linSpace(2,2,300);
    //std::vector<float> yPath = linSpace(1,1,300);
    float length = xPath.size();
    std::vector<float> t0 = linSpace(0,6*PI,length);
    std::vector<float> yawPath =sinVector(t0,1.1);
    std::vector<float> t1 = linSpace(0,5*PI,length);
    std::vector<float> rollPath = sinVector(t1,0.08);
    std::vector<float> t2 = linSpace(0,8*PI,length);
    std::vector<float> pitchPath = sinVector(t2,0.12);
    std::vector<float> zPath = linSpace(-0.9,-0.9,length);
    float pathScale = 1.8;
    std::vector<float>::iterator xIt = xPath.begin();
    std::vector<float>::iterator yIt = yPath.begin();
    std::vector<float>::iterator yawIt = yawPath.begin();

    while(xIt != xPath.end()){
            *xIt*=pathScale;
            *yIt*=pathScale;
            *yawIt += 1.15;
            xIt++;
            yIt++;
            yawIt++;
    }



    file_true.open("truePath.txt", std::ios::out | std::ios::app);
    std::vector<std::vector <float>> input{xPath,yPath,zPath,yawPath};
    build_path(input,file_true);
    file_true.close();






//Init odometry object
    cv::Mat_<float> K;
    warper.K.copyTo(K);//Can not assign with = as they then refer to same object. edit one edits the other
    cv::Mat_<float> T = warper.getZRot(-PI/2);//UAV frame is x forward, camera frame is -y forward


    //Initial values of UAV position
    //cv::Mat_<float> R = warper.getZRot(yawPath[0]);
    cv::Mat_<float> R = warper.getZRot(0);
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = xPath[0];//-1.299;
    t(1,0) = yPath[0];//-3.398;
    t(2,0) = zPath[0];// 1.664;



    //angulation init
    ang::angulation azipe(maxId,anchorPath);
    azipe.setKmat(K);
    azipe.setTmat(T);
    //Aruco init
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//Go through whole path
    for(int i=0;i<(int)length;i++){
//Get new image
        std::vector<float> trueCoordinate{xPath[i],yPath[i],zPath[i]};
        float pitch = 0;
        std::vector<float> angles{yawPath[i],pitchPath[i],rollPath[i]};
        cv::Mat rawFrame = warper.uav2BasePose(angles,trueCoordinate);
        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);
        cv::Mat colorFrame;//For illustration
        rawFrame.copyTo(colorFrame);

//Aruco detect and draw
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
//Give some offset her. could be random. Just to ot give perfect initial guess
/*        t(0,0) = xPath[i];
        t(1,0) = yPath[i];
        t(2,0) = zPath[i];
*/
        float zAngle_sin;
        bool success = azipe.calculate(corners,ids,t,zAngle_sin,rollPath[i],pitchPath[i]);
        //std::cout << "Est: " << zAngle_sin<<", True: " << yawPath[i] << std::endl;

        cv::aruco::drawDetectedMarkers(colorFrame, corners, ids, CV_RGB(0,250,0));



        //float zAngle_sin = 1;
        //Write to file
        std::vector<float> estimation{t(0,0),t(1,0),t(2,0),zAngle_sin};
        file_estimated.open("estPath.txt", std::ios::out | std::ios::app);
        build_row(estimation,file_estimated);
        file_estimated.close();


        //Illustrate
        //Simulation image


        cv::imshow("Chess board", colorFrame);
        if(i==0){//If first lap
            cv::waitKey(0);
        }
        if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}
        //cv::waitKey(0);

    }
    return 1;
}
