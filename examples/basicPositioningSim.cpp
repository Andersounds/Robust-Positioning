#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "../src/vopos.hpp"
#include "../src/simulatePose.hpp"
#include "../src/save2file.cpp"
#include "../src/settingsParser.cpp"

#define PI 3.1416

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


int main(int argc, char** argv){
    //Initialize settings
    set::settings S(argc,argv);
    //Initialize simulation
    // Define sim chessboard parameters
    int boxWidth = 11;
    int rowsOfBoxes = 30;
    int colsOfBoxes = 60;
// Define file objects to output data into
    std::ofstream file_true;
    std::ofstream file_estimated;
// Init simulation environment
    simulatePose warper;
    cv::Mat floor = cv::imread("spike/test2.png",cv::IMREAD_REDUCED_COLOR_4);
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


    //Initialize positioning object
    int maxIdAruco = 50;
    std::string anchorPath = "anchors.txt";
    int flowGrid = 4;
    cv::Rect2f roiSize(245,125,150,150);
    cv::Mat_<float> K;
    warper.K.copyTo(K);//Can not assign with = as they then refer to same object. edit one edits the other
    cv::Mat_<float> T = warper.getZRot(-PI/2);//UAV frame is x forward, camera frame is -y forward
    pos::positioning P(pos::OF_MODE_KLT,
                        pos::VO_MODE_AFFINE,
                        cv::aruco::DICT_4X4_50,
                        maxIdAruco,anchorPath,flowGrid,roiSize,K,T);
    //Init values of position and yaw
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = xPath[0];
    t(1,0) = yPath[0];
    t(2,0) = zPath[0];
    float yaw = yawPath[0];





//Go through whole path
    for(int i=0;i<(int)length;i++){
//Get new image
        std::vector<float> trueCoordinate{xPath[i],yPath[i],zPath[i]};
        std::vector<float> angles{yawPath[i],pitchPath[i],rollPath[i]};
        cv::Mat rawFrame = warper.uav2BasePose(angles,trueCoordinate);
        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);

        float roll = rollPath[i];
        float pitch = pitchPath[i];
        float height = -zPath[i];//THIS SHOULD BE "SIMULATED" FROM DATA not exactly height
        int mode = P.processAndIllustrate(pos::MODE_AZIPE_AND_VO,frame,rawFrame,pos::ILLUSTRATE_ALL,height,roll,pitch,yaw,t);
        std::cout << "Mode: " << mode << std::endl;

        //Write to file
        std::vector<float> estimation{t(0,0),t(1,0),t(2,0),yaw,(float)mode};
        file_estimated.open("estPath.txt", std::ios::out | std::ios::app);
        build_row(estimation,file_estimated);
        file_estimated.close();


        std::string str = std::to_string(i);
        std::cout << str << std::endl;
        //cv::putText(rawFrame,str,cv::Point(10,rawFrame.rows/2),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(118, 185, 0),2);
        //cv::imshow("showit",rawFrame);
        //cv::waitKey(0);
        if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}

    }
    return 1;
}
