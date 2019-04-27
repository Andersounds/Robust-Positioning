#include <iostream>
#include <opencv2/opencv.hpp>

//#include "../src/simulatePose.cpp"

#include "../src/KLTFlowField.cpp"
#include "../src/simulatePose.hpp"
#include "../src/homographyVO.hpp"
/*
This code is a basic stream of sim chessboard warped images that
also extracts and draws the flowfield
*/


/* Draws arrows between the point correspondances and scale them
 *
 */
void drawArrows(cv::Mat img,cv::Mat& outputImg,std::vector<cv::Point2f> features1,std::vector<cv::Point2f> features2,float scale){
    std::vector<cv::Point2f>::iterator it1 = features1.begin();
    std::vector<cv::Point2f>::iterator it2 = features2.begin();
    while(it1!=features1.end()){
        cv::Point2f from = *it1;
        cv::Point2f to = (*it2 - from)*scale + from;
        cv::arrowedLine(outputImg,from,to,CV_RGB(0,255,100),1,cv::LINE_8,0,0.1);
        it1++;
        it2++;
    }
}
/*Returns a linspace sequence starting from start with length no of steps of size step
 *
 */
std::vector<float> getPath(float start,float step,float length){
    std::vector<float> path;
    for(float i=0;i<length;i++){
        path.push_back(step*i+start);
    }
    return path;
}




int main(void){
// Define sim chessboard parameters
int boxWidth = 11;
int rowsOfBoxes = 30;
int colsOfBoxes = 60;
/*// Define K, T
    cv::Mat_<float> K = cv::Mat_<float>::zeros(3,3);
    float fx = (float) boxWidth*colsOfBoxes;//260;//1;
    float fy = fx;//260;//1;
    std::cout << "NORM K"<< std::endl;
    float cx = (float) boxWidth*colsOfBoxes/2;//376;//Change pixel center according to focus area
    float cy = (float) boxWidth*rowsOfBoxes/2;//240;
    K(0,0) = fx;
    K(1,1) = fy;
    K(2,2) = 1;
    K(0,2) = cx;
    K(1,2) = cy;
    cv::Mat_<float> T = cv::Mat_<float>::eye(3,3);
    std::cout << "K1: " << K << std::endl;
    */
// Init simulation environment
    simulatePose warper;
    warper.setBaseScene(boxWidth,rowsOfBoxes,colsOfBoxes);
    std::vector<float> angles{3.1415,0,0};
    std::vector<float> coordinates{0,0,1};
    warper.setBasePose(angles,coordinates);
    warper.setParam("sceneWidth",1);
//Create path of camera
    float length = 100;
    std::vector<float> xPath = getPath(0,0.005,length);
    std::vector<float> yPath = getPath(0,0,length);
    std::vector<float> zPath = getPath(0.5,0,length);
    std::vector<float> phi = getPath(0,0.04,length);//Crashes if this is zero??? or smthing
//Init flowField object
    //settings for corner finder
    double qualityLevel= 0.5;
    double minDistance = 20;
    cv::Mat mask;
    int blockSize = 3;
    double k = 0.04;
    bool useHarris = false;
    //settings for KLT tracker
    int windowSize = 51;
    int maxLevel = 2;
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.01);
    int flags = 0;//No flag. Use normal L2 norm error
    //Init object and set settings
    featureVO FlowField;
    FlowField.setFeatureSettings(qualityLevel,minDistance,blockSize,useHarris,k);
    FlowField.setKLTSettings(windowSize,maxLevel,termcrit,flags);
//Init odometry object
    cv::Mat_<float> K = warper.K;
    cv::Mat_<float> T = cv::Mat_<float>::zeros(3,3);//OBSOBSOBSOSBOSBSOBSOSBOSBSOSBOSBOBS
    std::cout<< "OBSOBSOBSOSBOSBSOBSOSBOSBSOSBOSBOBS" << std::endl;
    vo::planarHomographyVO odometer(K,T);
//Set some parameters that are used trhoughout the program
//and initialize some variables on bottom of stack
    std::vector<cv::Point2f> features; //The vector that keeps all the current features
    int noOfCorners = 500;
    int noOfTracked = 0;//Init
    float maskDir = -1;//to choose mask. init value -1 for whole scene
    int cols = boxWidth*colsOfBoxes;
    int rows = boxWidth*rowsOfBoxes;
    cv::Rect_<float> focusArea = FlowField.getFocusArea(cols,rows,100,100);//(cols,rows,size,size)
    cv::Point2f focusOffset(focusArea.x,focusArea.y);
    cv::Mat subPrevFrame;//For finding new corners
//Initial values of UAV position
    cv::Mat_<float> R = cv::Mat_<float>::eye(3,3);
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = 0;//-1.299;
    t(1,0) = 0;//-3.398;
    t(2,0) = 0;// 1.664;


//Go through whole path
    for(int i=0;i<(int)length;i++){
//Get new image
        float roll = 0;//3.1415/4;
        float pitch = 0.03;
        float height = 1;
        std::vector<float> trueCoordinate{xPath[i],yPath[i],zPath[i]};
        std::vector<float> angles{roll,pitch,phi[i]};
        cv::Mat rawFrame = warper.getWarpedImage(angles,trueCoordinate);
        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);
//Process image...
        cv::Mat subFrame = frame(focusArea);
        if(noOfTracked<=noOfCorners*0.7){
            int noOfNew = noOfCorners-noOfTracked;
            std::vector<cv::Point2f> newFeatures;
            cv::Point2f dir(0,0);
            cv::Rect_<float> dirMask = FlowField.getDirMask(focusArea,dir);
            //CHOOSE SUBSET HERE WITH DIRMASK
            //subPrevFrame(dirMask) put this in getCorners
            FlowField.getCorners(subPrevFrame, newFeatures,noOfNew);//,dirMask);
            FlowField.conCat(features,newFeatures);//Add the new features to old vector
        }
        std::vector<cv::Point2f> updatedFeatures; //The new positions estimated from KLT
        std::vector<uchar> status;
        std::vector<float> errors;
        FlowField.trackKLT(subPrevFrame,subFrame,features,updatedFeatures,status,errors);
        //Get just the correspondance pairs that are active. AND SHIFT TO IMAGE COORD
        std::vector<cv::Point2f> activeFeatures1,activeFeatures2;
        noOfTracked = FlowField.extractActiveFeatures(features,status,activeFeatures1,focusOffset);
        FlowField.extractActiveFeatures(updatedFeatures,status,activeFeatures2,focusOffset);
    //At this point the flow field is retrieved as point correspondeances activeFeatures1, activeFeatures2 given in whole image coordinate system


        bool odometerSuccess = odometer.process(activeFeatures1,activeFeatures2,roll,pitch,height,t,R);
        std::cout << t(0,0)<< ", "<<t(1,0)<< ", "<<t(2,0)<<"; ";
        float z_sin = R(0,1);
        float zAngle_sin = std::asin(R(0,1));
        //std::cout <<z_sin << ", "<<zAngle_sin;
        std::cout<< std::endl;




        //Illustrate
        cv::Mat colorFrame;//For illustration
        rawFrame.copyTo(colorFrame);
        //cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
        float scale = 3;
        drawArrows(colorFrame,colorFrame,activeFeatures1,activeFeatures2,scale);
        cv::rectangle(colorFrame,focusArea,CV_RGB(255,0,0),2,cv::LINE_8,0);
        cv::imshow("Chess board", colorFrame);
        cv::waitKey(0);
        //Time-shift frames and features
        features = activeFeatures2;
        subFrame.copyTo(subPrevFrame);//Could just shift prevframe and subprevframe would be automatically shifted?
    }
    return 1;
}
