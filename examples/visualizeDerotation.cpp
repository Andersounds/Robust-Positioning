#include <iostream>
#include <opencv2/opencv.hpp>

//#include "../src/simulatePose.cpp"


#include "../src/simulatePose.hpp"
#include "../src/homographyVO.hpp"
#include "../src/save2file.cpp"
#include "../src/opticalFlow.hpp"


#define PI 3.14159

/*
TODO: Standardizer order of roll/pitch?

*/

/* Draws arrows between the point correspondances and scale them
 *
 */
void drawArrows(cv::Mat& outputImg,std::vector<cv::Point2f> features1,std::vector<cv::Point2f> features2,float scale, cv::Point2f offset){
    std::vector<cv::Point2f>::iterator it1 = features1.begin();
    std::vector<cv::Point2f>::iterator it2 = features2.begin();
    while(it1!=features1.end()){

        cv::Point2f from = *it1 + offset;
        cv::Point2f to = (*it2 - *it1)*scale + *it1 + offset;
        cv::arrowedLine(outputImg,from,to,CV_RGB(200,50,0),2,cv::LINE_8,0,0.1);
        //cv::imshow("Chess board", outputImg);
        //cv::waitKey(0);
        it1++;
        it2++;
    }
}
/* Draws features
 *
 */
void drawFeatures(cv::Mat inputImg, std::vector<cv::Point2f> features, cv::Mat& outputImg, cv::Scalar color){
    std::vector<cv::KeyPoint> kp;
    cv::KeyPoint::convert(features,kp);
    cv::drawKeypoints(inputImg, kp, outputImg,color);

}
/* Draws a closed loop between all given points
 */
void drawLines(cv::Mat& img,std::vector<cv::Point2f> points,cv::Point2f offset){
    int size = points.size();
    int i = 0;
    while(i<(size-1)){
        cv::line(img,offset+points[i],offset+points[i+1],CV_RGB(255,0,0),2,cv::LINE_8,0);
        i++;
    }
    cv::line(img,offset+points[i],offset+points[0],CV_RGB(255,0,0),2,cv::LINE_8,0);//close loop
}
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
// Define sim chessboard parameters
    int boxWidth = 11;
    int rowsOfBoxes = 30;
    int colsOfBoxes = 60;
// Define file objects to output data into
    std::ofstream file_true;
    std::ofstream file_estimated;
// Init simulation environment
    simulatePose warper;
//    warper.setBaseScene(boxWidth,rowsOfBoxes,colsOfBoxes);
cv::Mat floor = cv::imread("/Users/Fredrik/Datasets/FloorTextures/test2.png",cv::IMREAD_REDUCED_COLOR_4);
    if(floor.empty()){std::cout << "Could not read file" << std::endl;return 0;}
    cv::Mat floor8U;
    cv::cvtColor(floor, floor8U, cv::COLOR_BGR2GRAY);
    warper.setBaseScene(floor);


//    cv::imshow("Chess board",floor);
//    cv::waitKey(0);

    warper.setParam("d",2);             //Camera in base pose is 1 m from scene
    warper.setParam("sceneWidth",4);    //Scenewidth is 2m
    warper.setParam("yaw",-3.1415/2);   // Camera is rotated 90 deg cc in base pose
    warper.setParam("x",2);         //Set global origin at (-x,-y) from basepose
    warper.setParam("y",0.7);
    warper.init(0);//Initialize with configuration 0

/*    float length = 200;
    std::vector<float> xPath = linSpace(2,2,length);
    std::vector<float> yPath = linSpace(0.7,0.7,length);
*/

    #include "../spike/testPath.cpp"
    float length = xPath.size();
    //xmax ca 1.9
    std::vector<float> yawPath =linSpace(0,6,length);
    std::vector<float> t_roll=linSpace(0,60,length);
    std::vector<float> t_pitch=linSpace(0,30,length);
    std::vector<float> rollPath = sinVector(t_roll,PI/16);
    std::vector<float> pitchPath = linSpace(0,0,length);//sinVector(t_pitch,3.14/12);


    float pathScale = 1;
    std::vector<float>::iterator xIt = xPath.begin();
    std::vector<float>::iterator yIt = yPath.begin();
    while(xIt != xPath.end()){
            *xIt*=pathScale;
            *yIt*=pathScale;
            xIt++;
            yIt++;
    }

    std::vector<float> zPath = linSpace(-0.8,-0.8,length);


    file_true.open("truePath.txt", std::ios::out | std::ios::app);
    std::vector<std::vector <float>> input{xPath,yPath,zPath,yawPath};
    build_path(input,file_true);
    file_true.close();
//Init flowField object with default settings
    of::opticalFlow FlowField(USE_KLT,3,150);
    FlowField.setDefaultSettings();
//Init odometry object
    cv::Mat_<float> K;
    warper.K.copyTo(K);//Can not assign with = as they then refer to same object. edit one edits the other
//Edit K matrix to work with image input of RoI-size
    K(0,2) = 75;
    K(1,2) = 75;
    cv::Mat_<float> T = warper.getZRot(-3.1415/2);//UAV frame is x forward, camera frame is -y forward
    vo::planarHomographyVO odometer(K,T,USE_AFFINETRANSFORM); //USE_HOMOGRAPHY
    //odometer.activateDerotation = false;//Deactivate derotation. is it needed even?
    //Initial values of UAV position
    cv::Mat_<float> R = warper.getZRot(0);
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = xPath[0];
    t(1,0) = yPath[0];
    t(2,0) = zPath[0];


//Set some parameters that are used trhoughout the program
//and initialize some variables on bottom of stack
    //std::vector<cv::Point2f> features; //The vector that keeps all the current features
    int cols = warper.baseScene.cols;//boxWidth*colsOfBoxes;
    int rows = warper.baseScene.rows;//boxWidth*rowsOfBoxes;
// TODO --- an easier way of setting the focusarea. should only define it once

    float x_ = (float) cols/2 - 75;
    float y_ = (float) rows/2 - 75;
    cv::Rect_<float> focusArea(x_,y_,150,150);
    cv::Point2f focusOffset(focusArea.x,focusArea.y);

    cv::Mat subPrevFrame;//For finding new corners
    cv::Mat colorFrame;//For illustration

//Go through whole path
    for(int i=0;i<(int)length;i++){
//Get new image
        std::vector<float> trueCoordinate{xPath[i],yPath[i],zPath[i]};
        std::vector<float> angles{yawPath[i],pitchPath[i],rollPath[i]};

        cv::Mat rawFrame = warper.uav2BasePose(angles,trueCoordinate);

        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);

//Process image...
        cv::Mat subFrame = frame(focusArea);

        //Get flow field
        std::vector<cv::Point2f> features;
        std::vector<cv::Point2f> updatedFeatures; //The new positions estimated from KLT
        FlowField.getFlow(subPrevFrame,subFrame,features,updatedFeatures);

        //Save flow field for plotting. De-rotation will scale it down so it wont be visible
        std::vector<cv::Point2f> p1_original = features;
        std::vector<cv::Point2f> p2_original = updatedFeatures;
        //VO estimation
        float height = abs(zPath[i])/(cos(pitchPath[i])*cos(rollPath[i]));//Används bara av odometer
        bool odometerSuccess = odometer.process(features,updatedFeatures,rollPath[i],pitchPath[i],height,t,R);





        float zAngle_sin = std::atan2(R(0,1),R(0,0));//tan with quadrant checking


        //Write to file
        if(zAngle_sin<0){zAngle_sin+=(2*PI);}
        std::vector<float> estimation{t(0,0),t(1,0),t(2,0),zAngle_sin};
        file_estimated.open("estPath.txt", std::ios::out | std::ios::app);
        build_row(estimation,file_estimated);
        file_estimated.close();


        //Illustrate
        //Simulation image
        rawFrame.copyTo(colorFrame);
        float scale = 10;
        drawArrows(colorFrame,p1_original,p2_original,scale,focusOffset);
        cv::rectangle(colorFrame,focusArea,CV_RGB(255,0,0),2,cv::LINE_8,0);
        cv::putText(colorFrame, "Original view and flow field",cv::Point(20, 20),cv::FONT_HERSHEY_DUPLEX,0.7,CV_RGB(118, 185, 0),2);
        //Straight image
        std::vector<cv::Point2f> roiCorners{cv::Point2f(0,0),cv::Point2f(150,0),cv::Point2f(150,150),cv::Point2f(0,150)};
        odometer.deRotateFlowField(roiCorners, rollPath[i],pitchPath[i]);
        //Set roll and pitch to zero. ORDER????
        angles[1] = 0;
        angles[2] = 0;
        cv::Mat straightFrame = warper.uav2BasePose(angles,trueCoordinate);
        odometer.deRotateFlowField(features, rollPath[i],pitchPath[i]);
        odometer.deRotateFlowField(updatedFeatures, rollPath[i],pitchPath[i]);
        drawArrows(straightFrame,features,updatedFeatures,scale,focusOffset);
        cv::putText(straightFrame, "No-pitch view with derotated flow field",cv::Point(20, 20),cv::FONT_HERSHEY_DUPLEX,0.7,CV_RGB(118, 185, 0),2);
        drawLines(straightFrame,roiCorners,focusOffset);
        //Create new image with both scenes
        ////cv::Mat both(cv::Size(colorFrame.rows*2,colorFrame.cols),colorFrame.type(),cv::Scalar(0));
        cv::Mat both(cv::Size(colorFrame.cols,colorFrame.rows*2),colorFrame.type(),cv::Scalar(0));
        cv::Mat roi1 = both(cv::Rect(0,0,colorFrame.cols,colorFrame.rows));
        colorFrame.copyTo(roi1);
        cv::Mat roi2 = both(cv::Rect(0,colorFrame.rows,colorFrame.cols,colorFrame.rows));
        straightFrame.copyTo(roi2);


        cv::imshow("Chess board", both);
        if(i==0){//If first lap
            cv::waitKey(0);
        }
        if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}
        //cv::waitKey(0);

        subFrame.copyTo(subPrevFrame);//Could just shift prevframe and subprevframe would be automatically shifted?
    }
    return 1;
}
