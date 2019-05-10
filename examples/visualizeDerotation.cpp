#include <iostream>
#include <opencv2/opencv.hpp>

//#include "../src/simulatePose.cpp"

#include "../src/KLTFlowField.cpp"
#include "../src/simulatePose.hpp"
#include "../src/homographyVO.hpp"
#include "../src/save2file.cpp"


/*
This code is a basic stream of sim chessboard warped images that
also extracts and draws the flowfield
*/

/*
Issue:
VO algorithm performs MUCH better when tracked features are simply thrown away every frame.
    Is there a bug? can I handle it somehow?

*/


/* Draws arrows between the point correspondances and scale them
 *
 */
void drawArrows(cv::Mat img,cv::Mat& outputImg,std::vector<cv::Point2f> features1,std::vector<cv::Point2f> features2,float scale, cv::Point2f offset){
    std::vector<cv::Point2f>::iterator it1 = features1.begin();
    std::vector<cv::Point2f>::iterator it2 = features2.begin();
    while(it1!=features1.end()){
        cv::Point2f from = *it1 + offset;
        cv::Point2f to = (*it2 - *it1)*scale + *it1 + offset;
        cv::arrowedLine(outputImg,from,to,CV_RGB(200,50,0),2,cv::LINE_8,0,0.1);
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
//std::vector<float>



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

    warper.setBaseScene(boxWidth,rowsOfBoxes,colsOfBoxes);
//cv::Mat floor = cv::imread("/Users/Fredrik/Datasets/FloorTextures/light_stone_wall.jpg",cv::IMREAD_COLOR);
//cv::Mat floor = cv::imread("/Users/Fredrik/Datasets/FloorTextures/test1.png",cv::IMREAD_COLOR);
/*cv::Mat floor = cv::imread("/Users/Fredrik/Datasets/FloorTextures/test2.png",cv::IMREAD_REDUCED_COLOR_4);
    cv::Mat floor8U;
    cv::cvtColor(floor, floor8U, cv::COLOR_BGR2GRAY);
    warper.setBaseScene(floor);
*/
/*
    cv::imshow("Chess board",floor);
    cv::waitKey(0);
*/

    warper.setParam("d",2);             //Camera in base pose is 1 m from scene
    warper.setParam("sceneWidth",4);    //Scenewidth is 2m
    warper.setParam("yaw",-3.1415/2);   // Camera is rotated 90 deg cc in base pose

    //warper.setParam("x",2);         //Set global origin at (-x,-y) from basepose
//    warper.setParam("y",1);

    warper.init(0);//Initialize with configuration 0


//Create path of camera and save to output file

    //Start out aligned with x-y of global coordinate system

    float length = 200;
    std::vector<float> xPath = linSpace(0,0,length);
    std::vector<float> yPath = linSpace(0,0,length);
    std::vector<float> rollPath = linSpace(3.14/7,3.14/7,length);


    //#include "testPath.cpp"
    //xmax ca 1.9
    std::vector<float> yawPath =linSpace(0,6,length);;
    float pathScale = 1.8;
    std::vector<float>::iterator xIt = xPath.begin();
    std::vector<float>::iterator yIt = yPath.begin();

    while(xIt != xPath.end()){
            *xIt*=pathScale;
            *yIt*=pathScale;

            xIt++;
            yIt++;
    }
   //float length = xPath.size();



    std::vector<float> zPath = linSpace(-0.5,-0.5,length);


    file_true.open("truePath.txt", std::ios::out | std::ios::app);
    std::vector<std::vector <float>> input{xPath,yPath,zPath,yawPath};
    build_path(input,file_true);
    file_true.close();

//Init flowField object with default settings
    featureVO FlowField;
    FlowField.setFeatureDefaultSettings();
    FlowField.setKLTDefaultSettings();
//Init odometry object
    cv::Mat_<float> K;
    warper.K.copyTo(K);//Can not assign with = as they then refer to same object. edit one edits the other
//Edit K matrix to work with image input of RoI-size
    K(0,2) = 75;
    K(1,2) = 75;
    cv::Mat_<float> T = warper.getZRot(-3.1415/2);//UAV frame is x forward, camera frame is -y forward
    vo::planarHomographyVO odometer(K,T);
    //Initial values of UAV position
    //cv::Mat_<float> R = warper.getZRot(yawPath[0]);
    cv::Mat_<float> R = warper.getZRot(0);
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = xPath[0];//-1.299;
    t(1,0) = yPath[0];//-3.398;
    t(2,0) = zPath[0];// 1.664;


//Set some parameters that are used trhoughout the program
//and initialize some variables on bottom of stack
    std::vector<cv::Point2f> features; //The vector that keeps all the current features
    int noOfCorners = 30;
    int noOfTracked = 0;//Init
    float maskDir = -1;//to choose mask. init value -1 for whole scene
    int cols = warper.baseScene.cols;//boxWidth*colsOfBoxes;
    int rows = warper.baseScene.rows;//boxWidth*rowsOfBoxes;
    cv::Rect_<float> focusArea = FlowField.getFocusArea(cols,rows,150,150);//(cols,rows,size,size)
    //focusArea.x+=100;focusArea.y+=40;
    cv::Point2f focusOffset(focusArea.x,focusArea.y);
    cv::Mat subPrevFrame;//For finding new corners


cv::Mat colorFrame;//For illustration
//Go through whole path
    for(int i=0;i<(int)length;i++){
//Get new image
        float pitch = 0;
        float height = 0.5;//Används bara av odometer
        std::vector<float> trueCoordinate{xPath[i],yPath[i],zPath[i]};

        //std::cout << "Coordinates true: "<< trueCoordinate[0] <<", " << trueCoordinate[1] << ", "<< trueCoordinate[2] << std::endl;
        //std::cout << "Coordinates est init: "<< t << std::endl;
        std::vector<float> angles{yawPath[i],pitch,rollPath[i]};
//        cv::Mat rawFrame = warper.getWarpedImage(angles,trueCoordinate);
        cv::Mat rawFrame = warper.uav2BasePose(angles,trueCoordinate);
        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);
//Process image...
        cv::Mat subFrame = frame(focusArea);
        if(noOfTracked<=28){//if(noOfTracked<=noOfCorners*0.7){
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

        std::vector<cv::Point2f> p1_original;
        std::vector<cv::Point2f> p2_original;
        for(int i=0;i<noOfTracked;i++){
            p1_original.push_back(activeFeatures1[i]);
            p2_original.push_back(activeFeatures2[i]);
        }

        bool odometerSuccess = odometer.process(activeFeatures1,activeFeatures2,rollPath[i],pitch,height,t,R);
        float z_sin = R(0,1);
        float zAngle_sin = std::atan2(R(0,1),R(0,0));//tan with quadrant checking


        //Write to file
        std::vector<float> estimation{t(0,0),t(1,0),t(2,0),zAngle_sin};
        file_estimated.open("estPath.txt", std::ios::out | std::ios::app);
        build_row(estimation,file_estimated);
        file_estimated.close();


        //Illustrate
        //Simulation image
        rawFrame.copyTo(colorFrame);
        //cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
        float scale = 10;
        drawArrows(colorFrame,colorFrame,p1_original,p2_original,scale,focusOffset);
        //drawFeatures(colorFrame, features, colorFrame, CV_RGB(2,200,0));
        cv::rectangle(colorFrame,focusArea,CV_RGB(255,0,0),2,cv::LINE_8,0);
        cv::putText(colorFrame, "Original view and flow field",cv::Point(20, 20),cv::FONT_HERSHEY_DUPLEX,0.7,CV_RGB(118, 185, 0),2);

        //Straight image
        std::vector<cv::Point2f> roiCorners{cv::Point2f(0,0),cv::Point2f(150,0),cv::Point2f(150,150),cv::Point2f(0,150)};
        odometer.deRotateFlowField(roiCorners, angles[2]);
        angles[2]=0;
        cv::Mat straightFrame = warper.uav2BasePose(angles,trueCoordinate);
        drawArrows(straightFrame,straightFrame,activeFeatures1,activeFeatures2,scale,focusOffset);
        cv::putText(straightFrame, "No-pitch view with derotated flow field",cv::Point(20, 20),cv::FONT_HERSHEY_DUPLEX,0.7,CV_RGB(118, 185, 0),2);
        drawLines(straightFrame,roiCorners,focusOffset);

        cv::Mat both(cv::Size(colorFrame.rows*2,colorFrame.cols),colorFrame.type(),cv::Scalar(0));
        cv::Mat roi1 = both(cv::Rect(0,0,colorFrame.cols,colorFrame.rows));
        colorFrame.copyTo(roi1);
        cv::Mat roi2 = both(cv::Rect(0,colorFrame.rows,colorFrame.cols,colorFrame.rows));
        straightFrame.copyTo(roi2);


        cv::imshow("Chess board", both);
        if(i==0){//If first lap
            cv::waitKey(0);
        }
        //if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}
        cv::waitKey(0);




        //Time-shift frames and features
        //features = activeFeatures2;

        //features.clear(); // These two lines are here because the algorithm performs much better when features are just thorwn away and found again
        //noOfTracked = 0;  // WHen timing manually it is actually  a tiny bit faster. probably because the copying takes time.

        subFrame.copyTo(subPrevFrame);//Could just shift prevframe and subprevframe would be automatically shifted?
    }
    return 1;
}
