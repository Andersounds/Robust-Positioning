#include <iostream>
#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include "../src/simulatePose.hpp"
#include "../src/save2file.cpp"
#include <opencv2/aruco.hpp>


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


//This function takes a line and parses it into a vector<string> using "," as deliminator and disregarding LEADING whitespaces
std::vector<std::string> parse(std::string line){
    char delim = ',';
    std::vector<std::string> parsed;
    std::string::iterator it = line.begin();
    while(it!=line.end()){
        std::string word;
        while(it!=line.end()){
            if(isspace(*it)){it++;}//Remove leading whitespaces
            else{break;}
        }
        while(it!=line.end()){
            if(*it != delim){
                word+=*it;//Append the char to the temporary string
                it++;
            }//Go through until deliminator
            else{it++;
                break;}
        }
        parsed.push_back(word);//Push back the parsed word onto the return vector
    }
    return parsed;
}


int readToDataBase(std::string path,std::vector<int>& IDs, std::vector<cv::Mat_<float>>& coordinates){
    std::string line;
    std::string delim = ",";
    std::ifstream file;
    file.open(path);
    if(file.is_open()){
         while(getline(file,line)){
            std::vector<std::string> parsed = parse(line);
            if(parsed.size()==4){//Disregard any lines that are not exactly 4 elements long
                int id      =   std::stoi(parsed[0]);
                float x     =   std::stof(parsed[1]);
                float y     =   std::stof(parsed[2]);
                float z     =   std::stof(parsed[3]);
                cv::Mat_<float> coord = cv::Mat_<float>::zeros(3,1);
                coord(0,0) = x;
                coord(0,1) = y;
                coord(0,2) = z;
                IDs.push_back(id);
                coordinates.push_back(coord);
            }
         }
    }else{return 0;}
    file.close();
    return 1;
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
    cv::Mat floor = cv::imread("/Users/Fredrik/Datasets/FloorTextures/test2.png",cv::IMREAD_REDUCED_COLOR_4);
    cv::Mat floor8U;
    cv::cvtColor(floor, floor8U, cv::COLOR_BGR2GRAY);
    warper.setBaseScene(floor);


    warper.setParam("d",2);             //Camera in base pose is 1 m from scene
    warper.setParam("sceneWidth",4);    //Scenewidth is 2m
    warper.setParam("yaw",-3.1415/2);   // Camera is rotated 90 deg cc in base pose
    warper.setParam("x",2);         //Set global origin at (-x,-y) from basepose
    warper.setParam("y",1);
    warper.init(0);//Initialize with configuration 0

    //Create path of camera and save to output file
    #include "../spike/testPath.cpp"
    float length = xPath.size();
    std::vector<float> yawPath =linSpace(0,6,length);
    std::vector<float> rollPath = linSpace(0,0,length);
    std::vector<float> zPath = linSpace(-0.8,-0.8,length);
    float pathScale = 1.8;
    std::vector<float>::iterator xIt = xPath.begin();
    std::vector<float>::iterator yIt = yPath.begin();
    while(xIt != xPath.end()){
            *xIt*=pathScale;
            *yIt*=pathScale;
            xIt++;
            yIt++;
    }



    file_true.open("truePath.txt", std::ios::out | std::ios::app);
    std::vector<std::vector <float>> input{xPath,yPath,zPath,yawPath};
    build_path(input,file_true);
    file_true.close();


//Init odometry object
    cv::Mat_<float> K;
    warper.K.copyTo(K);//Can not assign with = as they then refer to same object. edit one edits the other
    cv::Mat_<float> T = warper.getZRot(-3.1415/2);//UAV frame is x forward, camera frame is -y forward


    //Initial values of UAV position
    //cv::Mat_<float> R = warper.getZRot(yawPath[0]);
    cv::Mat_<float> R = warper.getZRot(0);
    cv::Mat_<float> t = cv::Mat_<float>::zeros(3,1);
    t(0,0) = xPath[0];//-1.299;
    t(1,0) = yPath[0];//-3.398;
    t(2,0) = zPath[0];// 1.664;


cv::Mat colorFrame;//For illustration
//angulation init
std::vector<int> IDs;
std::vector<cv::Mat_<float>> coordinates;
std::string path = "anchors.txt";
if(!readToDataBase(path,IDs,coordinates)){std::cout << "Could not read anchor database from file"<<std::endl;}
//Aruco init
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//Go through whole path
    for(int i=0;i<(int)length;i++){
//Get new image
        std::vector<float> trueCoordinate{xPath[i],yPath[i],zPath[i]};
        float pitch = 0;
        std::vector<float> angles{yawPath[i],pitch,rollPath[i]};
        cv::Mat rawFrame = warper.uav2BasePose(angles,trueCoordinate);
        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);
        rawFrame.copyTo(colorFrame);

//Aruco detect and draw
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
        std::cout << "size: " << corners.size() << std::endl;
        cv::aruco::drawDetectedMarkers(colorFrame, corners, ids, CV_RGB(0,250,0));



        float zAngle_sin = 1;
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
