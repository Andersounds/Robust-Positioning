#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
//#include <chrono> //For timestamps
#include "../src/logger.hpp"
#include "../src/timeStamp.cpp"


/*
  -p    path to directory where images are located (including trailing /)
  -b    base name if images defaulting to img_
*/
int parsePaths(std::vector<std::string>& paths,int argc, char** argv){
    std::string basePath = "";
    std::string baseName = "";
    for(int i=0;i<argc;i++){
        std::string flag = argv[i];
        std::string arg;
        if((i+1)<argc){
            arg = argv[i+1];
            if(flag == "-p"){
                //Make sure that dir ends with /
                if(std::regex_match(arg,std::regex("(.*)(/)"))){
                    basePath = arg;
                }else{
                    basePath = arg + "/";
                }
                std::cout << "Set relative path to dir to: " << basePath << std::endl;
            }else if(flag == "-b"){
                baseName = arg;
                std::cout << "Set base name to: " << baseName << std::endl;
            }
        }
    }
    if(argc == 1){
        std::cout << "Usage:" << std::endl;
        std::cout << "Flag Remark      Purpose" << std::endl;
        std::cout << "-p   optional    Relative path to directory containing the images to be renamed" << std::endl;
        std::cout << "-b   mandatory   Base name of images (eg img_)" << std::endl;
        return 0;
    }
    paths.clear();
    paths.push_back(basePath);
    paths.push_back(baseName);
    return 1;

}



int main(int argc, char** argv){

    robustPositioning::imageLogger imagebin;
    imagebin.init("","");
    std::vector<std::string> paths;
    if(!parsePaths(paths,argc,argv)){return 0;}
    imagebin.rename(cv::String(paths[0]),cv::String(paths[1]));


    return 1;
}
