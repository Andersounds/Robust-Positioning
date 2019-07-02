#include <iostream>
#include <fstream> //ofstream for out, ifstream for in, fstream for both
//#include <regex>//For syntax Check
#include <opencv2/opencv.hpp>
#include "logger.hpp"

log::dataLogger::dataLogger(void){
    bufferSize = 20;
    overwrite = true;
}

int log::dataLogger::init(std::string path, std::vector<std::string> dataFields){


    if(overwrite){
        if(!settingsFile.open(path, std::ofstream::out | std::ofstream::trunc)) return 0;//Trunc option clears the file before anything is written
    } else{
        if(!settingsFile.open(path, std::ofstream::out | std::ofstream::trunc)) return 0;
        file_true.open(path, std::ios::out | std::ios::app);
    }
}
