#include <iostream>
#include <fstream> //ofstream for out, ifstream for in, fstream for both
#include <regex>
#include <dirent.h>
#include <sys/stat.h>//For creating directory
//#include <limits> //
#include <opencv2/opencv.hpp>
#include "logger.hpp"

log::dataLogger::dataLogger(void){
    bufferLength = 30;
    overwrite = true;
    sep = ',';
    //Info regarding overflow
    /*
    float maxint = std::numeric_limits<float>::max();
    float hours = maxint/1000/3600;
    std::cout << "Time until overflow of timestamp: " << hours << " hours"<< std::endl;
    float days = hours/24;
    std::cout << "                                  " << days  << " days"<< std::endl;
    float years = days/365.25;
    std::cout << "                                  " << years  << " years"<< std::endl;
    std::cout << "Consider using double-logger if overflow will occur too fast" << std::endl;
    */
}
log::dataLogger::~dataLogger(void){
    file.open(fullPath, std::ofstream::out| std::fstream::ate);
    file.flush();
    file.close();
}
int log::dataLogger::init(std::string path, std::vector<std::string> dataFields){
    fullPath = path;
    //Test open data file and if applicable, write the names of the datafields
    if(overwrite){
        file.open(fullPath, std::ofstream::out | std::ofstream::trunc);//Trunc option clears the file before anything is written
        if(!file.is_open() || !writeInfoLine(file,dataFields)){
            std::cout << "Could not open file: \"" << path << "\". Aborting."<<std::endl;
            return 0;
        }
    } else{
        file.open(fullPath, std::ofstream::out| std::fstream::ate);//Opens file to check validity of path
        if(!file.is_open()){
            std::cout << "Could not open file: \"" << path << "\". Aborting." << std::endl;
            return 0;
        }
    }
    //file.close();
    return 1;
}

int log::dataLogger::init(std::string path, std::vector<std::string> dataFields, int bufSize, bool ovrwrite){
    bufferLength = bufSize;
    overwrite = ovrwrite;
    return init(path, dataFields);
}
/* Writes the info line in an already open file. First element is always timestamp*/
int log::dataLogger::writeInfoLine(std::ofstream& theFile, std::vector<std::string> dataInfo){
    theFile << dataInfo[0];
    for(int i=1;i<dataInfo.size();i++){
        theFile << sep << dataInfo[i];
    }
    theFile << std::endl;
    return 1;
}
/*Method for writing to the buffer, which when it is full is dumped to the file*/
int log::dataLogger::dump(std::vector<float>& data){
    static int counter = 0;
    std::vector<float>::iterator it = data.begin();
    //file.open(fullPath, std::ofstream::out | std::fstream::app);
    file << *it;
    it++;
    while(it!=data.end()){
        file << "," << *it;
        it++;
    }
    file << '\n'; //Not std::endl since this flushes the buffer and we do not need it right now
    counter++;//increment counter.
    if(counter>bufferLength){ //If counter is high enough, make sure that buffer is flushed
        counter = 0;
        file.flush();
    }
    //file.close();
    return 1;
}
int log::dataLogger::dump(std::vector<std::string>& data){
    static int counter = 0;
    std::vector<std::string>::iterator it = data.begin();
    //file.open(fullPath, std::ofstream::out | std::fstream::app);
    file << *it;
    it++;
    while(it!=data.end()){
        file << "," << *it;
        it++;
    }
    file << '\n'; //Not std::endl since this flushes the buffer and we do not need it right now
    counter++;//increment counter.
    if(counter>bufferLength){ //If counter is high enough, make sure that buffer is flushed
        counter = 0;
        file.flush();
    }
    //file.close();
    return 1;
}

/*
 * Source code for image logger
 *
 *
 *
 */

 log::imageLogger::imageLogger(void){
     imgBaseStr = "img_";       //After renaming the images will by default have the format img_XXXX.png
     dumpDirName = "images";        //By default images arw written to a new directory called images in the given directory
     imgFormatStr = ".png";
     numOfDigits = 4;
     renameFile = "data.csv";
 }

 // https://techoverflow.net/2013/04/05/how-to-use-mkdir-from-sysstat-h/
int log::imageLogger::init(std::string dumpDir){
     //Try our best to interpret the user given path and add one more directory to it with name dumpDirName
    if(dumpDir==""){
        dirPath = dumpDirName;
    }else if(std::regex_match(dumpDir,std::regex("(.*)(/)"))) {
        dirPath = dumpDir + dumpDirName;
    }else{
        dirPath = dumpDir + "/" + dumpDirName;
    }
    //Create directory. mkdir takes a const char object in first argument and not std::string. So it has to be called by .c_str()
    if(mkdir(dirPath.c_str(), 0777) == -1){//Mode 0777 is a directory that is open for everyone for everything
        std::cout << "Created directory for dumping images." << std::endl;
    } else{
        std::cout << "Could not create directory with path \"" << dirPath << "\". " << std::endl;
        return 0;
    }
    return 1;
}
 int log::imageLogger::init(std::string dumpDir, std::string format){
     imgFormatStr = format;
     return init(dumpDir);
 }
 int log::imageLogger::dump(const double& timeStamp, cv::Mat& image){
     int no_decimals = static_cast<int>(timeStamp);
     std::string ts = std::to_string(no_decimals);
     std::string file = dirPath + "/" + imgBaseStr + ts + imgFormatStr;
     std::vector<int> params;
     if(!cv::imwrite(file,image,params)){
         std::cout << "Could not write file. " << std::endl;
     }
     return 1;
 }

 int log::imageLogger::rename(cv::String path,cv::String baseName){
     //Get all filenames at the specified path
     std::cout << "Searching for files in \"" << path << "\"...";
     std::vector<cv::String> results; // We have to use cv::String and not std::string here
     cv::String pattern = path;  // Same here
     glob(pattern, results);
     std::cout << "Found " << results.size() << " files using cv::GLOB:"<< std::endl;
     std::vector<std::string> results_std;
     for(int j=0;j<results.size();j++){
         //std::cout << results[j].c_str() << std::endl; //Can convert cv:String to const char * via c_str()
         std::string filename = results[j].c_str();   // std::string has char * constructor so we can do this
         results_std.push_back(filename);
        // std::cout << j+1 << ":\t"<<results_std[j] << std::endl;
     }
    // std::cout << "==========END==========" << std::endl;
     //Create sorter object and let it know what is the basename of the files
     std::string path_rename = path.c_str();
     std::string name_rename = baseName.c_str();
     std::string basePathRename = path_rename + name_rename;
     std::cout << "Base string of files is: \"" << basePathRename << "\"." << std::endl;
     fileNameSort sorter(basePathRename);
     //Sort the whole list numerically
     sorter.numericSort(results_std);
     //Extract the timestamps
     std::vector<std::string> timeStamps = sorter.extractTimeStamps(results_std);
     //Initialize datalogger to save a data file
     dataLogger fileWriter;
     std::string path_std = path.c_str();
     if(!fileWriter.init(path_std+"data.csv",std::vector<std::string>{"Timestamp [ms]","File name"})){
         std::cout << "Could not initialize data file " <<path_rename << "data.csv"<< std::endl;
         return 0;
     }
     int num_of_files = results_std.size();
     int num_of_digits = std::to_string(num_of_files).size();
     for(int i=0;i<num_of_files;i++){
         std::string stamp = timeStamps[i];
         //Create padding string
         std::to_string(i).size();
         int padLength = num_of_digits - std::to_string(i).size();
         std::string padding(padLength, '0');
         std::string newName = name_rename + padding + std::to_string(i) + sorter.imgEndName;
         std::vector<std::string> data{stamp,newName};
         fileWriter.dump(data);
     }


     return 1;
 }
