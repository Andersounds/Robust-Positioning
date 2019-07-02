#include <iostream>
#include <fstream> //ofstream for out, ifstream for in, fstream for both
#include <limits> //
#include <opencv2/opencv.hpp>
#include "logger.hpp"

log::dataLogger::dataLogger(void){
    bufferLength = 30;
    overwrite = true;
    sep = ',';

    //Info regarding overflow

    float maxint = std::numeric_limits<float>::max();
    float hours = maxint/1000/3600;
    std::cout << "Time until overflow of timestamp: " << hours << " hours"<< std::endl;
    std::cout << "Consider using double-logger if overflow will occur too fast" << std::endl;
}

int log::dataLogger::init(std::string path, std::vector<std::string> dataFields){
    bufferWidth = dataFields.size();
    fullPath = path;
    if(bufferWidth<2){
        std::cout << "Too few datafields in dataLogger::init. Must be at least 2." <<std::endl;
        return 0;
    }
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
    file.close();
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
    theFile << '\n';
    return 1;
}
/*Method for writing to the buffer, which when it is full is dumped to the file*/
int log::dataLogger::dump(std::vector<float>& data){
    static int counter = 0;
    std::vector<float>::iterator it = data.begin();
    file.open(fullPath, std::ofstream::out | std::fstream::app);
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
    file.close();
    return 1;
}


/*
 * Source code for image logger
 *
 *
 *
 */
