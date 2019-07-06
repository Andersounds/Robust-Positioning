#include <iostream>
#include <fstream>
#include "dataStream.hpp"





// Constructor that reads the provided csv file and converts it to vector<vector<float>>
robustPositioning::dataStreamer::dataStreamer(std::string path){
    skipLines = 1;
    readOnlySpecifiedSize = false;
    initialize(path);
}
robustPositioning::dataStreamer::dataStreamer(std::string path,int skipLines_){
    skipLines = skipLines_;
    readOnlySpecifiedSize = false;
    dataFields = 0;
    initialize(path);
}
robustPositioning::dataStreamer::dataStreamer(std::string path,int skipLines_,int dataFields_){
    skipLines = skipLines_;
    readOnlySpecifiedSize = true;
    dataFields = dataFields_;
    initialize(path);
}
//This is the method that is used to get the next datarow
bool robustPositioning::dataStreamer::get(std::vector<float>& nextData){
    nextData.clear(); //make sure that it is empty
    static int sequence = -1;//start at -1 so that it is 0 on first return
    sequence++;
    if(sequence<data_f.size()){
        nextData = data_f[sequence];
        return true;
    }else{
        return false;
    }
}

void robustPositioning::dataStreamer::initialize(std::string path){
    std::cout << "Initializing csv data streamer..." << std::endl;
    std::cout << "\tReading data file..." << std::endl;
    if(!readDataFile(path)){
        std::cout << "\tCould not open specified data file: \"" << path << "\". Is it really correct?" <<std::endl;
        std::cout << "Failed." << std::endl;
        return;
    }
    //convert string data to float data. If valie can not be converted to float then it is replaced by the value replace to keep ordering
    float replace = -1000000;
    int row_nmbr = skipLines;
    std::vector<std::string> failed_to_convert;
    std::vector<int> failed_to_convert_row_nmbr;
    for(std::vector<std::string> row:data_s){
        row_nmbr++;
        std::vector<float> row_float;
        for(std::string value_s:row){
            try{
                float value_f = std::stof(value_s);
                row_float.push_back(value_f);
            } catch(const std::invalid_argument& e){

                row_float.push_back(replace);
                failed_to_convert.push_back(value_s);
                failed_to_convert_row_nmbr.push_back(row_nmbr);
            }
        }
        data_f.push_back(row_float);//Save the float-row to the data_f
    }
    std::cout << "\tRead " << data_f.size() << " rows from " << path << std::endl;
    if(failed_to_convert.size()>0){
        std::cout << "\tFailed to convert the following variables to float. Replaced values with " << replace << "." << std::endl;
        for(int e=0;e<failed_to_convert.size();e++){
            std::cout << "\t\t- " << failed_to_convert[e] << "\" on row " << failed_to_convert_row_nmbr[e] << std::endl;
        }
    }
    std::cout << "Done." << std::endl;
}


int robustPositioning::dataStreamer::readDataFile(std::string path){
    std::string line;
    std::string delim = ",";
    std::ifstream file;
    file.open(path);
    int count = 0;
    if(file.is_open()){
         while(getline(file,line)){
            count++;
            if(count<=skipLines) continue;//skip reading this line if we should skip it
            std::vector<std::string> parsed = parseRow(line);
            if(parsed.size()==dataFields || dataFields==0){//Disregard any lines that are not dataFields long, if dataFields has been set
                data_s.push_back(parsed);//save the string data
            } else{std::cout << "\t\tSkipped data row: \""<< line << "\".";}
         }
    }else{return 0;}
    file.close();
    return 1;
}
//This function takes a line and parses it into a vector<string> using "," as deliminator and disregarding LEADING whitespaces
std::vector<std::string> robustPositioning::dataStreamer::parseRow(std::string line){
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
