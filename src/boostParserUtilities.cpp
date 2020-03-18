#include <iostream>
#include <fstream> //Input stream from file
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>

/*
These functions are meant to be used with boost program options command line and config file parser

*/


namespace boostParserUtilites{
    // Prototype function. Allows the function to be defined below main function for readability.
    // See program_options_boilerPlate.cpp for reference
    int readCommandLine(int, char**,boost::program_options::variables_map&);

/*
    Method that takes a string, and converts it to a opencv mat_<float>
    Possibly overload this with int versions
    Matrices given as string in matlab style using ',' as column separator, ';' as row separator
*/
int string2CVMat(std::string str0, cv::Mat_<float>& M){
    boost::trim_if(str0,boost::is_any_of("[]"));//Trim brackets
    std::vector<std::string> SplitVec;
    boost::split(SplitVec, str0, boost::is_any_of(";"));//Split into rows
    int rows = SplitVec.size();

    std::vector<float> V;
    int cols = -1;
    for(std::string rowStr:SplitVec){//
        std::vector<std::string> row;//Split row string to string elements
        boost::split(row, rowStr, boost::is_any_of(","));//Must have ',' as column delimiter
        cols = row.size();//Will be redefined for every row but must always be same so whatever
        for(std::string i:row){
            std::string elementSTR = boost::trim_copy(i);
            float element;
            try{
                element = std::stof(elementSTR);
            }catch(...){
                std::cerr << "ERROR: could not convert element '" << elementSTR <<"' to float." << std::endl;
                std::cerr << "In string2CVMat" << std::endl;
                return 1;
            }
            V.push_back(element);//remove whitespaces, convert to float and push back
        }
    }
    cv::Mat_<float> V2;
    try{
        V2 = cv::Mat(V).reshape(cols);
    }catch(...){
        std::cerr << "ERROR: Specified matrix does not have consistent number of columns" << std::endl;
        std::cerr << "In string2CVMat" << std::endl;
        throw(1);
    }
    try{
        V2.copyTo(M); //Do this inside another try block to catch specific error
        return 0;
    }catch(...){
        std::cerr << "ERROR: Could not copy parsed matrix onto inputoutput cv mat." << std::endl;
        std::cerr << "In string2CVMat" << std::endl;
        throw(1);
    }
return 1;
}


/*
    Extracts the base path including last '/' from a given filepath. If just filename is given it returns
    an empty string
*/
std::string basePathFromFilePath(std::string str){
    std::size_t found = str.find_last_of("/");
    if(found==str.npos){
        return "";
    }else{
        return str.substr(0,found+1);
    }
}

}
