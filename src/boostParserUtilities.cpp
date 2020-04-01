#include <iostream>
#include <fstream> //Input stream from file
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>

/*
These functions are meant to be used with boost program options command line and config file parser

*/


/*
Write an assign-function here with overloaded functions for different types. calls string2cvmat accordingly
Also raises appropriate error if setting is missing

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
 Function to convert matlab style vector definition to std::vector
*/
int string2vec(std::string str0, std::vector<int>& v){
    v.clear();
    boost::trim_if(str0,boost::is_any_of("[]"));//Trim brackets
    std::vector<std::string> SplitVec;
    boost::split(SplitVec, str0, boost::is_any_of(",;"));//Split into elements with either deliminator
    for(std::string element:SplitVec){
        int element_i;
        try{
            element_i = std::stoi(element);
        }catch(...){
            std::cerr << "ERROR: could not convert element '" << element <<"' to int." << std::endl;
            throw(1);
            return 0;
        }
        v.push_back(element_i);
    }
return 1;
}
/*
    Overloaded conversion function for floats
*/
int string2vec(std::string str0, std::vector<float>& v){
    v.clear();
    boost::trim_if(str0,boost::is_any_of("[]"));//Trim brackets
    std::vector<std::string> SplitVec;
    boost::split(SplitVec, str0, boost::is_any_of(",;"));//Split into elements with either deliminator
    for(std::string element:SplitVec){
        int element_i;
        try{
            element_i = std::stof(element);
        }catch(...){
            std::cerr << "ERROR: could not convert element '" << element <<"' to float." << std::endl;
            throw(1);
            return 0;
        }
        v.push_back(element_i);
    }
return 1;
}

/*
    Set of overloaded functions that assigns casted option to given inputoutput argument
*/

int assign(const boost::program_options::variables_map& vm, int& var,std::string key){
    try{
        var = vm[key].as<int>();
        return 0;
    }catch(...){
        std::cerr << "ERROR: could not convert element '" << key <<"' to int." << std::endl;
        std::cerr << "In boostParserUtilites::assign" << std::endl;
        return 1;
    }
}
int assign(const boost::program_options::variables_map& vm, float& var,std::string key){
    try{
        var = vm[key].as<float>();
        return 0;
    }catch(...){
        std::cerr << "ERROR: could not convert element '" << key <<"' to float." << std::endl;
        std::cerr << "In boostParserUtilites::assign" << std::endl;
        return 1;
    }
}
int assign(const boost::program_options::variables_map& vm, std::string& var,std::string key){
    try{
        var = vm[key].as<std::string>();
        return 0;
    }catch(...){
        std::cerr << "ERROR: could not convert element '" << key <<"' to string." << std::endl;
        std::cerr << "In boostParserUtilites::assign" << std::endl;
        return 1;
    }
}
int assign(const boost::program_options::variables_map& vm, cv::Mat_<float>& var,std::string key){
    try{
        std::string var_STR = vm[key].as<std::string>();
        string2CVMat(var_STR,var);
        return 0;
    }catch(...){
        std::cerr << "ERROR: could not convert element '" << key <<"' to cv::Mat_<float>." << std::endl;
        std::cerr << "In boostParserUtilites::assign" << std::endl;
        return 1;
    }
}
int assign(const boost::program_options::variables_map& vm, std::vector<int>& var,std::string key){
    try{
        std::string var_STR = vm[key].as<std::string>();
        string2vec(var_STR,var);
        return 0;
    }catch(...){
        std::cerr << "ERROR: could not convert element '" << key <<"' to std::vector<int>." << std::endl;
        std::cerr << "In boostParserUtilites::assign" << std::endl;
        return 1;
    }
}
int assign(const boost::program_options::variables_map& vm, std::vector<float>& var,std::string key){
    try{
        std::string var_STR = vm[key].as<std::string>();
        string2vec(var_STR,var);
        return 0;
    }catch(...){
        std::cerr << "ERROR: could not convert element '" << key <<"' to std::vector<float>." << std::endl;
        std::cerr << "In boostParserUtilites::assign" << std::endl;
        return 1;
    }
}
int assign(const boost::program_options::variables_map& vm, bool& var,std::string key){
    std::string var_STR = vm[key].as<std::string>();
    if(var_STR == "YES"){
        var = true;
    }else if(var_STR == "NO"){
        var = false;
    }else{
        std::cerr << "ERROR: could not convert element '" << key <<"' to bool <YES/NO>" << std::endl;
        std::cerr << "In boostParserUtilites::assign" << std::endl;
        return 1;
    }
    return 0;

}
//Check dimension of matrix
//Error: return 0
//Pass:  return 1
int checkDimOfCVMatOption(const boost::program_options::variables_map& vm,std::string KEY, int rows, int cols){
    cv::Mat_<float> mat_test;
    assign(vm,mat_test,KEY);
    int actualRows = mat_test.rows;
    int actualCols = mat_test.cols;
    if(actualRows!=rows || actualCols!=cols){
        std::cerr << "Incorrect format of option --" << KEY << ". Should be [" << rows << "x"<< cols <<"]." << std::endl;
        std::cerr << "  Actual dimentions: [" << actualRows << "x" << actualCols << "]." << std::endl;
        return 0;
    }
    return 1;
}



/*
    Extracts the base path including last '/' from a given filepath. If just filename is given it returns
    an empty string
*/
int basePathFromFilePath(std::string str,std::string&base, std::string&file){
    std::size_t found = str.find_last_of("/");
    if(found==str.npos){
        base = "";
        file = str;
    }else{
        base = str.substr(0,found+1);
        file = str.substr(found+1,str.npos);
    }
    return 1;
}

}
