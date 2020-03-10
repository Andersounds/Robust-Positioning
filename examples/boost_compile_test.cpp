//#include <boost/lambda/lambda.hpp>
#include <iostream>
#include <fstream> //Input stream from file
//#include <iterator>
//#include <algorithm>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
//#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
/*
 PARSE COMMAND LINE:
    (First define all options. As well as the config file name)
    - read argc argv
    - is config file option given?
        - Then pare config file
    - Then read rest of command line.

    When variable are defined from value map. If needed then include conversion function


    - How to write default ini file?
    - How to let command line override ini file?
        MULTIPLE SOURCES: https://www.boost.org/doc/libs/1_72_0/doc/html/program_options/tutorial.html
        DONE.
    - Sectioning. See abit in link above
*/


/*
    How its is to be used.
        - Every program defines their own program options.
        - This is done in a separate function defined in the main file
        - The function defines options, their default values
        - Reads argc argc
        - For each option, one conversion is done to a correct format, and then added to a map with the same keys
        - This map is passed back to main, which accesses values using the same keys as settings.


        - How to write config file?
        - Order options in categories
            -multiple sources
                https://www.boost.org/doc/libs/1_72_0/doc/html/program_options/tutorial.html
        - Write parameter description as comment in config file
            https://www.boost.org/doc/libs/1_55_0/doc/html/program_options/overview.html#idp163379208
        - config file multitoken
            - in ini-file, one line can only be one value.
            - How can multitoken be handled properly? Ideas below.
                1. Do not use multitoken. Instead specify as single string and write parser for it.
                    - Simple parser that
                        -Goes through whole string
                            -if char is numeric
                            -read all numbers in a row, allowing a single point '.' somewhere
                            -Convert this string to either int or float depending on option
                        - This will allow to write pretty much however. [1,2,3;4,5,6] | 1 2 3 4 5 6 etc.


    Additional options:
        - Write default settings file
            - Possibly at some specified path
        - Settings file must be given as argument.
            - Can give just settings file, and its directory will be parsed, and all paths in file are relative to this path
            - Option to specifically specify base path, i.e path to some other dataset. Other paths will then be relative to this instead
        - ARguments given om command line should override arguments in ini file.
*/


/*
    New comments regarding function of settings parser
        - Every program should probably define options in the main function itself. That way relevance is kept
        - No multitoken works due to ini file. But I can write a separate set of overloaded functions that parse strings of matlab style to either float or int mat or vector
        - Should maybe write a class that contains all boost program options including 2 additional methods:
            - conversion methors as stated above
            - function to write default ini file including sections and comments


*/



/*
    Method that takes a string, and converts it to a opencv mat_<float>
    Possibly overload this with int versions
*/

//int string2CVMat(std::string str0,cv::Mat_<float>& cvMat){
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
            V.push_back(std::stof(boost::trim_copy(i)));//remove whitespaces, convert to float and push back
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



int printDefaultConfigFile(const boost::program_options::options_description& descr){

/*
Is this even necessary? We have the help option that also shows default values as defined in the program
I think we can skip

In help message, just show how to write config file. (as an ini file)
*/


    return 0;
}

/*
INI file of following syntax


ROLL_COLUMN=4 #Works
OUT=testi 1 2 4     #Works
ROLL_INIT=1.2   #Works



Will add string2Mat which will allow:
K_MAT = [1,2,3;1,2,4;1,2.5,3]
*/


//https://www.pyimagesearch.com/2015/04/27/installing-boost-and-boost-python-on-osx-with-homebrew/
int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    // Declare a group of options that will be
    // allowed only on command line
    po::options_description generic("Generic options");
    generic.add_options()
            ("help,h", "produce help message")
            ("f","configuration file")
            ("d","write default configuration file")
            // Also path to output file. But this may be stated both on cmnd line and ini file
    ;

    std::cout << generic << std::endl;

    po::options_description parameters("Parameters");
    parameters.add_options()
        ("OUT,o",   po::value<std::string>()->default_value("[9,9,9]"), "Write output data to specified file. No output is not set")// Single string argument
        //Parameters
        ("RES_XY",  po::value<std::vector<int> >(), "Camera resolution in X and Y direction")
        ("RES_X",  po::value<int>(), "Camera resolution in X direction")
        ("RES_Y",  po::value<int>(), "Camera resolution in X direction")
        ("K_MAT",   po::value<std::vector<float> >(), "Camera K matrix specified as float numbers row by row separated by whitespace") //Tänk om man kan definiera denna direkt som en opencv mat och ge 9 argument på rad?
        ("T_MAT",   po::value<std::vector<float> >(), "UAV - camera T matrix specified as float numbers row by row separated by whitespace")
        ("CAMERA_BARREL_DISTORTION",    po::value<std::vector<float> >(), "Barrel distortion coefficients K1, K2, K3 as floats")
        ("OPTICAL_FLOW_GRID",           po::value<int>(),"Sqrt of number of optical flow vectors")//Single int
        ("XYZ_INIT",                    po::value<std::vector<float> >(), "Initial position expressed as X Y Z coordinate floats")
        ("ROLL_INIT", po::value<float>(),"Initial roll of UAV, radians")
        ("optimization", po::value<int>()->default_value(10), "optimization level")
        ;

            std::cout << parameters << std::endl;
    po::options_description initValues("Initial values");
    initValues.add_options()
//        ("PITCH_INIT", po::value<float>(),"Initial pitch of UAV, radians")
//        ("YAW_INIT", po::value<float>(),"Initial yaw of UAV, radians")
        ("ROLL_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
//        ("PITCH_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
//        ("YAW_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
        ;
    po::options_description moreValues("Something else");
    moreValues.add_options()
        ("RES_XY",  po::value<std::vector<int> >(), "Camera resolution in X and Y direction")
        ;





    // Parse command line
/*    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, parameters), vm);
    po::notify(vm);
*/

//Parse config file
    std::ifstream ini_file("config.ini");
    po::variables_map vm;
    //What are arguments to parse config file?
    po::store(po::parse_config_file(ini_file, parameters, true), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << parameters << "\n";
        std::cout << "HELP" << std::endl;
        return 1;
    }

    if (vm.count("ROLL_COLUMN")) { //Funkar
        std::cout << "Compression level was set to "
     << vm["ROLL_COLUMN"].as<int>() << ".\n";
    } else {
        std::cout << "CRES_XY was not set.\n";
    }
    if (vm.count("OUT")) { //Funkar
        std::cout << "output file: " << vm["OUT"].as<std::string>() << std::endl;
        cv::Mat_<float> M;
        string2CVMat(vm["OUT"].as<std::string>(),M);
        std::cout << "Matrix: " << M << std::endl;
    }else{
        std::cout << "OUT was not set.\n";
    }
    if (vm.count("ROLL_INIT")) { //Funkar
        std::cout << "Float:" << vm["ROLL_INIT"].as<float>() << std::endl;
    }else{
        std::cout << "float not set.\n";
    }

    if (vm.count("RES_XY")) { //funkar
        std::vector<int> resolution = vm["RES_XY"].as<std::vector<int> >();
        for(int i:resolution){
            std::cout << i << ", ";
        }
        std::cout << std::endl;
    }else{
        std::cout << "res int vector not set.\n";
    }
    if(vm.count("optimization")){
        std::cout << "Optimization: " << vm["optimization"].as<int>() << std::endl;
    }

}

//g++ -std=c++11 -fvisibility=hidden /usr/local/lib/libboost_program_options.a examples/boost_compile_test.cpp -o bin/example
