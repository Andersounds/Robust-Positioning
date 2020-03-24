#include <iostream>
#include <fstream> //Input stream from file
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>
#include "../src/boostParserUtilities.cpp"


/*
--------------Boilerplate program for boost::program_options-------------------
-Options defined in one or more options_description objects. Possibly with init values
-Options passed either on command line or with configuration file
-Priority:  1. Command line
            2. Configuration file
            3. [if defined] default value

All options are to be defined in every program to keep relevance



Compilation command:
g++ -std=c++11 -fvisibility=hidden /usr/local/lib/libboost_program_options.a `pkg-config --cflags --libs opencv` examples/program_options_boilerPlate.cpp -o bin/example

########### Example configuration file ###########
######### Initial values:
XYZ_INIT =[0,0,-1.8]                    #Initial position expressed as [X,Y,Z] coordinate floats
ROLL_INIT = 0                           #Initial roll of UAV, radians
PITCH_INIT = 0                          #Initial pitch of UAV, radians
YAW_INIT =0                             #Initial yaw of UAV, radians

######## Program settings:
OUT = outFile.csv                       #Write output data to specified file. No output is not set
TILT_COLUMNS =[4,3]                     #Specifies which columns of csv file that contains [roll,pitch] data (0-indexed)
DIST_COLUMN =2                          #Specifies which column of csv file that contains distance (lidar) data
PATH_TO_ARUCO_DATABASE = anchors.csv     #Path to anchor database from base path

*/


int main(int argc, char** argv)
{
    namespace bpu=boostParserUtilites;

    boost::program_options::variables_map vm;

    boostParserUtilites::readCommandLine(argc, argv,vm);

    std::cout << "Checking program options..." << std::endl;
    if(vm.count("K_MAT")){
        std::cout<< "Read K mat as string: " << vm["K_MAT"].as<std::string>() << std::endl;
        cv::Mat_<float> M;
        boostParserUtilites::string2CVMat(vm["K_MAT"].as<std::string>(), M);
        std::cout << "K mat as cv float mat:\n" << M << std::endl;
    }
    std::cout << "Checking init values..." << std::endl;
    if(vm.count("XYZ_INIT")){
        std::cout<< "XYZ init: " << vm["XYZ_INIT"].as<std::string>() << std::endl;
    }
    std::cout << "Checking dist column..." << std::endl;
    if(vm.count("DIST_COLUMN")){
        std::cout<< "DIST_COLUMN =  " << vm["DIST_COLUMN"].as<int>() << std::endl;
    }



    std::cout << "Checking BASE_PATH..." << std::endl;
    if(vm.count("BASE_PATH")){
        std::cout<< "BASE_PATH =  " <<vm["BASE_PATH"].as<std::string>() << std::endl;
        //std::cout<< "BASE_PATH =  " <<vm["BASE_PATH"].as<int>() << std::endl;
    }


    std::cout << "Assigning....." << std::endl;
    int variable;
    std::string key = "OPTICAL_FLOW_GRID";
    bpu::assign(vm, variable,key);
    std::cout << "value: " << variable << std::endl;

}



/*
    This function is to specify all options. Unique for all programs.
*/
int boostParserUtilites::readCommandLine(int argc, char** argv,boost::program_options::variables_map& vm){
    // Declare a group of options that will be
    // allowed only on command line
    namespace po=boost::program_options;

    po::options_description generic("Command line options");
    generic.add_options()
        ("help,h", "produce help message")
        ("file,f",po::value<std::string>(),"configuration file")// Possibly set this as first positional option?
        //("BASE_PATH,p",po::value<std::string>(),"Base path from which I/O paths are relative. Defaults to pwd but may be overridden with this flag.\nGive as either absolute or relative path.")
    ;

    po::options_description parameters("Parameters");
    parameters.add_options()
        //Parameters
        ("RES_XY",  po::value<std::string>(), "Camera resolution in X and Y direction")
        ("K_MAT",  po::value<std::string>(), "Camera K matrix specified in matlab style. ',' as column separator and ';' as row separator") //Tänk om man kan definiera denna direkt som en opencv mat och ge 9 argument på rad?
        ("T_MAT",  po::value<std::string>()->default_value("[0,-1,0;1,0,0;0,0,1]"), "UAV-to-Camera matrix. Default +90deg. Specified in matlab style")
        ("CAMERA_BARREL_DISTORTION",    po::value<std::string>()->default_value("[0.2486857357354474,-1.452670730319596,2.638858641887943]"), "Barrel distortion coefficients given as [K1,K2,K3]")
        ("OPTICAL_FLOW_GRID",           po::value<int>()->default_value(4),"Sqrt of number of optical flow vectors")//Single int
        ("ROI_SIZE",po::value<int>()->default_value(150), "Side length of VO ROI. Used to edit K mat of VO alg.")
        ;

    po::options_description initValues("Initial values");
    initValues.add_options()
        ("XYZ_INIT",                    po::value<std::string>()->default_value("[0,0,-1.8]"), "Initial position expressed as [X,Y,Z] coordinate floats")
        ("ROLL_INIT", po::value<float>()->default_value(0),"Initial roll of UAV, radians")
        ("PITCH_INIT", po::value<float>()->default_value(0),"Initial pitch of UAV, radians")
        ("YAW_INIT", po::value<float>()->default_value(0),"Initial yaw of UAV, radians")
        ;
    po::options_description modes("Program settings");
    modes.add_options()
        ("OUT,o",   po::value<std::string>()->default_value("[9,9,9]"), "Write output data to specified file. No output is not set")// Single string argument
        ("TILT_COLUMNS", po::value<std::string>()->default_value("[4,3]"),"Specifies which columns of csv file that contains [roll,pitch] data (0-indexed)")
        ("DIST_COLUMN", po::value<int>()->default_value(1),  "Specifies which column of csv file that contains distance (lidar) data")
        ("PATH_TO_ARUCO_DATABASE", po::value<std::string>()->default_value("anchors.csv"),"Path to anchor database from base path")
        ;


    // Parse command line
    po::options_description all("All options");
    all.add(generic).add(parameters).add(initValues).add(modes);
    po::store(po::parse_command_line(argc, argv, all), vm);//Read command line
    po::notify(vm);
    /*Produce help message */
    if (vm.count("help")) {
        std::cout << generic<< std::endl;
        std::cout << "All parameters below are to be defined in a configuration file specified with flag -f" << std::endl;
        std::cout << "Format: \nPARAMETER_FLAG_1 = <value>   #Disregarded comment\nPARAMETER_FLAG_2 = <value>   #Some other comment" << std::endl;
        std::cout << parameters<< std::endl;
        std::cout << initValues<< std::endl;
        std::cout << modes << std::endl;
        std::cout << "---------------" << std::endl;
        return 0;
    }
    /*Read settings from file if specified*/
    if(vm.count("file")){
        std::string iniFile = vm["file"].as<std::string>();
        std::cout << "Reading configuration file " << iniFile << "...";
        std::ifstream ini_file(iniFile);//Try catch block?
        po::store(po::parse_config_file(ini_file, all, true), vm);//What is true?
        std::cout << "Done." << std::endl;
        //Add BASE_PATH option
        std::string basePath;
        std::string configFile;
        boostParserUtilites::basePathFromFilePath(iniFile,basePath,configFile);
        std::cout << "--Set --BASE_PATH to '" << basePath << "'." << std::endl;
        std::cout << "--Config file is '" << configFile << "'." << std::endl;
        vm.insert(std::make_pair("BASE_PATH", po::variable_value(basePath, false)));
        po::notify(vm);
    }else{
        std::cout << "WARNING: No configuration file specified. BASE_PATH unusable" << std::endl;

    }
    return 0;
}

//https://www.pyimagesearch.com/2015/04/27/installing-boost-and-boost-python-on-osx-with-homebrew/
