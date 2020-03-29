#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
#include "../src/simulatePose.hpp"
#include "../src/logger.hpp"
#include "../src/timeStamp.cpp"
#include "../src/dataStream.hpp"

#include "../src/boostParserUtilities.cpp" //Includes boost and two utility functions
#define PI 3.1416


int main(int argc, char** argv){
timestamp::timeStamp_ms stamp;
double timeStamp;
stamp.get(timeStamp);


//Initialize settings
namespace bpu=boostParserUtilites;
boost::program_options::variables_map vm;
//if(!boostParserUtilites::readCommandLine(argc, argv,vm)) return 0;





std::cout << "This file will be edited to be used to create simulation dataset" << std::endl;
std::cout << "1. Read a predefined true path with roll, pitch, yaw" << std::endl;
std::cout << "2. Simulation background. chessboard? backgroundimage?" << std::endl;
std::cout << "3. What are the physical dimensions of scene?" << std::endl;
std::cout << "4. Camera parameters? what are they?" << std::endl;
std::cout << "5. When saving the data, a complete settings file shall also be created (for k mat etc)" << std::endl;


std::cout << "NEW INFO" << std::endl;
std::cout << "Provide path to sim settings file" << std::endl;
std::cout << "mandatory settings:" << std::endl;
std::cout << "K mat" << std::endl;
std::cout << "base scene width in meters" << std::endl;
std::cout << "base scene. either path to image or something else to specify chessboard" << std::endl;
std::cout << "optional settins" << std::endl;
std::cout << "T mat" << std::endl;
std::cout << "Base scene coordinate system must be aligned with pixel coordinates." << std::endl;
std::cout << "The base scene will be resized to camera resolution proportions by padding with black pixles" << std::endl;

robustPositioning::imageLogger imagebin;
std::string pathToDir = "Generated-dataSets/29-mar/";
std::string newDir = "images";
imagebin.init(pathToDir,newDir);

//robustPositioning::dataStreamer getData("Generated-dataSets/5_jul/truePath.csv");
robustPositioning::dataStreamer getData("Generated-dataSets/13-okt/path.csv");


    //Initialize settings
    //set::settings S(argc,argv);
    //if(!S.success){return 0;}
    //Initialize simulation
    // Define sim chessboard parameters
    int boxWidth = 11;
    int rowsOfBoxes = 30;
    int colsOfBoxes = 60;
// Define file objects to output data into
//    std::ofstream file_true;
//    std::ofstream file_estimated;
// Init simulation environment
    simulatePose warper;
    //cv::Mat floor = cv::imread("spike/test2.png",cv::IMREAD_REDUCED_COLOR_4);
    cv::Mat floor = cv::imread("Generated-dataSets/Scene/baseScene.png");
    if(floor.empty()){std::cout << "Could not read base image" << std::endl; return 0;}
    cv::Mat floor8U;
    cv::cvtColor(floor, floor8U, cv::COLOR_BGR2GRAY);

    cv::Mat_<float> K = cv::Mat_<float>::eye(3,3);
    K(0,0) = 607;
    K(1,1) = 607;
    K(0,2) = 320;
    K(1,2) = 240;
    warper.setBaseScene(floor);
    warper.setBaseSceneWidth(4);    //Scenewidth is 4m
    warper.setKMat(K);
    warper.init();//Initialize with configuration 0


std::cout << "Cpordinates are correct? sign of z?" << std::endl;

//Go through whole path
std::vector<float> data;
    while(getData.get(data)){
        //Get data
        std::vector<float> trueCoordinate{data[1],data[2],data[3]};
        std::vector<float> angles{data[4],data[5],data[6]};
    //Get new image
        cv::Mat rawFrame = warper.simulateView(angles,trueCoordinate);
        //cv::Mat rawFrame = warper.uav2BasePose(angles,trueCoordinate);
        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);
        cv::imshow("showit",rawFrame);

        //Dump image
        //stamp.get(timeStamp);
        timeStamp = (double)data[0];
        imagebin.dump(timeStamp,frame);
        //cv::waitKey(0);
        if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}
    }
    //imagebin.rename(cv::String(pathToDir+newDir+"/"),cv::String("img_"));
    return 1;
}



/*
 *
 * Definition of allowed program options according to the prototype defined in namespace boostParserUtilites
 *
 *
 *
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
    po::options_description settings("Program settings");
    settings.add_options()
        ("STREAM_DATA_FILE",    po::value<std::string>(),   "Path to data file from config file path")
        ("XYZ_COLUMS",          po::value<std::string>(),   "Specifies [<X_COL>,<Y_COL>,<Z_COL>] as matlab style vector (0-indexed)")
        ("TIMESTAMP_COL,",      po::value<int>(),           "Specifies which column of csv file that contains timestamp data (0-indexed)")
        ("DIST_COLUMN",         po::value<int>(),           "Specifies which column of csv file that contains distance (lidar) data (0-indexed)")
        ("R_P_Y_COLUMNS",         po::value<int>(),         "Specifies which columns of csv file that contains [roll,pitch,yaw] data (0-indexed)")
        ("BASESCENE",           po::value<std::string>()->default_value(""),   "Path to the base scene image that is to be warped. Defaults to chess board")
        ("BASESCENE_WIDTH",      po::value<float>(),        "Physical width of base scene in meter (x-dir)")
        ("K_MAT",  po::value<std::string>(),                "Camera K matrix specified in matlab style. ',' as column separator and ';' as row separator")
        ("CHESSBASE_BOXWIDTH",         po::value<int>()->default_value(11),     "Size of each chessbox if not custom --BASESCENE is used")
        ("CHESSBASE_ROWS",         po::value<int>()->default_value(30),         "Number of chessbox rows if not custom --BASESCENE is used")
        ("CHESSBASE_COLS",         po::value<int>()->default_value(60),         "Number of chessbox cols if not custom --BASESCENE is used")
        ;

    // Parse command line
    po::options_description all("All options");
    all.add(generic).add(settings);
    po::store(po::parse_command_line(argc, argv, all), vm);//Read command line
    po::notify(vm);
    /*Produce help message */
    if (vm.count("help")) {
        std::cout << generic<< std::endl;
        std::cout << "All parameters below are to be defined in a configuration file specified with flag -f" << std::endl;
        std::cout << "Format: \nPARAMETER_FLAG_1 = <value>   #Disregarded comment\nPARAMETER_FLAG_2 = <value>   #Some other comment" << std::endl;
        std::cout << settings<< std::endl;
        std::cout << "---------------" << std::endl;
        std::cout << "This program takes a trajectory csv file containing timestamps, roll, pitch, yaw, x, y, z \n";
        std::cout << " and simulates a UAV collected dataset by homographic warping of a defined basescene.\n";
        std::cout << "Program output to a new directory 'images' in the directory of the given configuration file (flag -f).\n";
        std::cout << "All paths in configuration file are relative to the drectory of the configuration file itself.\n";
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
        std::string file;
        std::string basePath;
        boostParserUtilites::basePathFromFilePath(iniFile,basePath,file);
        std::cout << "Set --BASE_PATH to '" << basePath << "'." << std::endl;
        vm.insert(std::make_pair("BASE_PATH", po::variable_value(basePath, false)));
        po::notify(vm);
    }else{
        std::cerr << "No mandatory --file argument given. " << std::endl;
        return 0;
    }

    // Check format of some critical inputs
        namespace bpu=boostParserUtilites;
        if(!bpu::checkDimOfCVMatOption(vm,"K_MAT",3, 3)){return 0;}
        if(!bpu::checkDimOfCVMatOption(vm,"XYZ_COLUMNS",1, 3)){return 0;}
        if(!bpu::checkDimOfCVMatOption(vm,"R_P_Y_COLUMNS",1, 3)){return 0;}


    return 1;
}
