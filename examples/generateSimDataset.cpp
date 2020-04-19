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
//timestamp::timeStamp_ms stamp;
double timeStamp_ms;
//stamp.get(timeStamp);


//Initialize settings
namespace bpu=boostParserUtilites;
boost::program_options::variables_map vm;
if(!bpu::readCommandLine(argc, argv,vm)) return 0;
//Assign values
std::string basePath = vm["BASE_PATH"].as<std::string>();
std::string trajectoryFile;     bpu::assign(vm,trajectoryFile,"STREAM_DATA_FILE");
std::vector<int> xyz_cols;      bpu::assign(vm,xyz_cols,"XYZ_COLUMS");
int xCol = xyz_cols[0]; int yCol = xyz_cols[1]; int zCol = xyz_cols[2];
std::vector<int> rpy_cols;      bpu::assign(vm,rpy_cols,"Y_P_R_COLUMNS");
int yawCol = rpy_cols[0];int pitchCol = rpy_cols[1];int rollCol = rpy_cols[2];
int timeCol = vm["TIMESTAMP_COL"].as<int>();
int distCol = vm["DIST_COLUMN"].as<int>();
cv::Mat_<float> K;              bpu::assign(vm,K,"K_MAT");
float yawOffset = vm["YAW_OFFSET"].as<float>();
std::string baseScene = vm["BASESCENE"].as<std::string>();
float baseSceneWidth = vm["BASESCENE_WIDTH"].as<float>();
int boxWidth = vm["CHESSBASE_BOXWIDTH"].as<int>();
int rowsOfBoxes = vm["CHESSBASE_ROWS"].as<int>();
int colsOfBoxes = vm["CHESSBASE_COLS"].as<int>();
bool log;                       bpu::assign(vm,log,"LOG");
bool step;                      bpu::assign(vm,step,"STEP");




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
std::string pathToDir = basePath;
std::string newDir = "images";
imagebin.init(pathToDir,newDir);

robustPositioning::dataStreamer getData(basePath+trajectoryFile);

simulatePose warper;
cv::Mat floor;
//cv::Mat floor = cv::imread("spike/test2.png",cv::IMREAD_REDUCED_COLOR_4);
if(baseScene == ""){// If no base scene defined go with default
    warper.setBaseScene(boxWidth,rowsOfBoxes,colsOfBoxes);
}else{
    cv::Mat floor = cv::imread(basePath+baseScene);
    if(floor.empty()){std::cout << "Could not read base image" << std::endl; return 0;}
    //cv::Mat floor8U;
    //cv::cvtColor(floor, floor8U, cv::COLOR_BGR2GRAY);
    warper.setBaseScene(floor);
}

warper.setBaseSceneWidth(baseSceneWidth);
warper.setKMat(K);
warper.setYawOffset(yawOffset);
warper.init();//Initialize with configuration 0


//Go through whole path
std::vector<float> data;
    while(getData.get(data)){
        //Get data
        std::vector<float> trueCoordinate{data[xCol],data[yCol],data[zCol]};
        std::vector<float> angles{data[yawCol],data[pitchCol],data[rollCol]};
    //Get new image
        cv::Mat rawFrame = warper.simulateView(angles,trueCoordinate);
        //cv::Mat rawFrame = warper.uav2BasePose(angles,trueCoordinate);
        cv::Mat frame;
        cv::cvtColor(rawFrame, frame, cv::COLOR_BGR2GRAY);
        cv::imshow("showit",rawFrame);

        //Dump image
        if(log){
            timeStamp_ms = (double)(data[timeCol]*1000);
            imagebin.dump(timeStamp_ms,frame);
            std::cout << "timestamp: " <<timeStamp_ms << std::endl;
        }
        if(step){
            cv::waitKey(0);
        }
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
        ("LOG,l",    po::value<std::string>(),   "Log dataset? <YES/NO>")
        ("STEP,l",    po::value<std::string>(),   "Wait for keypress after every image? <YES/NO>")
    ;
    po::options_description settings("Program settings");
    settings.add_options()
        ("STREAM_DATA_FILE",    po::value<std::string>(),   "Path to data file from config file path")
        ("XYZ_COLUMS",          po::value<std::string>(),   "Specifies [<X_COL>,<Y_COL>,<Z_COL>] as matlab style vector (0-indexed)")
        ("TIMESTAMP_COL",      po::value<int>(),           "Specifies which column of csv file that contains timestamp data (0-indexed)")
        ("DIST_COLUMN",         po::value<int>(),           "Specifies which column of csv file that contains distance (lidar) data (0-indexed)")
        ("Y_P_R_COLUMNS",       po::value<std::string>(),   "Specifies which columns of csv file that contains [yaw,pitch,col] data (0-indexed)")
        ("BASESCENE",           po::value<std::string>()->default_value(""),   "Path to the base scene image that is to be warped. Defaults to chess board")
        ("BASESCENE_WIDTH",     po::value<float>(),        "Physical width of base scene in meter (x-dir)")
        ("K_MAT",               po::value<std::string>(),                "Camera K matrix specified in matlab style. ',' as column separator and ';' as row separator")
        ("YAW_OFFSET",          po::value<float>()->default_value(0),"Relative yaw between image coordinate system and UAV. -pi/2 for UAV x axis == neg image y axis")
        ("CHESSBASE_BOXWIDTH",     po::value<int>()->default_value(11),     "Size of each chessbox if not custom --BASESCENE is used")
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


    return 1;
}
