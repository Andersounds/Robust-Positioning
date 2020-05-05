#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
//#include <chrono> //For timestamps
#include "../src/vopos.hpp"
#include "../src/simulatePose.hpp"
//#include "../src/save2file.cpp"
#include "../src/videoStream.hpp"
#include "../src/dataStream.hpp"
#include "../src/boostParserUtilities.cpp" //Includes boost and two utility functions

#include "../src/logger.hpp"
#include "../src/timeStamp.cpp"
#define PI 3.1416


#include <typeinfo>

//
const int ALG_AZIPE = 0;
const int ALG_VO = 1;
const int ALG_MARTON = 2;

int main(int argc, char** argv){
timestamp::timeStamp_ms stamp;
//Can these two rows be written as stamp.get(double timeStamp); is timeStamp available in this scope then?
double timeStamp;
stamp.get(timeStamp);



//Initialize settings
namespace bpu=boostParserUtilites;
boost::program_options::variables_map vm;
if(!boostParserUtilites::readCommandLine(argc, argv,vm)) return 0;
std::string basePath;   bpu::assign(vm,basePath,"BASE_PATH");
//Initialize video stream
std::string imageInfoFile;  bpu::assign(vm,imageInfoFile,"STREAM_IMAGES_INFO_FILE");
std::string imageBase;
std::string imageInfo;
boostParserUtilites::basePathFromFilePath(imageInfoFile,imageBase,imageInfo);
robustPositioning::Streamer VStreamer(basePath+imageBase,basePath+imageBase+imageInfo);
cv::Mat frame, colorFrame;
//Initialize data stream
std::string dataFile;   bpu::assign(vm,dataFile,"STREAM_DATA_FILE");
int skipLines = 1;
robustPositioning::dataStreamer getData(basePath + dataFile,skipLines);
std::vector<float> data;
//Initialize data logger
robustPositioning::dataLogger databin_LOG;
std::string outFile;
if(vm["OUT_TO_PWD"].as<std::string>()=="YES"){
    outFile = vm["OUT"].as<std::string>();
}else{
    outFile = vm["BASE_PATH"].as<std::string>() + vm["OUT"].as<std::string>();
}
const int log = vm["LOG"].as<int>();
if(log){
    std::cout << "Writing output file to " << outFile << std::endl;
    if(!databin_LOG.init(outFile,std::vector<std::string>{"timestamp [ms]","X [m]","Y [m]","Z [m]","Roll [rad]","Pitch [rad]","Yaw [rad]","Known anchors []","Mode"})) return 0;
}else{
    std::cout << "Will not log values" << std::endl;
}
const int step = vm["STEP"].as<int>();
bool derotate = vm["DEROTATE_OF"].as<bool>();
bool useRollPitch = vm["USE_ROLLPITCH"].as<bool>();


//Initialize positioning object
int maxIdAruco = 50;
std::string anchorPath;     bpu::assign(vm,anchorPath,"PATH_TO_ARUCO_DATABASE");
int flowGrid;               bpu::assign(vm, flowGrid,"OPTICAL_FLOW_GRID");
float roi_side;             bpu::assign(vm, roi_side,"ROI_SIZE");
float roi_x = (vm["RES_X"].as<float>()-roi_side)/2;//assign manually as we need a calculation
float roi_y = (vm["RES_Y"].as<float>()-roi_side)/2;
cv::Rect2f roiSize = cv::Rect2f(roi_x,roi_y,roi_side,roi_side);;
cv::Mat_<float> K;          bpu::assign(vm,K,"K_MAT");
cv::Mat_<float> T;          bpu::assign(vm,T,"T_MAT");

int marton_buffsize;                 bpu::assign(vm,marton_buffsize,"MARTON_BUFFERSIZE");
/*
    Define execution modes
*/
int of_mode;    std::string of_mode_str;    bpu::assign(vm,of_mode_str,"OF_MODE");
int vo_mode;    std::string vo_mode_str;    bpu::assign(vm,vo_mode_str,"VO_MODE");
int pos_alg;   std::string pos_alg_str;   bpu::assign(vm,pos_alg_str,"POS_ALG");
/*--------Optical Flow--------*/
if(of_mode_str=="KLT"){
    std::cout << "Starting positioning object using KLT based optical flow." << std::endl;
    of_mode = pos::OF_MODE_KLT;}
else if(of_mode_str=="CORR"){
    std::cout << "Starting positioning object using Correlation based optical flow." << std::endl;
    of_mode = pos::OF_MODE_CORR;}
else{std::cout << "Unknown --OF_MODE argument" << std::endl;return 0;}
/*--------Visual Odometry----------*/
if(vo_mode_str=="HOMOGRAPHY"){
    std::cout << "Starting positioning object using Homography based Visual odometry." << std::endl;
    vo_mode = pos::VO_MODE_HOMOGRAPHY;}
else if(vo_mode_str=="AFFINE"){
    std::cout << "Starting positioning object using Affine based Visual odometry." << std::endl;
    vo_mode = pos::VO_MODE_AFFINE;}
else{std::cout << "Unknown --VO_MODE argument" << std::endl;return 0;}
/*-------Positioning algorithm----------*/
if(pos_alg_str=="AZIPE"){
    std::cout << "Starting positioning object using Azipe mode" << std::endl;
    pos_alg = ALG_AZIPE;}
else if(pos_alg_str == "VO"){
    std::cout << "Starting positioning object using Visual Odometry as fallback" << std::endl;
    pos_alg = ALG_VO;
}
else if(pos_alg_str == "MARTON"){
    std::cout << "Starting positioning object using MARTON as fallback" << std::endl;
    pos_alg = ALG_MARTON;
}
else{std::cout << "Unknown --POS_ALG argument" << std::endl;return 0;}

pos::positioning P(of_mode,
                    vo_mode,
                    cv::aruco::DICT_4X4_50,
                    maxIdAruco,basePath+anchorPath,flowGrid,roiSize,K,T);
P.activateDerotation = derotate;
//Init values of position and yaw
cv::Mat_<float> t;          bpu::assign(vm,t,"XYZ_INIT");
float yaw;                  bpu::assign(vm, yaw,"YAW_INIT");
float nmbrOfAnchors = 0;



//Read data until done
float timeStamp_data;
float timeStamp_image;
int counter = 0;
double timeStamp_start;
stamp.get(timeStamp_start);
float rad2Grad = 57.2958;

int distColumn;                 bpu::assign(vm,distColumn,"DIST_COLUMN");
int pitchColumn;                bpu::assign(vm,pitchColumn,"PITCH_COLUMN");
int rollColumn;                 bpu::assign(vm,rollColumn,"ROLL_COLUMN");

int algmode = pos::MODE_AZIPE_AND_FALLBACK;
//marton::circBuff buffer(3);
//marton::circBuff2 buffer2(5);
    while(getData.get(data)){

        timeStamp_data = data[0];
        float dist = data[distColumn];//This is used as a subst as actual height is not in dataset
        float pitch = 0;
        float roll = 0;
        if(useRollPitch){
            float pitch = data[pitchColumn];
            float roll = data[rollColumn];
        }
        if((counter%22)<11){algmode = pos::MODE_AZIPE_AND_FALLBACK;}
        else{algmode = pos::MODE_FALLBACK;}

//Get new image
//Separate processAndIllustrate with just process using switch statement
        if(VStreamer.peek()<=timeStamp_data){
            VStreamer.getImage(frame);
            if(frame.empty()){std::cout << "Video stream done."<< std::endl; return 0;}
            cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
            std::cout << "Lap " << counter  << ", time: " << timeStamp_data<< std::endl;
            //int mode = P.processAndIllustrate(pos_alg,frame,colorFrame,pos::ILLUSTRATE_ALL,dist,roll,pitch,yaw,t,nmbrOfAnchors);
            //std::cout << "SWITCH:::::::::" << std::endl;
            switch(pos_alg){
                case ALG_AZIPE:{
                    std::cout << "ALG_AZIPE:::" << std::endl;
                    pos::argStruct arguments = {dist,roll,pitch,yaw};
                    int mode = P.process_AZIPE(frame, colorFrame,t,arguments);
                    if(log){
                        std::vector<float> logData{timeStamp_data,t(0,0)/100,t(1,0)/100,t(2,0)/100,arguments.roll,arguments.pitch,arguments.yaw,nmbrOfAnchors,(float)mode};
                        databin_LOG.dump(logData);
                    }
                    yaw = arguments.yaw;
                    break;
                }
                case ALG_VO:{
                    std::cout << "ALG_VO:::" << std::endl;
                    pos::VOargStruct arguments = {dist*100,roll,pitch,yaw};//Scale dist measurement to cm
                    int mode = P.process_VO_Fallback(algmode,frame, colorFrame, t,arguments);
                    yaw = arguments.yaw;
                    if(log){
                        std::vector<float> logData{timeStamp_data,t(0,0)/100,t(1,0)/100,t(2,0)/100,arguments.roll,arguments.pitch,arguments.yaw,nmbrOfAnchors,(float)mode};
                        databin_LOG.dump(logData);
                    }
                    break;
                }
                case ALG_MARTON:{
                    std::cout << "ALG_MARTON:::" << std::endl;
                    pos::MartonArgStruct arguments = {roll,pitch,yaw,timeStamp_data, marton_buffsize};
                    int mode = P.process_Marton_Fallback(algmode,frame, colorFrame, t,arguments);
                    yaw = arguments.yaw;
                    if(log){
                        std::vector<float> logData{timeStamp_data,t(0,0)/100,t(1,0)/100,t(2,0)/100,arguments.roll,arguments.pitch,arguments.yaw,nmbrOfAnchors,(float)mode};
                        databin_LOG.dump(logData);
                    }
                    if((mode!=pos::RETURN_MODE_AZIPE_FAILED)&(mode!=pos::RETURN_MODE_MARTON_FAILED)){
                    //    buffer.add(t,arguments.yaw,arguments.time);//Dont add if positioning failed
                    //    buffer2.add(arguments.time); //test also with counter directly if not working
                    //    std::cout << "Added " << arguments.time << " to buffer2" << std::endl;
                    }
                    //float toff = buffer.read_T_offset();
                    //std::cout << "Marton T offset: " << toff<< std::endl;
                    //double tX[3];
                    //buffer.read_t(tX);
                    //std::vector<float> tX(5);
                    //buffer2.read(tX);
                    //std::cout <<" T: [" << tX[0] << ", " << tX[1]<<", " << tX[2] << ", " << tX[3] << ", " << tX[4] <<"]" << std::endl;


                    break;
                }
            }



            //int mode = P.processAz(pos::MODE_AZIPE,frame,colorFrame,pos::ILLUSTRATE_ALL,dist,roll,pitch,yaw,t,nmbrOfAnchors);
            //Log data
//            if(true){
//                std::vector<float> logData{timeStamp_data,t(0,0),t(1,0),t(2,0),arguments.roll,arguments.pitch,arguments.yaw,nmbrOfAnchors,(float)mode};
//                databin_LOG.dump(logData);
//            }
            //std::cout << "Main:   X: "<< t(0,0) << ", Y: "<< t(1,0) << ", Z: " << t(2,0) <<", roll: " << roll<<", pitch: " << pitch << "yaw: " << yaw<< std::endl;
            P.illustrateYaw(colorFrame,yaw);
            cv::imshow("showit",colorFrame);
            if(step){
                cv::waitKey(0);
            }
            if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}

            counter++;
            std::cout << ":::::::::::::::" << std::endl;
      }


    //std::cout << "t: " << t.t() << std::endl;
        //Maybe convert to color for illustartion?
        //cv::cvtColor(colorFrame, frame, cv::COLOR_BGR2GRAY);

        //std::cout << "Mode: " << mode << std::endl;

    /*    //Write to file
        stamp.get(timeStamp);
        std::vector<float> truePath{(float)timeStamp, xPath[i],yPath[i],zPath[i],yawPath[i],pitchPath[i],rollPath[i]};
        databin_LOG.dump(truePath);
        imagebin.dump(timeStamp,frame);

*/
    //    std::vector<float> estimation{(float)timeStamp,t(0,0),t(1,0),t(2,0),yaw,(float)mode};
    //    databin_EST.dump(estimation);

//if(i>100) return 0;
        //std::string str = std::to_string(i);
        //std::cout << str << std::endl;
        //cv::putText(colorFrame,str,cv::Point(10,colorFrame.rows/2),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(118, 185, 0),2);


    }
    double timeStamp_end;
    stamp.get(timeStamp_end);
    double tot_time_s = (timeStamp_end-timeStamp_start)/1000;
    double fps = ((double)counter)/tot_time_s;
    std::cout << "Processed " << counter << "images in " << tot_time_s << " s. (" << fps << " fps)" << std::endl;
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
     ;

     po::options_description parameters("UAV properties");
     parameters.add_options()
         //Parameters
         ("RES_X",  po::value<float>(), "Camera resolution in X direction")
         ("RES_Y",  po::value<float>(), "Camera resolution in Y direction")
         ("K_MAT",  po::value<std::string>(), "Camera K matrix specified in matlab style. ',' as column separator and ';' as row separator") //Tänk om man kan definiera denna direkt som en opencv mat och ge 9 argument på rad?
         ("T_MAT",  po::value<std::string>()->default_value("[0,-1,0;1,0,0;0,0,1]"), "UAV-to-Camera matrix. Default +90deg. Specified in matlab style")
         ("CAMERA_BARREL_DISTORTION",    po::value<std::string>()->default_value("[0.2486857357354474,-1.452670730319596,2.638858641887943]"), "Barrel distortion coefficients given as [K1,K2,K3]")
         ;

     po::options_description initValues("Initial values");
     initValues.add_options()
         ("XYZ_INIT",                    po::value<std::string>()->default_value("[0;0;-1.8]"), "Initial position expressed as [X,Y,Z] coordinate floats")
         ("ROLL_INIT", po::value<float>()->default_value(0),"Initial roll of UAV, radians")
         ("PITCH_INIT", po::value<float>()->default_value(0),"Initial pitch of UAV, radians")
         ("YAW_INIT", po::value<float>()->default_value(0),"Initial yaw of UAV, radians")
         ;
     po::options_description modes("Program settings");
     modes.add_options()
         ("OUT,o",   po::value<std::string>()->default_value("log.csv"), "Write output data to specified file.")// Single string argument
         ("OUT_TO_PWD",   po::value<std::string>()->default_value("YES"), "Write output file pwd instead of to BASEPATH (NO or YES)")// Single string argument
         ("LOG",   po::value<int>()->default_value(0), "Output log file 0 (NO) / 1 (YES)")
         ("STEP",   po::value<int>()->default_value(0), "Wait for keypress between each image? 0 (NO) / 1 (YES)")
         ("DIST_COLUMN", po::value<int>(),  "Specifies which column of csv file that contains distance (lidar) data")
         ("ROLL_COLUMN", po::value<int>(),  "Specifies which column of csv file that contains roll data")
         ("PITCH_COLUMN", po::value<int>(),  "Specifies which column of csv file that contains pitch data")
         ("PATH_TO_ARUCO_DATABASE", po::value<std::string>()->default_value("anchors.csv"),"Path to anchor database from base path")
         ("STREAM_IMAGES_INFO_FILE",po::value<std::string>(),"Path to images info file from config file path")
         ("STREAM_DATA_FILE",po::value<std::string>(),"Path to data file from config file path")
         ("POS_ALG",po::value<std::string>(),"Positioning algorithm | AZIPE or VO or MARTON")
         ("USE_ROLLPITCH",   po::value<bool>()->default_value(true), "Use roll/pitch? otherwise set to 0. true/1 or false/0")
         ;
     po::options_description voSettings("Visual Odometry settings");
     voSettings.add_options()
         ("OF_MODE",po::value<std::string>(),"Mode of Optical flow algorithm. KLT or CORR")
         ("VO_MODE",po::value<std::string>(),"Mode of Visual Odometry algorithm. HOMOGRAPHY or AFFINE")
         ("OPTICAL_FLOW_GRID",           po::value<int>()->default_value(4),"Sqrt of number of optical flow vectors")//Single int
         ("ROI_SIZE",po::value<float>()->default_value(150), "Side length of VO ROI. Used to edit K mat of VO alg.")
         ("DEROTATE_OF",   po::value<bool>()->default_value(true), "Derotate optical flow field using roll/pitch? true/1 or false/0")
         ;
     po::options_description martonSettings("Marton robust settings");
     martonSettings.add_options()
         ("MARTON_BUFFERSIZE", po::value<int>()->default_value(3),  "Number of previous points to align marton polynomial to")
         ;

     // Parse command line
     po::options_description all("All options");
     all.add(generic).add(parameters).add(initValues).add(modes).add(voSettings).add(martonSettings);
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
         std::cout << voSettings << std::endl;
         std::cout << martonSettings << std::endl;
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
        if(!bpu::checkDimOfCVMatOption(vm,"XYZ_INIT",3, 1)){return 0;}
        if(!bpu::checkDimOfCVMatOption(vm,"T_MAT",3, 3)){return 0;}
        if(!bpu::checkDimOfCVMatOption(vm,"K_MAT",3, 3)){return 0;}
        if(!bpu::checkDimOfCVMatOption(vm,"CAMERA_BARREL_DISTORTION",1, 3)){return 0;}


     return 1;
 }
