//#include <cstdio>
#include <iostream>
#include <fstream>
#include <map>
#include <typeinfo>
#include <vector>

#include <string>
#include <iterator>

#include <regex>

#include <opencv2/opencv.hpp>


namespace set{
        const int TYPE_INT = 0;
        const int TYPE_FLOAT = 1;
        const int TYPE_STRING = 2;

/*
    Use a struct like this to access all settings later?
*/
struct dataStruct{
    cv::Mat_<float> K;
    cv::Mat_<float> T;
    std::string anchorPath;
    float x0;
    float y0;
    float z0;
    float yaw0;
    std::string imageStreamBasePath;//Base path to image directory (ending with / if not pwd)
    std::string imageStreamInfoFile;//path to image info file to be appended to base path
    std::string dataStreamFile;//Path to csv fata file to be streamed
};
/* This is a class containing all necessary settings and their default values.
*/

    class settings{
        public:
            settings(int, char**);//Constructor that sets all default  setting values
            //std::vector<std::string> settingKeys;                         //Vector of all available setting keys
            dataStruct data;
            bool success(void);
        private:
            std::map<std::string, int> settingsTYPE;            //Map stating the type of each key (0:int,1:float,2:string)
            std::map<std::string, std::string> settingsDescription;
            std::map<std::string, int> settingsI;
            std::map<std::string, float> settingsF;
            std::map<std::string, std::string> settingsS;
            std::map<std::string, std::string> flags;


            int setAllDefault(void);
            int readArguments(int, char**);
            int initFlags(void);
            int setDefault(std::string,int,         std::string); //Method for setting a setting
            int setDefault(std::string,float,       std::string);
            int setDefault(std::string,std::string, std::string);
            int set(std::string, std::string); //Used to set new settings.
            int writeDefaultSettingsFile(std::string);
            int readSettingsFile(std::string);//If it can not be read then write default and say where it has been written
            std::vector<std::string> parseRow(std::string);
            bool success_;//Check if settings are read correctly.
            bool useDefaultSettings;// = true;
            std::string pathToSettings;
            int streamMode;
            std::string pathToDataSet;
            bool log;
            std::string pathTologDirectory;
            std::string pathToOutputFile;
            bool settingsPathGiven;
            bool constructSettingsStruct(void);
    };

}
bool set::settings::success(void){
    return success_;
}
/*Constructor. Initializes the setting-maps*/
set::settings::settings(int argc, char** argv){
    success_ = true;
    /*DEFAULT SETTINGS*/
    setAllDefault();
    initFlags();
    /*Check input*/
    readArguments(argc,argv);
    if(useDefaultSettings == false){
        if(settingsPathGiven == true){
            std::cout << "Reading settings file from \"" << pathToSettings << "\"... " << std::endl;
            //if(!readSettingsFile(argv[1])){
            if(!readSettingsFile(pathToSettings)){
                success_=false;
                std::cout << "Failed. \n\tCould not open provided settings-file: \"" << pathToSettings << "\". Please give valid path." << std::endl;
            }else{
                std::cout << "Done." << std::endl;
            }
        } else{
            std::cout << "Can not set settings. use flag -d to use default settings." << std::endl;;
            success_=false;
        }
    } else if(settingsPathGiven == true){
        std::cout << "Writing default settings file to \"" << pathToSettings <<"\" ...";
        if(writeDefaultSettingsFile(pathToSettings)){
            std::cout << "Done.";
        }else{
            std::cout << "Failed.";
            success_=false;
        }
        std::cout << std::endl;
    } else{
        std::cout << "Writing default settings file to settings.txt ...";
        if(writeDefaultSettingsFile("settings.txt")){
            std::cout << "Done.";
        }else{
            std::cout << "Failed.";
            success_=false;
        }
        std::cout << std::endl;
    }


    //Set all the settings to the struct where they are accessible. If we are successful up until now
    if(success_){
        success_ = constructSettingsStruct();
    }
}
int set::settings::readArguments(int argc, char** argv){
    if(argc==1){
        std::cout << "No arguments given. " << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout <<  "-flag\t<argument>     Decription\n--------------------------------" << std::endl;
        std::map<std::string, std::string>::iterator it;
        for ( it = flags.begin(); it != flags.end(); it++ )
        {
            std::cout << it->first <<"\t" << it->second << std::endl;
        }
        std::cout << "--------------------------------" << std::endl;

    }else{
        for(int i=1;i<argc;i++){
            std::string flag = argv[i];
            std::string arg;
            if((i+1)<argc){ arg = argv[i+1];}
            if(flag == "-d"){
                useDefaultSettings = true;
            } else if(flag == "-p"){
                if((i+1)<argc && !std::regex_match(arg,std::regex("(-)(.*)")) && std::regex_match(arg,std::regex("(.*)(.txt)"))) { //if there is another argument, that does not begin with "-" and that ends with .txt
                    pathToSettings =  arg;
                    settingsPathGiven = true;
                    i++;
                } else{
                    std::cout << "No valid path for flag -p is given" << std::endl;
                    std::cout << "Given path: \"" << arg << "\""<<std::endl;
                }
            } else if(flag == "-rpi"){
                streamMode = 2;
            } else if(flag == "-dset"){
                if((i+1)<argc && !std::regex_match(arg,std::regex("(-)(.*)")) && std::regex_match(arg,std::regex("(.*)(.csv)"))){ //if there is another argument, that does not begin with "-" and that ends with .txt
                    streamMode = 3;
                    pathToDataSet = arg;
                    i++;
                } else{
                    std::cout << "No valid path for flag -dset is given" << std::endl;
                }

            } else if(flag == "-usb"){
                streamMode = 1;

            } else if(flag == "-log"){
                if((i+1)<argc && !std::regex_match(arg,std::regex("(-)(.*)")) && std::regex_match(arg,std::regex("(.*)(/)"))){ //if there is another argument, that does not begin with "-" and that ends with /
                    pathTologDirectory = arg;
                    i++;
                } else{
                    std::cout << "No valid path for flag log directory is given. Must end with '/' " << std::endl;
                }

            } else if(flag == "-out"){
                if((i+1)<argc && !std::regex_match(arg,std::regex("(-)(.*)"))){//if there is another argument, that does not begin with "-"
                    if(std::regex_match(arg,std::regex("(.*)(/)"))){//If argument is directory
                        pathToOutputFile = arg + "output.csv";
                        std::cout << "output is given as directory. Defaults to filename output.csv" << std::endl;
                        i++;
                        continue;
                    }else if(std::regex_match(arg,std::regex("(.*)(.csv)"))){//If argument is file
                        pathToOutputFile = arg;
                        i++;
                        continue;
                    }
                }
                std::cout << "No valid path for flag output file/directory is given. Must end with '/' for directory, .csv for file." << std::endl;
            } else{
                std::cout << "argument \"" << arg<< "\" not known.";
            }
        }


    }
    return 1;
}
int set::settings::initFlags(void){
    flags.insert ( std::pair<std::string,std::string>("-d",         "<No argument>  Use default settings and write file to settings.txt if not -p is set") );
    flags.insert ( std::pair<std::string,std::string>("-p",         "<Path to file> Path to settings file. If -d is set, file is written, otherwise read") );
    flags.insert ( std::pair<std::string,std::string>("-rpi",       "<No argument>  Use rpi cam and i2c as input") );
    flags.insert ( std::pair<std::string,std::string>("-dset",      "<Path to set>  Use dataset as input") );
    flags.insert ( std::pair<std::string,std::string>("-usb",       "<No argument>  Use usb cam 0 as input. No acc/gyro data.") );
    flags.insert ( std::pair<std::string,std::string>("-log",       "<Path to dir>  Log collected data") );
    flags.insert ( std::pair<std::string,std::string>("-out",       "<Path file/dir>Output file. Path to dir to auto name, path to file to set name") );
    return 1;
}
int set::settings::setAllDefault(void){
    useDefaultSettings = false;
    settingsPathGiven = false;
    std::string pathToSettings = "";
    //int streamMode;
    std::string pathToDataSet = "";
    log = false;
    std::string pathTologDirectory = "";
    std::string pathToOutputFile = "";

    /*DEFAULT SETTINGS*/
    //Obtain data
    setDefault("USE_CAM_NMBR",(int) 0,"-1: rpi cam, >=0: webcam nmbr");
    //Modes
    setDefault("OPTICAL_FLOW_MODE", (int) 0," KLT with grid or correlation based");
    setDefault("OPTICAL_FLOW_GRID", (int) 0," ");
    setDefault("VISUAL_ODOMETRY_MODE", (int) 0," ");
    setDefault("POS_EST_MODE", (int) 0,"Azipe, VO, benchmark, azipe+VO, azipe+benchmark");
    setDefault("LOG_MODE", (int) 0,"Log position estimation, pos est and video");
    //General values that are needed
    setDefault("FRAME_RESOLUTION_X", (int) 0,"In pixles. Used to specify RPI video input res. Specify here or hardcode?");
    setDefault("FRAME_RESOLUTION_Y", (int) 0," ");
    setDefault("K_MAT_cx", (float) 320,"For image resolution 640 in x dir");
    setDefault("K_MAT_cy", (float) 240,"For image resolution 480 in y dir");
    setDefault("K_MAT_fx", (float) 607.13635578,"For image resolution 640 in x dir");
    setDefault("K_MAT_fy", (float) 607.13635578,"For image resolution 480 in y dir");
    setDefault("T_MAT_1_1", (float) 0," ");
    setDefault("T_MAT_1_2", (float) 0," ");
    setDefault("T_MAT_1_3", (float) 0," ");
    setDefault("T_MAT_2_1", (float) 0," ");
    setDefault("T_MAT_2_2", (float) 0," ");
    setDefault("T_MAT_2_3", (float) 0," ");
    setDefault("T_MAT_3_1", (float) 0," ");
    setDefault("T_MAT_3_2", (float) 0," ");
    setDefault("T_MAT_3_3", (float) 0," ");
    setDefault("DIST_COEFF_K1", (float) 0.2486857357354474,"Barrel distortion coefficient k1");
    setDefault("DIST_COEFF_K2", (float) -1.452670730319596,"Barrel distortion coefficient k2");
    setDefault("DIST_COEFF_K3", (float) 2.638858641887943,"Barrel distortion coefficient k3");

    setDefault("PATH_TO_ARUCO_DATABASE", "database.txt" ," ");
    setDefault("ARUCO_DICT_TYPE", (int) 0," ");
    setDefault("MAX_ID_ARUCO", (int) 0,"Database size. must be able to contain all IDs in ARUCO_DICT_TYPE");
    setDefault("ROI_SIZE", (int) 0,"Specify the size in pixles of the side of the centered ROI that is considered in VO. Used to edit K mat of VO alg.");

    setDefault("INITIAL_X", (float) 0," ");
    setDefault("INITIAL_Y", (float) 0," ");
    setDefault("INITIAL_Z", (float) 0," ");
    setDefault("INITIAL_YAW", (float) 0," ");

    setDefault("STREAM_IMAGES_BASEPATH", "Generated-dataSets/5_jul/","Base path from program working directory to image directory, ending with /");
    setDefault("STREAM_IMAGES_INFO_FILE", "data.csv","File (located in IMAGES_BASEPATH with image data. (timestamps, image name))");
    setDefault("STREAM_DATA_FILE", "Generated-dataSets/5_jul/truePath.csv","Path to csv file to be streamed");
    return 1;
}
int set::settings::setDefault(std::string key, int          value, std::string description){
    settingsTYPE[key] = TYPE_INT;
    settingsDescription[key] = description;
    settingsI[key] = value;
    return 1;
}
int set::settings::setDefault(std::string key, float        value, std::string description){
    settingsTYPE[key] = TYPE_FLOAT;
    settingsDescription[key] = description;
    settingsF[key] = value;
    return 1;
}
int set::settings::setDefault(std::string key, std::string  value, std::string description){
    settingsTYPE[key] = TYPE_STRING;
    settingsDescription[key] = description;
    settingsS[key] = value;
    return 1;
}
/* This function takes the the string that is read from settings-file and converts it to suitable type and replaces default*/
int set::settings::set(std::string key, std::string value){
    //See if the key is valid
    std::map<std::string, int>::iterator it;
    it = settingsTYPE.find(key);
    if (it != settingsTYPE.end()){
        int type = it->second;
        switch (type) {
            case TYPE_INT:{
                try{
                    int value_int = std::stoi(value);
                    settingsI[key] = value_int;
                    //std::cout << "Set setting  << key << " to " << value_int << std::endl;
                }catch(const std::invalid_argument& e){
                    std::cout << "Could not convert \"" << value <<"\" to int for key \"" << key << "\"" << std::endl;
                    return 0;
                }
                break;
            }
            case TYPE_FLOAT:{
                try{
                    float value_float = std::stof(value);
                    settingsF[key] = value_float;
                    //std::cout << "Set setting  << key << " to " << value_float << std::endl;
                }catch(const std::invalid_argument& e){
                    std::cout << "Could not convert \"" << value <<"\" to float for key \"" << key << "\"" << std::endl;
                    return 0;
                }
                break;
            }
            case TYPE_STRING:{
                settingsS[key] = value;
                //std::cout << "Set setting  << key << " to " << value << std::endl;
                break;
            }
        }
    }else{
        std::cout << "Setting key \"" << key << "\" is not valid." << std::endl;
        return 0;
    }
    return 1;
}
/* Reads all default settings and writes them into a file settings.txt
 */
int set::settings::writeDefaultSettingsFile(std::string path){
    std::ofstream settingsFile;
    settingsFile.open(path, std::ofstream::out | std::ofstream::trunc);//Trunc option clears the file before anything is written
    std::map<std::string, int>::iterator it;
    for ( it = settingsTYPE.begin(); it != settingsTYPE.end(); it++ )
    {
        std::string row = "";//The row that is to be appended to file
        std::string key = it->first;
        int type = it->second;
        row += (key + ", ");
        if(type==0){
            std::string x = std::to_string(settingsI.at(key));
            row += (x + ", ");
        } else if(type==1){
            std::string x = std::to_string(settingsF.at(key));
            row += (x + ", ");
    } else if(type==2){
            row += (settingsS.at(key) + ", ");
        } else{
            std::cout << "Default type of key: " << key << " is not valid." << std::endl;
            return 0;
        }
        row += (settingsDescription[key] + '\n');
        settingsFile << row;
    }
    settingsFile.close();
    return 1;
}
int set::settings::readSettingsFile(std::string path){
    std::string line;
    std::string delim = ",";
    std::ifstream file;
    file.open(path);
    if(file.is_open()){
         while(getline(file,line)){
            std::vector<std::string> parsed = parseRow(line);
            if((parsed.size()==3 || parsed.size()==2) && line.at(0)!='#'){//Disregard any lines that are not 2 or 3 elements long, or start with #
                set(parsed[0],parsed[1]);//Set the setting (or try at least)
            }
         }
    }else{return 0;}
    file.close();
    return 1;
}
//This function takes a line and parses it into a vector<string> using "," as deliminator and disregarding LEADING whitespaces
std::vector<std::string> set::settings::parseRow(std::string line){
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

//This method reads all the internal values and constructs a struct where the properly
//  formatted settings are available from another scope
bool set::settings::constructSettingsStruct(void){
    data.K = cv::Mat_<float>::ones(3,3);
        data.K(0,0) = settingsF["K_MAT_fx"];
        data.K(1,1) = settingsF["K_MAT_fy"];
        data.K(0,2) = settingsF["K_MAT_cx"];
        data.K(1,2) = settingsF["K_MAT_cy"];
    data.T = cv::Mat_<float>::zeros(3,3);
        data.T(0,0) = settingsF["T_MAT_1_1"];   data.T(0,1) = settingsF["T_MAT_1_2"];   data.T(0,2) = settingsF["T_MAT_1_3"];
        data.T(1,0) = settingsF["T_MAT_2_1"];   data.T(1,1) = settingsF["T_MAT_2_2"];   data.T(1,2) = settingsF["T_MAT_2_3"];
        data.T(2,0) = settingsF["T_MAT_3_1"];   data.T(2,1) = settingsF["T_MAT_3_2"];   data.T(2,2) = settingsF["T_MAT_3_3"];
    data.x0 = settingsF["INITIAL_X"];
    data.y0 = settingsF["INITIAL_Y"];
    data.z0 = settingsF["INITIAL_Z"];
    data.yaw0 = settingsF["INITIAL_YAW"];
    data.anchorPath = settingsS["PATH_TO_ARUCO_DATABASE"];
    data.imageStreamBasePath = settingsS["STREAM_IMAGES_BASEPATH"];
    data.imageStreamInfoFile = settingsS["STREAM_IMAGES_INFO_FILE"];
    data.dataStreamFile = settingsS["STREAM_DATA_FILE"];
    return true;
}

/*
Square_size: 26.2 (Expressed in mm). It is the same if we set square size 0.0262 (expressed in meter)
Resolution:
2592*1944
Camera matrix: K= [2458.902240893194, 0, 1296;
 0, 2458.902240893194, 972;
 0, 0, 1]
-> 640*480
Camera matrix: K= [607.13635578, 0, 320;
 0, 607.13635578, 240;
 0, 0, 1]
Distortion coefficients: [0.2486857357354474;
 -1.452670730319596;
 0;
 0;
 2.638858641887943]
*/
