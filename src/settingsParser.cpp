//#include <cstdio>
#include <iostream>
#include <fstream>
#include <map>
#include <typeinfo>
#include <vector>

#include <string>
#include <iterator>

namespace set{
        const int TYPE_INT = 0;
        const int TYPE_FLOAT = 1;
        const int TYPE_STRING = 2;

/* This is a class containing all necessary settings and their default values.
*/
    class settings{
        public:
            //std::vector<std::string> settingKeys;                         //Vector of all available setting keys
            std::map<std::string, int> settingsTYPE;            //Map stating the type of each key (0:int,1:float,2:string)
            std::map<std::string, std::string> settingsDescription;
            std::map<std::string, int> settingsI;
            std::map<std::string, float> settingsF;
            std::map<std::string, std::string> settingsS;

            settings(int, char**);//Constructor that sets all default  setting values
            int setAllDefault(void);
            int setDefault(std::string,int,         std::string); //Method for setting a setting
            int setDefault(std::string,float,       std::string);
            int setDefault(std::string,std::string, std::string);
            int set(std::string, std::string); //Used to set new settings.
            int writeDefaultSettingsFile(void);
            int readSettingsFile(std::string);//If it can not be read then write default and say where it has been written

            std::vector<std::string> parseRow(std::string);
    };
}
/*Constructor. Initializes the setting-maps*/
set::settings::settings(int argc, char** argv){
    /*DEFAULT SETTINGS*/
    setAllDefault();
    /*Check input*/
    if(argc>2){
        std::cerr << "Invalid arguments to settingsParser. " << std::endl;
        std::cout << "Give 0 arguments to create default settings.txt file and use these settings." << std::endl;
        std::cout << "Give single argument <path/to/settingsFile.txt> to use those settings to override default." << std::endl;
    }else if(argc==2){
        std::cout << "Reading settings file from \"" << argv[1] << "\"... ";
        if(!readSettingsFile(argv[1])){
            std::cout << "\n Could not open provided settings-file: \"" << argv[1] << "\". Please give valid path or no path." << std::endl;
        }else{
            std::cout << "Done." << std::endl;
        }
    }else{
        std::cout << "No path to settings-file provided.";
        std::cout << "Exporting complete file with default values to /settings.txt ...";
        if(writeDefaultSettingsFile()){
            std::cout << "Done.";
        }
        std::cout << std::endl;
    }
}

int set::settings::setAllDefault(void){
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
    setDefault("K_MAT_cx", (float) 0," ");
    setDefault("K_MAT_cy", (float) 0," ");
    setDefault("K_MAT_fx", (float) 0," ");
    setDefault("K_MAT_fy", (float) 0," ");
    setDefault("T_MAT_1_1", (float) 0," ");
    setDefault("T_MAT_1_2", (float) 0," ");
    setDefault("T_MAT_1_3", (float) 0," ");
    setDefault("T_MAT_2_1", (float) 0," ");
    setDefault("T_MAT_2_2", (float) 0," ");
    setDefault("T_MAT_2_3", (float) 0," ");
    setDefault("T_MAT_3_1", (float) 0," ");
    setDefault("T_MAT_3_2", (float) 0," ");
    setDefault("T_MAT_3_3", (float) 0," ");

    setDefault("PATH_TO_ARUCO_DATABASE", "database.txt" ," ");
    setDefault("ARUCO_DICT_TYPE", (int) 0," ");
    setDefault("MAX_ID_ARUCO", (int) 0,"Database size. must be able to contain all IDs in ARUCO_DICT_TYPE");
    setDefault("ROI_SIZE", (int) 0,"Specify the size in pixles of the side of the centered ROI that is considered in VO. Used to edit K mat of VO alg.");

    setDefault("INITIAL_X", (float) 0," ");
    setDefault("INITIAL_Y", (float) 0," ");
    setDefault("INITIAL_Z", (float) 0," ");
    setDefault("INITIAL_YAW", (float) 0," ");

    setDefault("TILT_FILT_A", (float) 0,"");
    setDefault("TILT_FILT_B", (float) 0," ");
    setDefault("TILT_FILT_C", (float) 0," ");
    setDefault("TILT_FILT_D", (float) 0," ");
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
                    std::cout << "Set setting \"" << key << "\" to " << value_int << std::endl;
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
                    std::cout << "Set setting \"" << key << "\" to " << value_float << std::endl;
                }catch(const std::invalid_argument& e){
                    std::cout << "Could not convert \"" << value <<"\" to float for key \"" << key << "\"" << std::endl;
                    return 0;
                }
                break;
            }
            case TYPE_STRING:{
                settingsS[key] = value;
                std::cout << "Set setting \"" << key << "\" to " << value << std::endl;
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
int set::settings::writeDefaultSettingsFile(void){
    std::ofstream settingsFile;
    settingsFile.open ("settings.txt");
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
            if(parsed.size()==3 || parsed.size()==2){//Disregard any lines that are not 2 or 3 elements long
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
