#include <iostream>
#include <opencv2/opencv.hpp>
#include "videoStream.hpp"
#include "../include/raspicam-0.1.6/src/raspicam_cv.h"


/*
 *
 *
 *
 *      =======   Master streamer class constructors and getimage ========
 *
 *
 *
*/

//Master getImage wrapper
float robustPositioning::Streamer::getImage(cv::Mat& frame){
    switch (chosenMode) {
        case robustPositioning::MODE_USB_CAM:{
            return usbcamSTR.getImage(frame);
        }
        case robustPositioning::MODE_RPI_CAM:{
            return picamSTR.getImage(frame);
        }
        case robustPositioning::MODE_IMG_STREAM:{
            return datasetSTR.getImage(frame);
        }
        case robustPositioning::MODE_IMG_TIMESTAMP_STREAM:{
            return datasetSTR.getImage(frame);
        }
    }
    return -1;

}
float robustPositioning::Streamer::peek(void){
    return datasetSTR.peek();
}

//When giving just the single int argument, only rpi videostream or default internal cam stream is possible
robustPositioning::Streamer::Streamer(int choice){
    chosenMode = choice;
    if(choice == robustPositioning::MODE_USB_CAM){
        usbcamSTR = usbCamStreamer(0);
    }else if(choice == robustPositioning::MODE_RPI_CAM){
        chosenMode = robustPositioning::MODE_RPI_CAM;
	picamSTR = piCamStreamer(CV_8UC1);//Default to black white images
    }
    //usbcamSTR = usbCamStreamer(0);
}
//When two int arguments are given, it must be internal cam and the camera choice
robustPositioning::Streamer::Streamer(int choice,int setting){
    chosenMode = choice;
    if(choice == robustPositioning::MODE_USB_CAM){
        usbcamSTR = usbCamStreamer(setting);//choose internal camera number "setting"
    }else if(choice == robustPositioning::MODE_RPI_CAM){
        chosenMode = robustPositioning::MODE_RPI_CAM;
        picamSTR = piCamStreamer(setting);
    }
}
//For choosing dataset. Just reading images with cv cap. i.e. someDir/images/img_%04.png
robustPositioning::Streamer::Streamer(std::string datasetPath){
    chosenMode = robustPositioning::MODE_IMG_STREAM;
    std::cout << "Dataset image streamer chosen with path \"" << datasetPath << "\"." << std::endl;
    datasetSTR = datasetStreamer(datasetPath,"",false);
}
//Same as above but with explicitly stated choice
robustPositioning::Streamer::Streamer(int choice, std::string datasetPath){
    bool useTimeStampFile;
    if(choice == robustPositioning::MODE_IMG_STREAM){
        chosenMode = robustPositioning::MODE_IMG_STREAM;
        std::cout << "Dataset image streamer chosen with path \"" << datasetPath << "\"." << std::endl;
        useTimeStampFile = false;
        datasetSTR = datasetStreamer(datasetPath,"",false);
    }else if(choice == robustPositioning::MODE_IMG_TIMESTAMP_STREAM){
        chosenMode = robustPositioning::MODE_USB_CAM;
        std::cout << "Dataset image with timestamp streamer chosen. but no path to datafile given\"" << std::endl;
        useTimeStampFile = false;
        usbcamSTR = usbCamStreamer(0);
    } else{
        std::cout << "No valid choice given. To stream dataset choose either " << robustPositioning::MODE_IMG_STREAM << " or " << robustPositioning::MODE_IMG_TIMESTAMP_STREAM << std::endl;
        std::cout << "Defaulting to internal camera 0" << std::endl;
        chosenMode = robustPositioning::MODE_USB_CAM;
        usbcamSTR = usbCamStreamer(0);
    }
}

//For reading a datafile from a csv file that specifies timestamps and filenames
robustPositioning::Streamer::Streamer(std::string basePath, std::string dataFile){
    chosenMode = robustPositioning::MODE_IMG_TIMESTAMP_STREAM;
//    std::cout << "Initializing dataset video streamer with timestamps..." << std::endl;
    datasetSTR = datasetStreamer(basePath, dataFile, true);

}
//overloaded. choice is not considered
robustPositioning::Streamer::Streamer(int choice, std::string basePath, std::string dataFile){
    chosenMode = robustPositioning::MODE_IMG_TIMESTAMP_STREAM;
//    std::cout << "Initializing dataset video streamer with timestamps..." << std::endl;
    datasetSTR = datasetStreamer(basePath, dataFile, true);
}


//Default constructor. does not do anything. onl called when Streamer does NOT use usb cam
robustPositioning::usbCamStreamer::usbCamStreamer(void){

}
/*
 *Constructor that saves camera number
 */
robustPositioning::usbCamStreamer::usbCamStreamer(int nmbr){
    cameraNmbr = nmbr;
    initialized = 0;
    std::cout << "Creating internal camera streaming object ...";
    cv::Mat emptyFrame;
    getImage(emptyFrame);//Initialization
}
/*
 *Get next image interface. Initializes streamer object upon first call
 */
float robustPositioning::usbCamStreamer::getImage(cv::Mat& frame){
  static cv::VideoCapture cap;
  if(!initialized){
    if(!cap.open(cameraNmbr)){std::cout << "Failed.\n\t" << "Internal camera number not valid: " << cameraNmbr <<std::endl;return 0;}
    initialized=1;
    std::cout << "Done." <<std::endl;
    return 1;
  }
  cap.read(frame);
  if( frame.empty() ){std::cout << "Stream done." << std::endl; return 0;}//If stream is done return 0
  return 1;
}
int robustPositioning::usbCamStreamer::setSettings(int test){
  return 1;
}
/*
 *
 *
 *
 *      =======   DATASET STREAMER ========
 *
 *
 *
*/
//Default constructor. does not do anything. only called from Streamer Constructor if dataset streamer is NOT used
robustPositioning::datasetStreamer::datasetStreamer(void){

}
/*
 *Constructor that saves path to dataset using just cv::cap
 */
robustPositioning::datasetStreamer::datasetStreamer(std::string basePath,std::string dataFile_, bool useTimeStampFile_){
std::cout << "Initializing image dataset streamer..." << std::endl;
    pathToDataset = basePath;
    dataFile = dataFile_;
    initialized = 0;
    sequence = 0;
    useTimeStampFile = useTimeStampFile_;
    if(useTimeStampFile){
        std::cout <<"\tReading filenames from \"" << pathToDataset+dataFile << "\"..." << std::endl;
        if(readTimeStampData(dataFile_)){
            initialized = 1;
            std::cout << "\tRead " << timeStamps_f.size() << " timestamps and image file names" << std::endl;
            std::cout << "Done." << std::endl;
        }else{
            std::cout << "Failed." << std::endl;
            pathToDataset = basePath;//This is now the whole path i.e. somedir/images/img_%01.png
            //std::cout << "Could not read time stamp data from file \"" << path << "\"." << std::endl;
        }
    }
    datasetSize = timeStamps_f.size();
}
/*
 *Get next image interface. Initializes streamer object upon first call
 */
float robustPositioning::datasetStreamer::getImage(cv::Mat& frame){
    if(useTimeStampFile){
        return getImageViaTimestampList(frame);
    }else{
        return getImageViaCapStream(frame);
    }
}

float robustPositioning::datasetStreamer::getImageViaCapStream(cv::Mat& frame){
    static cv::VideoCapture cap;
    if(!initialized){
      if(!cap.open(pathToDataset)){std::cerr << "Path to dataset not valid:" << pathToDataset <<std::endl;return 0;}
      initialized=1;
    }
    cap.read(frame);
    if( frame.empty() ){std::cout << "Video stream done." << std::endl; return 0;}//If stream is done return 0
    return 1;

}

float robustPositioning::datasetStreamer::getImageViaTimestampList(cv::Mat& frame){
    if(sequence == datasetSize){//If stream is done
        cv::Mat emptyMat;
        emptyMat.copyTo(frame);
        std::cout << "xxx" << std::endl;
        return -1;
    }
    float timeStamp = timeStamps_f[sequence];
    frame = cv::imread(pathToDataset + imgNames[sequence],CV_LOAD_IMAGE_GRAYSCALE);
    sequence++;
    return timeStamp;
}
//This method is used to peek at the next timestamp to see if it is time (in simulation time) to read next frame
float robustPositioning::datasetStreamer::peek(void){
    if(sequence+1 < datasetSize){
        return timeStamps_f[sequence+1];
    }else{
        return -1;
    }
}

int robustPositioning::datasetStreamer::readTimeStampData(std::string path_to_data_file){
        std::string line;
        std::string delim = ",";
        std::ifstream file;
        file.open(path_to_data_file);
        int skiplines = 1;// skip header
        int count = -1; //start at -1 so it is 0 in first lap as while starts with incrementing count
        if(file.is_open()){
            std::cout << "\topened " <<path_to_data_file << "\"..."<< std::endl;
            //std::cout << "Opened file \"" << pathToDataset+ dataFile <<"\" ." <<std::endl;
             while(getline(file,line)){
                count++;
                if(count<skiplines){continue;}
                std::vector<std::string> parsed = parseRow(line);
                if(parsed.size()==2){//Disregard any lines that are not 2 elements long
                    timeStamps_s.push_back(parsed[0]);
                    imgNames.push_back(parsed[1]);
                }
             }
        }else{

            std::cout << "datasetStreamer::readTimeStampData: Could not open file \"" << path_to_data_file <<"\" ." <<std::endl;
            return 0;
        }
        file.close();
        //Convert timestamps from strings to floats
        for(int j=0;j<timeStamps_s.size();j++){
            //try
            float stamp = std::stof(timeStamps_s[j]);
            //catch
            timeStamps_f.push_back(stamp);
        }
        return 1;
}

//This method parses a string line into chunks separated by a ',' and ignoring LEADING whitespaces
//This function takes a line and parses it into a vector<string> using "," as deliminator and disregarding LEADING whitespaces
std::vector<std::string> robustPositioning::datasetStreamer::parseRow(std::string line){
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


int robustPositioning::datasetStreamer::setSettings(int test){
  return 1;
}



//Default constructor. does not do anything. onl called when Streamer does NOT use picam
robustPositioning::piCamStreamer::piCamStreamer(void){

}
/*robustPositioning::piCamStreamer::~piCamStreamer(void){
    std::cout << "Releasing rpi camera module...";
    cv::Mat temp;
    getImage(temp,false);
    std::cout << "Done." << std::endl;
}*/

robustPositioning::piCamStreamer::piCamStreamer(int imgType_){
    std::cout << "Initializing rpi camera module...";
    switch (imgType_) {
        case CV_8UC1:{
            imgType = imgType_;
            break;
        }
        case CV_8UC3:{
            imgType = imgType_;
            break;
        }
        default:{
            std::cout << "\nInvalid image type given to piCamStreamer constructor. Must be CV_8UC1 or CV_8UC3" << std::endl;
        }
    }
    cv::Mat temp;
    getImage(temp);
}

float robustPositioning::piCamStreamer::getImage(cv::Mat& frame){
    static bool initialized = false;
    static raspicam::RaspiCam_Cv RPICamera;
    if(!initialized){
	RPICamera.set(CV_CAP_PROP_FORMAT,imgType); //Set image to whatever is set to imgType
 	RPICamera.set(CV_CAP_PROP_FRAME_WIDTH,640);
	RPICamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
        RPICamera.open();
        if(!RPICamera.isOpened()){
            std::cerr<<"Failed. \n\tError opening camera"<<std::endl;
            cv::Mat emptyFrame;
            emptyFrame.copyTo(frame);
            return -1;
        }else{
            std::cout<<"Done.\n\tConnected to camera ="<<RPICamera.getId() <<std::endl;
            double format=RPICamera.get(CV_CAP_PROP_FORMAT);
            double c1 = CV_8UC1;
            double c2 = CV_8UC3;
            std::cout << "Format: ";
            if(format==c1){std::cout<< "CV_8UC1, ";}
            if(format==c2){std::cout << "CV_8UC3, ";}
            std::cout << "Width="<<RPICamera.get(CV_CAP_PROP_FRAME_WIDTH)<< ", Height="<<RPICamera.get(CV_CAP_PROP_FRAME_HEIGHT)<< std::endl;
            initialized = true;
            return -1;
        }
    }
    RPICamera.grab();
    RPICamera.retrieve(frame);
    return 1;
}

/*
 *
 *
 *
 *      =======   RPI Camera STREAMER ========
 *
 *
 *
*/
