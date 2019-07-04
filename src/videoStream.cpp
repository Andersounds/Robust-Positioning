#include <iostream>
#include <opencv2/opencv.hpp>
#include "videoStream.hpp"


/* Factory methods for abstract streamer class with default settings
 *
 */
Streamer *robustPositioning::Streamer::make_streamer(int choice, std::string path, int cam)
{
  if      (choice == 1)
  {
    if(cam==0){return new usbCamStreamer(0);}//Default cam
    else {return new usbCamStreamer(cam);}//
  }
  else if (choice == 3)
  {
    if(path==""){return new datasetStreamer("/Users/Fredrik/Datasets/Euroc/V101/cam0/data/cam0_%04d.png");}//Default path
    else {return new datasetStreamer(path);}
  }
  else
  {
    std::cout << "No valid mode given. Defaulting to internal cam 0" << std::endl;
    return new usbCamStreamer(cam);
  }
}
//When giving just the single int argument, only rpi videostream or default internal cam stream is possible
Streamer *robustPositioning::make_streamer(int choice){
    if(choice == MODE_USB_CAM){
        return new usbCamStreamer(0);
    }else if(choice == MODE_RPI_CAM){
        std::cout << "RPI cam stream chosen. not available in this implementation." << std::endl;
        std::cout << "Defaulting to internal camera 0" << std::endl;
        return new usbCamStreamer(0);
    }

}
//When two int arguments are given, it must be internal cam and the camera choice
Streamer *robustPositioning::make_streamer(int choice,int cam){
    if(choice == MODE_USB_CAM){
        return new usbCamStreamer(cam);
    }else{
        std::cout << "Camera number choice is not possible if not MODE_USB_CAM is chosen" << std::endl;
        std::cout << "Defaulting to internal camera 0" << std::endl;
        return new usbCamStreamer(0);
    }
}
//For choosing dataset. Just reading images
Streamer *robustPositioning::make_streamer(std::string datasetPath){
    std::cout << "Dataset image streamer chosen with path \"" << datasetPath << "\"." << std::endl;
    return new datasetStreamer(datasetPath);
}
Streamer *robustPositioning::make_streamer(int choice, std::string datasetPath){
    bool useTimeStampFile;
    if(choice == MODE_IMG_STREAM){
        std::cout << "Dataset image streamer chosen with path \"" << datasetPath << "\"." << std::endl;
        useTimeStampFile = false;
        return new datasetStreamer(datasetPath,useTimeStampFile);
    }else if(choice == MODE_IMG_TIMESTAMP_STREAM){
        std::cout << "Dataset image with timestamp streamer chosen. using path \"" << datasetPath << "\"." << std::endl;
        useTimeStampFile = true;

    } else{
        std::cout << "No valid choice given. To stream dataset choose either " << MODE_IMG_STREAM << " or " << MODE_IMG_TIMESTAMP_STREAM << std::endl;
        std::cout << "Defaulting to internal camera 0" << std::endl;
        return new usbCamStreamer(0);
    }
    return new datasetStreamer(datasetPath,useTimeStampFile);
}











/*
 *Constructor that saves camera number
 */
robustPositioning::usbCamStreamer::usbCamStreamer(int nmbr){
    cameraNmbr = nmbr;
    initialized = 0;
    std::cout << "Mode " << MODE_USB_CAM ". USB camera streamer object created." << std::endl;
}
/*
 *Get next image interface. Initializes streamer object upon first call
 */
int robustPositioning::usbCamStreamer::getImage(cv::Mat& frame){
  static cv::VideoCapture cap;
  if(!initialized){
    if(!cap.open(cameraNmbr)){std::cerr << "Internal camera number not valid: " << cameraNmbr <<std::endl;return 0;}
    initialized=1;
  }
  cap.read(frame);
  if( frame.empty() ){std::cout << "Stream done." << std::endl; return 0;}//If stream is done return 0
  return 1;
}
int robustPositioning::usbCamStreamer::setSettings(int test){
  return 1;
}




/*
 *Constructor that saves path to dataset
 */
robustPositioning::datasetStreamer::datasetStreamer(std::string path,bool useTimeStampFile_){
    pathToDataset = path;
    initialized = 0;
    useTimeStampFile = useTimeStampFile_;
    if(useTimeStampFile){
        if(readTimeStampData(path)){
            initialized = 1;
        }else{
            std::cout << "Could not read time stamp data from file \"" << path << "\"." << std::endl;
        }
    }
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
    if( frame.empty() ){std::cout << "Stream done." << std::endl; return 0;}//If stream is done return 0
    return 1;

}

float robustPositioning::datasetStreamer::getImageViaTimestampList(cv::Mat& frame){
    if(!initialized){
        readTimeStampData();
    }


}
int robustPositioning::datasetStreamer::readTimeStampData(void){
        std::string line;
        std::string delim = ",";
        std::ifstream file;
        file.open(pathToDataset);
        int skiplines = 1;
        int count = 0;
        if(file.is_open()){
             while(getline(file,line)){
                if(count<skiplines) continue;
                count++;
                std::vector<std::string> parsed = parseRow(line);
                if(parsed.size()==2){//Disregard any lines that are not 2 elements long
                    timeStamps_s.push_back(parsed[0]);
                    imgNames.push_back(parsed[1]);
                }
             }
        }else{return 0;}
        file.close();
        //Convert timestamps from strings to floats
        for(int j=0;j<timeStamps_s.size();j++){
            //try
            float stamp = std::stof(timeStamps_s[j]);
            //catch
            timeStamps_f.push_back(stamp)
        }
        return 1;
}
int robustPositioning::datasetStreamer::setSettings(int test){
  return 1;
}
