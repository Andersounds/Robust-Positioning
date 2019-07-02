/*
 Data logger header
 Initial version only log float and Mat
 i.e timestamp is also float. Use <limits> ti check max time until overflow on current system!
 Futura meybe do double logger and see if performance is affected

 ADD a destructor that flushes the buffer when object is destroyed
*/



namespace log{
class dataLogger{
public:
    std::ofstream file;
    //cv::Mat_<float> buffer;
    //float* buffPtr;
    dataLogger(void);//Constructor. sets default settings
    int init(std::string,std::vector<std::string>);//init function that gives path to dir and datafield names
    int init(std::string,std::vector<std::string>,int,bool);//init function that gives path to dir, buffer size, and overwrite selection
    int bufferLength;
    int bufferWidth;
    bool overwrite;
    char sep;
    std::string fullPath;
    int dump(std::vector<float>&);//timestamp is given as double, data is given as vector
private:
    int writeInfoLine(std::ofstream&,std::vector<std::string>);//Used to write out the first line of datafield names
};
class imageLogger{
    imageLogger(void);//Constructor. sets default settings
    int init(std::string);
    int init(std::string,int,bool);//init function that gives path to dir, buffer size, and overwrite selection
    int bufferSize;
    bool overwrite;
    int initialImageNumber;
    int numOfDigits;//standard four for images img_0001, img_0002, etc.
    std::string fullPathFile;
    std::string imgBaseStr;
    int dump(double,cv::Mat);//Timestamp is given as double, image is given as Mat
};
}
