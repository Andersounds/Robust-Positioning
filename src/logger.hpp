

namespace log{
class dataLogger{
    datalogger(void);//Constructor. sets default settings
    int init(std::string,std::vector<std::string>);//init function that gives path to dir and datafield names
    int init(std::string,std::vector<std::string>,int,bool);//init function that gives path to dir, buffer size, and overwrite selection
    int bufferSize;
    bool overwrite;
    std::std fullPath;
    int dump(double,std::vector<float>);//timestamp is given as double, data is given as vector
};
class imageLogger{
    imageLogger(void);//Constructor. sets default settings
    int init(std::string);
    int init(std::string,int,bool);//init function that gives path to dir, buffer size, and overwrite selection
    int bufferSize;
    bool overwrite;
    int initialImageNumber;
    int numOfDigits;//standard four for images img_0001, img_0002, etc.
    std::str fullPathFile;
    std::str imgBaseStr;
    int dump(double,cv::Mat);//Timestamp is given as double, image is given as Mat
};
}
