/*First version of Optical Flow-based Visual odometry
    Uses IMU measurements to derotate flow field
    Uses single point range sensor for scaling
*/
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //For sleep

//using namespace cv;
//using namespace std;

class dataSetStreamer{
public:
    /*This method reads timestamps and data into vectors and vectors of vectors
     * 0. image base path. 1.imageData 2. imuData. 3.viconData (paths)
     */
    dataSetStreamer(std::string, std::string,std::string,std::string);
    /*This method retrieves the next IMU data. If available, vicon and image are returned as well
     *  A status-vector indicates which data is new
     */
    std::vector<int> getData(std::vector<float>&,std::vector<float>&,std::string&);//Updates IMU measurements and vicon (range) if available updataes image. In that case returns 1
private:
    /*These overloaded methods reads a text document with data separated by whitespaces
     * One or more data colums of wither float or string type.
     * All text documents are expected to have a double-type timestamp.
     */
    void readTXT(std::string,int,int,int,std::vector<int>,std::vector<std::vector<std::string>>&,std::vector<double>&);
    void readTXT(std::string,int,int,int,std::vector<int>,std::vector<std::vector<float>>&,std::vector<double>&);
    /*
     * This method parses a given string into a vector of substrings
     * Whitespace is only separator
     */
    std::vector<std::string> parse(std::string);
    /*
     * This method converts a string to double-type timestamp.
     * Offsets by skipping the given amount of leading chars
     */
    void string2Timestamp(std::vector<std::string>,int,int,std::vector<double>&);
    /*
     * This method converts a string to string or float-type data (with given cols)
     *  row, columns, inputoutput. Pushes the data back onto the inputoutput
     */
    void string2Data(std::vector<std::string>,std::vector<int>,std::vector<std::vector<std::string>>&);
    void string2Data(std::vector<std::string>,std::vector<int>,std::vector<std::vector<float>>&);
    /*
     * This method just prints a vector of string or float or double just for control
     */
    void show(std::string, std::vector<std::string>);
    void show(std::string, std::vector<float>);
    void show(std::string, std::vector<double>);
    /*
     * These are some attributes. All data is read in. (except image data which is saved as a path)
     */
    //IMU
    std::vector<double> imuStamps;
    std::vector<double>::iterator imuStampsIterator;
    std::vector<std::vector<float>> imuData;//incl timestamp
    std::vector<std::vector<float>>::iterator imuDataIterator; //Funkar eller måste definiera size på inre?
    //Images
    std::string imageBasePath;
    std::vector<double> imageStamps;
    std::vector<double>::iterator imageStampsIterator;
    std::vector<std::vector<std::string>> imagePaths;
    std::vector<std::vector<std::string>>::iterator imagePathsIterator;
    //Vicon
    std::vector<double> viconStamps;
    std::vector<double>::iterator viconStampsIterator;
    std::vector<std::vector<float>> viconData;
    std::vector<std::vector<float>>::iterator viconDataIterator;
};
/*
 * Constructor. reads in imu, image, and vicon data
 */
dataSetStreamer::dataSetStreamer(std::string imageBasePath_,
                            std::string imageDataPath,
                            std::string imuDataPath,
                            std::string viconDataPath){
    int offset = 5;
//Read IMU Data
    std::vector<int> imuDataCols{1,2,3,7,8,9};
    readTXT(imuDataPath,4,0,offset,imuDataCols,imuData,imuStamps);
    imuDataIterator = imuData.begin();
    imuStampsIterator = imuStamps.begin();
    show("IMU",imuData[0]);
// Read image Data
    imageBasePath = imageBasePath_;
    std::vector<int> imageDataCols{1};
    readTXT(imageDataPath,3,0,offset,imageDataCols,imagePaths,imageStamps);
    imagePathsIterator = imagePaths.begin();
    imageStampsIterator = imageStamps.begin();
    show("Images",imagePaths[0]);
// Read vicon data
    std::vector<int> viconDataCols{1,2,3,4,5,6};
    readTXT(viconDataPath,4,0,offset,viconDataCols,viconData,viconStamps);
    viconDataIterator = viconData.begin();
    viconStampsIterator = viconStamps.begin();
    show("Vicon",viconData[0]);
}
/*
 * This function takes a long string and parses it into a vector<string> using whitespaces as deliminators
 */
std::vector<std::string> dataSetStreamer::parse(std::string line){
    std::vector<std::string> parsed;
    std::string::iterator it = line.begin();
    while(it!=line.end()){
        std::string word;
        while(it!=line.end()){
            if(isspace(*it)){it++;}//Remove leading whitespaces
            else{break;}
        }
        while(it!=line.end()){
            if(!isspace(*it)){
                word+=*it;//Append the char to the temporary string
                it++;
            }//Go through until first whitespace
            else{break;}
        }
        parsed.push_back(word);//Push back the parsed word onto the return vector
    }
    return parsed;
}
/*
 * This function reads the specified text document. Skips the specified lines
 * Inputoutputs a vector<double> of timestamps and a vector<vector<string>> of data from the specified columns
 * int timestampOffset is the number of leading digits to omit before converting to double
 */
void dataSetStreamer::readTXT(std::string path,
                                                int skiplines,
                                                int timestampCol,
                                                int timestampOffset,
                                                std::vector<int> dataCols,
                                                std::vector<std::vector<std::string>>& data,
                                                std::vector<double>& timeStamps){
    std::string line;
    int rowNmbr = 0;
    std::ifstream file(path);
    if(file.is_open()){
        while(rowNmbr<skiplines){//Disregard the given number of lines
            getline(file,line);
            rowNmbr++;
        }
        while(getline(file,line)){
            std::vector<std::string> row = parse(line);
            string2Data(row,dataCols,data);
            string2Timestamp(row,timestampCol,timestampOffset,timeStamps);
        }
        file.close();
    }
    else std::cout << "Unable to open file" << std::endl;

}
// Overloaded version for float type data. Only difference is input argument. The inner string2Data
// is overloaded with automatic float-conversion
void dataSetStreamer::readTXT(std::string path,
                                                int skiplines,
                                                int timestampCol,
                                                int timestampOffset,
                                                std::vector<int> dataCols,
                                                std::vector<std::vector<float>>& data,
                                                std::vector<double>& timeStamps) {
    std::string line;
    int rowNmbr = 0;
    std::ifstream file(path);
    if(file.is_open()){
        while(rowNmbr<skiplines){//Disregard the given number of lines
            getline(file,line);
            rowNmbr++;
        }
        while(getline(file,line)){
            std::vector<std::string> row = parse(line);
            string2Data(row,dataCols,data);
            string2Timestamp(row,timestampCol,timestampOffset,timeStamps);
        }
        file.close();
    }
    else std::cout << "Unable to open file" << std::endl;
}
/*
 * This function takes a vector<string> and makes a new vector containgn only the såpecified columns
 *  and pushes that data back onto the inputoutput vector<vector<string>> or vector<vector<float>>
 */
void dataSetStreamer::string2Data(std::vector<std::string> row, std::vector<int> cols,std::vector<std::vector<std::string>>& data){
    std::vector<std::string> dataRow;
    for(int col=0;col<row.size();col++){
        if(std::find(cols.begin(), cols.end(), col) != cols.end()){
            dataRow.push_back(row[col]);//Add the given column to the datarow
        }
    }
    data.push_back(dataRow);
}
void dataSetStreamer::string2Data(std::vector<std::string> row, std::vector<int> cols,std::vector<std::vector<float>>& data){
    std::vector<float> dataRow;
    std::size_t s; //Must have this in stof
    for(int col=0;col<row.size();col++){
        if(std::find(cols.begin(), cols.end(), col) != cols.end()){
            float value = stof(row[col],&s);
            dataRow.push_back(value);//Add the given column to the datarow
        }
    }
    data.push_back(dataRow);
}
/*
 * This function takes a vector<string> row and a number which represent the column where the timestamps area
 * Also an integer char offset (leading number of chars omitted)
 * converts to double and pushes back to inputoutput vector<double>
 */
void dataSetStreamer::string2Timestamp(std::vector<std::string> row,int timestampCol,int offset,std::vector<double>& timestamps){
    std::string timestampST = row[timestampCol].substr(offset);
    std::size_t s; //Must have this in stod
    double timestamp = stod(timestampST,&s);
    timestamps.push_back(timestamp);
}
void dataSetStreamer::show(std::string name, std::vector<std::string> data){
    std::cout << name << ": ";
    for(std::string element:data){std::cout << element << ", ";}
    std::cout << std::endl;
}
void dataSetStreamer::show(std::string name, std::vector<float> data){
    std::cout << name << ": ";
    for(float element:data){std::cout << element << ", ";}
    std::cout << std::endl;
}
void dataSetStreamer::show(std::string name, std::vector<double> data){
    std::cout << name << ": ";
    for(double element:data){std::cout << element << ", ";}
    std::cout << std::endl;
}
std::vector<int> dataSetStreamer::getData(std::vector<float>& imuData,
                                std::vector<float>& viconData,
                                std::string& imagePath){
    std::vector<int> status;
    //As long as there is available IMU data, read new
    if(imuStampsIterator != imuStamps.end()){
        //std::vector<float> imu = *imuDataIterator;//Get data
        //imuData.push_back(imu);
        imuData = *imuDataIterator;//Get data
        status.push_back(1);//Got new imu data
    }else{status.push_back(0);return status;}
    //If there is "available" vicon data, and if vicon end has not been reached
    if(*imuStampsIterator >= *viconStampsIterator && viconStampsIterator!=viconStamps.end()){
        //std::vector<float> vicon = *viconDataIterator;
        //viconData.push_back(vicon);
        viconData = *viconDataIterator;
        viconDataIterator ++;
        viconStampsIterator ++;
        status.push_back(1);
    }else{status.push_back(0);}
    //If there is "available" image data, and if images end has not been reached
    if(*imuStampsIterator >= *imageStampsIterator && imageStampsIterator != imageStamps.end()){
        std::vector<std::string> name = *imagePathsIterator;
        std::string path = imageBasePath + name[0];
        imagePath = path;
        imagePathsIterator ++;
        imageStampsIterator++;
        status.push_back(1);
    }else{status.push_back(0);}
    //Increment imu iterators
    imuDataIterator++;
    imuStampsIterator++;
    return status;
}
