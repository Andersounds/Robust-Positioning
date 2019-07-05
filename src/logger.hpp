/*
 Data logger header
 Initial version only log float and Mat
 i.e timestamp is also float. Use <limits> ti check max time until overflow on current system!
 Futura meybe do double logger and see if performance is affected

 ADD a destructor that flushes the buffer when object is destroyed
*/

#ifndef LOGGER_H
#define LOGGER_H
#include <regex>


namespace log{
class dataLogger{
public:
    std::ofstream file;
    //cv::Mat_<float> buffer;
    //float* buffPtr;
    dataLogger(void);//Constructor. sets default settings
    ~dataLogger(void);
    int init(std::string,std::vector<std::string>);//init function that gives path to dir and datafield names
    int init(std::string,std::vector<std::string>,int,bool);//init function that gives path to dir, buffer size, and overwrite selection
    int bufferLength;
    int bufferWidth;
    bool overwrite;
    char sep;
    std::string fullPath;
    int dump(std::vector<float>&);//timestamp is given as double, data is given as vector
    int dump(std::vector<std::string>&); //overloaded for a set of strings instead
private:
    int writeInfoLine(std::ofstream&,std::vector<std::string>);//Used to write out the first line of datafield names
};
class imageLogger{
public:
    imageLogger(void);//Constructor. sets default settings
    int init(std::string,std::string);      //init funtion that gives path to dir, and name of newdir at that path
    int init(std::string,std::string, std::string);//init function that gives path to dir, name of newdir, and image format
    std::string dumpDirectory;
    std::string imgBaseStr;     //used for initial dump of images
    std::string dirPath;
    std::string dumpDirName;
    std::string imgFormatStr;
    int numOfDigits;        //standard four for images img_0001, img_0002, etc.
    std::string renameFile;
    int dump(const double&,cv::Mat&);//Timestamp is given as float, image is given as Mat
    int rename(cv::String,cv::String);//Method to rename a set of image files to an incremented sequence and create a csv file relating the old and new names
};

// Object to keep a subset of all image files with a certain number of digits in the number part
class subset{
   int fullNameLength;
   int baseNameLength;
   std::vector<std::string> fileNames;
public:
  // här kan vi istället bara ge antalet tecken i hela namnet, och antal tecken som i början är basestring
   subset(int baseNameLength_, std::string fileName_){
       fullNameLength = fileName_.size();
       baseNameLength = baseNameLength_;
       add(fileName_);
   }
   int add(std::string fileName){
       if(fileName.size() == fullNameLength){
            fileNames.push_back(fileName);
            return 1;
       }
       return 0;//If size is wrong
   }
   //Some operators that are needed to sort
   bool operator==(int nmbr){return fullNameLength == nmbr;}
  // bool operator<=(int nmbr){return numOfDigits <= nmbr;}
  // bool operator>=(int nmbr){return numOfDigits >= nmbr;}
  // bool operator<(int nmbr){return numOfDigits < nmbr;}
  // bool operator>(int nmbr){return numOfDigits > nmbr;}

  // This method empties the fileNames vector by popping and pushing it back to sortedVector
    void pop_push(int paddedSize,std::vector<std::string>& sortedVector){
        int subsetSize = fileNames.size();
        for(int i=0;i<subsetSize;i++){
            std::string fileName = fileNames.back();
            fileNames.pop_back();
            //No padding here
            //Create padding vector of right length with zeros
        //    int padNmbr = paddedSize - fileName.size();
        //    std::string padding = "";
        //    for(int j=0;j<padNmbr;j++){
        //        padding += "0";
        //    }
            //Build complete padded file name
            //std::paddedName = img base + padding + rest of string
        //    std::string paddedName = fileName.substr(0,baseNameLength-1) + padding + fileName.substr(baseNameLength-1,std::string::npos);
            sortedVector.push_back(fileName);
        }
//poppar vektorn, paddar numren med ngra nollor, och pushar tillbaka på inputoutputvektorn


    }
};
class fileNameSort{
public:
    std::vector<subset> subSets;
    int shortestName;
    int longestName;
    int imgBaseStrLength;
    std::string imgBaseName;
    std::string regexSyntax;
    std::string imgEndName;
    fileNameSort(std::string baseName){
        shortestName = -1;
        longestName = -1;
        imgBaseName = baseName;
        regexSyntax = "(" + baseName + ")([1-9])([0-9]*)(\\.)(.+)";
        imgBaseStrLength = baseName.size();
        //  (.*)            Any characters zero or more times
        //  (baseName)      Base name once
        //  [1-9]           Number 1 to 9 one time
        //  [0-9]*          Number 1 to 9 zero or more times
        //  \\.              A dot
        //  .+              Any character one or more times for the file ending
    }
    bool add(std::string fileName){
        static bool initialized = false;//This is false until a regex match allows us to identify file ending
        // Check syntax (baseName) ([0-9]) (.)***
        if(!std::regex_match(fileName,std::regex(regexSyntax))){return false;}
        else if(!initialized){//Identify the file ending.
            int from = imgBaseStrLength;
            //Check all characters after base name until a non-digit number is found
            while(isdigit(fileName[from])) {from++;}
            imgEndName = fileName.substr(from,std::string::npos);
            std::cout << "File ending identified as \"" << imgEndName << "\"." << std::endl;
            regexSyntax = "(" + imgBaseName + ")([1-9])([0-9]*)(" + imgEndName + ")";
            initialized = true;
        }
        // Here we could add another regex check. If the number start with 0, then that need to be handled
        // Count how many digits there are
        // We could also just count the number of chars in the whole string. if another file has x charcters and highest number is y, then the file needs (y-x) extra zeros
//        int num = countDigits(fileName);
        int num = fileName.size();//This is the full length of the filename.
        if(num<shortestName || shortestName==-1) shortestName = num;
        if(num>longestName) longestName = num;
        bool alreadyPresent = false;//init as false
        for(int i=0;i<subSets.size();i++){
            if(subSets[i]==num){//If number of digits are the same as a subset that is already present
                subSets[i].add(fileName);
                alreadyPresent = true;
                break;
            }
        }
        if(!alreadyPresent){
            subSets.push_back(subset(imgBaseStrLength,fileName));//Add new subset to vector of subsets
        }
        return true;
    }
    void pushBackSorted(std::vector<std::string>& emptyVector){
        std::cout << "Longest filename is " << longestName << " characters long." << std::endl;
        // Pop all subsets in order, shortestName first and then ordered up to longestName
        for(int i=shortestName;i<=longestName;i++){
            // Check all subsets j is the one next in turn
            for(int j=0;j<subSets.size();j++){
                // If it is, pop it and push the padded filenames back onto emptyVector
                if(subSets[j]==i){
                    std::cout << "Pushing all filenames with length " << i << " onto sorted vector" << std::endl;
                    subSets[j].pop_push(longestName,emptyVector);
                    break;
                }
            }
        }
    }

    void numericSort(std::vector<std::string>& fileNameVector){
        // - AlphaSort of the whole vector (At least make sure that it is sorted)
        std::sort(fileNameVector.begin(), fileNameVector.end());
        // - Empty the whole vector by popping it and adding all valid filenames
        int vectorLength = fileNameVector.size();
        for(int i=0;i<vectorLength;i++){
            std::string fileName = fileNameVector.back();
            fileNameVector.pop_back();
            if(!add(fileName)) {std::cout << "Syntax check failed for file \"" << fileName << "\". Skipping it." << std::endl;}
        }
        // - pop subsets in right order and push back to sortedVector.
        //std::vector<std::string> numericSortedVector;
        std::cout << "Sorting vector numerically w.r.t <xxx> in "<< imgBaseName << "<xxx>"<< imgEndName << "... " << std::endl;
        pushBackSorted(fileNameVector);
        std::cout << "Done." << std::endl;
    }
    std::vector<std::string> extractTimeStamps(std::vector<std::string> fileNames){
        std::vector<std::string> stamps;
        for(std::string fullName:fileNames){
            int from = imgBaseStrLength;
            int len = fullName.size() - imgEndName.size() - imgBaseName.size();
            std::string stamp = fullName.substr(from,len);
            stamps.push_back(stamp);
        }
        return stamps;
    }
};



}

#endif



//X   1. konstruktorn till subset
//X   2. pop pad push skriv klart
//X   3. pushBackSorted se till så det funkar
