#include <iostream>
#include <opencv2/opencv.hpp>
#include "angulation.hpp"
#include "azipe.hpp"




/*constructor. Set max id of database, initializes it with the provided path to csv file containing id and coordinates
*/
ang::angulation::angulation(int maxId_,std::string path){
    maxId = maxId_;
    for(int i=0;i<maxId;i++){
        dataBase.push_back(cv::Mat_<float>::zeros(3,1));//Set empty mat
        activeAnchors.push_back(false);                 //Set all anchors to inactive
    }
    std::vector<int> ids;
    std::vector<cv::Mat_<float>> coordinates;
    if(!readToDataBase(path,ids,coordinates)){
        std::cout << "Could not read anchor database" << std::endl;
    }else{
        int index = 0;
        for(int id:ids){
            coordinates[index].copyTo(dataBase[id]);//Set coordinates to the right anchor ID
            index++;
        }
    }
}
/*Takes a path and reads the file into two vectors, vector<int> IDs and vector<mat> coordinates
 * Ignores any LEADING whitespaces and rows not containing exactly 4 data fields. delimiter is char ','
 * Each anchor shall be specified as id,x,y,z\n (Whit optional whitespaces after any comma)
 */
int ang::angulation::readToDataBase(std::string path,std::vector<int>& IDs, std::vector<cv::Mat_<float>>& coordinates){
    std::string line;
    std::string delim = ",";
    std::ifstream file;
    file.open(path);
    if(file.is_open()){
         while(getline(file,line)){
            std::vector<std::string> parsed = parse(line);
            if(parsed.size()==4){//Disregard any lines that are not exactly 4 elements long
                int id      =   std::stoi(parsed[0]);
                float x     =   std::stof(parsed[1]);
                float y     =   std::stof(parsed[2]);
                float z     =   std::stof(parsed[3]);
                cv::Mat_<float> coord = cv::Mat_<float>::zeros(3,1);
                coord(0,0) = x;
                coord(0,1) = y;
                coord(0,2) = z;
                IDs.push_back(id);
                coordinates.push_back(coord);
            }
         }
    }else{return 0;}
    file.close();
    return 1;
}

//This function takes a line and parses it into a vector<string> using "," as deliminator and disregarding LEADING whitespaces
std::vector<std::string> ang::angulation::parse(std::string line){
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
