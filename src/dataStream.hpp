
/*
 * This is a -simple- datastream class.
 *  -It reads the provided csv-file row-by-row, and converts the values to float or double-type
 *  -It is out of the scope of the dataset streamer to follow simulation time by reading timestamps
 *  -Timestamps does not hold a special case in the datastream
 *  -It is up to the user to specify which column is which data (time, gyro-x-y-z etc.)
 *
 */

#include <iostream>
#include <fstream>
#include <vector>

#ifndef DATASTREAMER_H
#define DATASTREAMER_H





namespace robustPositioning{

class dataStreamer{
public:
    dataStreamer(std::string);          //constructor that reads the file into vector of vectors of floats
    dataStreamer(std::string, int);     //overloaded constructor where we define header size. i.e. number of rows to be skipped
    dataStreamer(std::string, int,int); //Overloaded constructor where we also define number of datafields
    bool get(std::vector<float>&);   //Reads next datarow
private:
    int skipLines;
    int dataFields;
    bool readOnlySpecifiedSize;
    std::vector<std::vector<std::string>> data_s; //Container for all data in string format
    std::vector<std::vector<float>>       data_f;//Container of all data in float-format
    void initialize(std::string);           //Reads data from file and converts it to float
    int readDataFile(std::string);
    std::vector<std::string> parseRow(std::string);
};

}

#endif
