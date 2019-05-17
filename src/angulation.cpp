#include <iostream>
#include <opencv2/opencv.hpp>
#include "angulation.hpp"
#include "azipe.hpp"




/*constructor. Set max id of database
*/
angulation::angulation::angulation(int maxId_){
    maxId = maxId_;
    for(int i=0;i<maxId;i++){
        dataBase.push_back(cv::Mat_<float>::zeros(3,1));//Set empty mat
        activeAnchors.push_back(false);                 //Set all anchors to inactive
    }
}
