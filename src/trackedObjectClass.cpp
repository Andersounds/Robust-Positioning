#include <iostream>
#include <opencv2/opencv.hpp>

#include "trackedObjectClass.hpp"


/*
 *Constructor. It saves all identifying features of the object to be tracked as well as
 *  calculate the keypoint-rect offsets that are needed lated.
 */
trackedObject::trackedObject(std::vector<cv::KeyPoint> keyPoints,
                                cv::Mat descriptors,
                                cv::Rect_<float> rectangle){
    // Save all the defining data in the object itself
    originalKeyPoints = keyPoints;
    originalDescriptors = descriptors;
    originalRectangle = rectangle;
    rectCorner = cv::Point2f(rectangle.x,rectangle.y);
    // Calculate all the offsets from keyPoints to rectangle corner
    for(int i=0;i<keyPoints.size();i++){
        cv::Point2f offset_i = rectCorner - originalKeyPoints[i].pt;
        offsets.push_back(offset_i);
    }
    matches = 0;
    matchAttempts = 0;
}
/*
 * Returns new cv::Point2f that can be used by the KLT tracker. Also inputoutput-returns a new Rect for the KLT
 * For now does not consider rotation. Just scaling
 */
std::vector<cv::Point2f> trackedObject::getTrackablePoints(float scale,
                                                            float angle,
                                                            std::vector<cv::DMatch> winningMatches,
                                                            std::vector<cv::KeyPoint> sceneKeyPoints,
                                                            cv::Rect_<float>& newRect){
    //Set new size of rectangle
    newRect.width = originalRectangle.width*scale;
    newRect.height = originalRectangle.height*scale;
    // Get corner coordinate of new rectangle by applying the original offset of the keypoint to rect rectCorner
    //  to the corresponding new keypoints that have a good match with the original ones
    cv::Point2f rectCoordinate = cv::Point2f(0,0);
    for(int i=0;i<winningMatches.size();i++){
        //Get index of old KP and new KP
        int keyPointOG_index = winningMatches[i].queryIdx;//NOTE QUERY OR TRAIN??
        int keyPointNEW_index = winningMatches[i].trainIdx;
        // Add the new coordinate estimation according to KP match i (new KP coordinate + old offset of corresponding KP)
        rectCoordinate += sceneKeyPoints[keyPointNEW_index].pt + offsets[keyPointOG_index]*scale;
    }
    //Normalize
    rectCoordinate /= (float) winningMatches.size();
    //Set new Coordinate
    newRect.x = rectCoordinate.x;
    newRect.y = rectCoordinate.y;
    // Get all new Keypoints that are located within the new rect
    std::vector<cv::Point2f> newPoints;
    int counter =0;
    for(int i=0;i<sceneKeyPoints.size();i++){
        //std::cout << "Rect coordinate: " << newRect.x << ", " << newRect.y << ", width: " << newRect.width <<". Point: " << sceneKeyPoints[i].pt << std::endl;
        //This is here to just return the keypoints that are in the rect
        if(newRect.contains(sceneKeyPoints[i].pt) && counter<100){//max 100 features
            newPoints.push_back(sceneKeyPoints[i].pt);
            counter++;
        }
    }
    return newPoints;
}
/*
 *This is an overloaded version of the above function used directly after instantiation.
 *  It simply converts original KP tp point2f and returns
 */
std::vector<cv::Point2f> trackedObject::getTrackablePoints(void){
    std::vector<cv::Point2f> points;
    cv::KeyPoint::convert(originalKeyPoints,points);
    return points;
}
void trackedObject::improve3D(cv::Point3f position,
                                float azimuthal,
                                float polar){



}//Position, azimuthal, polar





////////////////////////////////////


/*
 *Constructor. reserves specified size for vectors
 */
to::trackedObjectList::trackedObjectList(int size){
    list.reserve(size);
    activeIDs.reserve(size);
    maxNmbr = size;
    no_of_tracked = 0;
}
/*
 * Adds a new object. At first available place, or if there are any available, replaces the worst one
 */
void to::trackedObjectList::add(trackedObject* obj){
    if(no_of_tracked<maxNmbr){
        list[no_of_tracked] = obj;
        no_of_tracked++;
    }else{
        int index = getReplaceIndex();
        replace(index, obj);
    }
}
/*
 * Replaces an old index with a new one and deallocates the old memory
 */
void to::trackedObjectList::replace(int index, trackedObject* obj){
    delete list[index];     //Deallocate memory of object that is replaced
    list[index] = obj;      // Replace old pointer with new pointer
}
/*
 * Gets index of the worst tracked object according to match-rate
 *  Just returns the first of any index with the same and lowest rate
 */
int to::trackedObjectList::getReplaceIndex(void){
    float worstRate = 1;
    float rate = 1;
    int worstRateIndex = 0;
    for(int index = 0;index<maxNmbr;index++){
        rate = list[index]->matchRate;
        if(rate<worstRate){
            worstRate = rate;
            worstRateIndex = index;}
    }
    return worstRateIndex;
}
/*
 * Deallocates all objects
 */
void to::trackedObjectList::clear(void){ //C
    for(int i = 0; i<no_of_tracked; i++){
        delete list[i];
    }
}
