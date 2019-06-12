 #include <iostream>
#include <opencv2/opencv.hpp>

#include "trackedObjectClass.hpp"


/*
 *Constructor. It saves all identifying features of the object to be tracked as well as
 *  calculate the keypoint-rect offsets that are needed lated.
 */
trackedObject::trackedObject(std::vector<cv::KeyPoint> keyPoints,
                                cv::Mat descriptors,
                                cv::Rect_<float> rectangle,
                                int maxPoints,
                                cv::Mat roiFrame){
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
    color = cv::Scalar(0,0,255); //Default red
    //Prepare the constantly updated rect and points
    trackedRect = rectangle;
    trackedPoints.reserve(maxPoints);
    roiFrame.copyTo(originalFrame);
}
/*
 * Returns new cv::Point2f that can be used by the KLT tracker.
 * Also inputoutput-returns a new Rect for the KLT
 * Bases the decision on the winning matches.
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
to::trackedObjectList::trackedObjectList(int maxAnchors,int parallelAnchors){
    list.reserve(maxAnchors);
    activeIDs.reserve(parallelAnchors);
    activeStates.reserve(parallelAnchors);
    for(int i=0;i<parallelAnchors;i++){ //Set initial values
        activeIDs.push_back(1);    //Start in state 1. If no anchor is tracked yet it will move on to state 0
        activeStates.push_back(0);  //Initial state for switch statement is 0
        activeDelays.push_back(0);
    }
    maxNmbr = maxAnchors;
    no_of_parallel = parallelAnchors;
    no_of_tracked = 0;
    std::vector<cv::Scalar> tempColors{cv::Scalar(0,0,255),
                                        cv::Scalar(0,255,255),
                                        cv::Scalar(255,234,0),
                                        cv::Scalar(255,0,170),
                                        cv::Scalar(0,127,255),
                                        cv::Scalar(0,255,191),
                                        cv::Scalar(255,149,0),
                                        cv::Scalar(170,0,255),
                                        cv::Scalar(0,212,255),
                                        cv::Scalar(0,255,106),
                                        cv::Scalar(255,64,0),
                                        cv::Scalar(185,185,237),
                                        cv::Scalar(237,215,185),
                                        cv::Scalar(237,185,220),
                                        cv::Scalar(224,237,185),
                                        cv::Scalar(35,35,145),
                                        cv::Scalar(143,98,35),
                                        cv::Scalar(143,35,107),
                                        cv::Scalar(35,143,79)};
    colors = tempColors;
}
/*
 * Draws the tracked rect and points
 */
void to::trackedObjectList::drawAnchor(cv::Mat& frame,int activeAnchor){
    if(activeStates[activeAnchor]==2){
        //tracker.drawPoints(mat4visual,objectList.list[objectList.activeIDs[0]]->trackedPoints,mat4visual,CV_RGB(255,0,0));
        cv::rectangle(frame,list[activeIDs[activeAnchor]]->trackedRect,list[activeIDs[activeAnchor]]->color,2,cv::LINE_8,0);
    }
}
/*
 * Adds a new object. At first available place, or if there are any available, replaces the worst one
 * Returns the index at which the object is placed
 */
int to::trackedObjectList::add(trackedObject* obj){
    if(no_of_tracked<maxNmbr){
        list[no_of_tracked] = obj;
        obj->ID = no_of_tracked;//Give the object an ID (which is its index in the list)
        obj->color = colors[no_of_tracked%(colors.size()-1)];//Get a color from the colors vector
        no_of_tracked++;
    }else{
        int index = getReplaceIndex();
        if(index==-1){//If there is no available place for the objects
            delete obj;//Deallocate the object
            return index;//Return invalid index
        }
        replace(index, obj);
        obj->ID = index;//Give the object an ID, which is the index in the trackedObjects list
        obj->color = colors[index%(colors.size()-1)];//Get a color from the colors vector
    }
    return obj->ID;//The index at which the object pointer is placed
}
/*
 * Replaces an old index with a new one and deallocates the old memory
 * Returns the index at which the object is placed
 */
void to::trackedObjectList::replace(int index, trackedObject* obj){
    obj->ID = list[index]->ID;//Give the new object the ID of the object that it is replacing
    obj->color = list[index]->color;//Give the new object the same color as the object that it is replacing
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
    int worstRateIndex = -1;// start with invalid index. if this i returned then there is no available place
    for(int index = 0;index<maxNmbr;index++){
        if(isActive(index)){continue;}//Dont assign to a anchor that is already active
        rate = list[index]->matchRate;
        if(rate<worstRate){
            worstRate = rate;
            worstRateIndex = index;}
    }
    return worstRateIndex;
}
/*
 *
 */
void to::trackedObjectList::assignAnchor(int activeIndex,int anchorID){
    activeIDs[activeIndex] = anchorID;
}
/*
 *
 */
void to::trackedObjectList::updateAnchor(int activeIndex,
                                        cv::Rect_<float> newRect,
                                        std::vector<cv::Point2f> newPoints){
    list[activeIDs[activeIndex]]->trackedRect = newRect;
    list[activeIDs[activeIndex]]->trackedPoints.clear();//Necessary?
    list[activeIDs[activeIndex]]->trackedPoints = newPoints;//Is this correct? if size changes?

}

/*
 * Checks is the given ID is one of the active IDs
 */
bool to::trackedObjectList::isActive(int anchorID){
    return std::find(activeIDs.begin(),activeIDs.end(),anchorID) != activeIDs.end();
}
/*
 * Gets the number of active elements with state zero
 * Future: set state with a method setState instead. Then this can keep track of number of 0 states internally and this method just return an integer
 */
 std::vector<int> to::trackedObjectList::getZeroStateIndexes(void){
     std::vector<int> indexes;
     int index = 0;
     for(int state:activeStates){
         if(state==0||state==-1){
             indexes.push_back(index);
         }
         index++;
     }
     return indexes;
 }
/*
 * Setstate method
 */
void to::trackedObjectList::setState(int currentAnchor, int state){
     //int currentState = activeStates[currentAnchor];
     //if(currentState == 0){
    //     Use this statement to keep track of zeroStateUndexes for future
     //}
     activeStates[currentAnchor] = state;
 }
 /*
  * Getstate method
  */
int to::trackedObjectList::getState(int currentAnchor){ //Maybe zero delay here??
      return activeStates[currentAnchor];
}
/*
 *Set the specified anchor to state delay state (-1) with specified delay
 */
 void to::trackedObjectList::setDelay(int anchorID,int delay){
     setState(anchorID,-1);//Set to state 0
     activeDelays[anchorID] = delay;//Make abs here
 }
 /*
  * Decrements the delay of the specified anchor
  */
 void to::trackedObjectList::decrementDelay(int anchorID){
     activeDelays[anchorID]--;
 }
 /*
  * Gets the delay of the specified anchor (in the list of active anchors)
  */
 int to::trackedObjectList::getDelay(int anchorID){
     return activeDelays[anchorID];
 }

/*
 * Deallocates all objects
 */
void to::trackedObjectList::clear(void){
    for(int i = 0; i<no_of_tracked; i++){
        delete list[i];
    }
}
