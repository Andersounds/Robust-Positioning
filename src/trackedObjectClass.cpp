#include <iostream>
#include <opencv2/opencv.hpp>

#include "trackedObjectClass.hpp"


/*
 *Constructor. It saves all identifying features of the object to be tracked as well as
 *  calculate the keypoint-rect offsets that are needed lated.
 */
trackedObject::trackedObject(std::vector<cv::KeyPoint> originalKeyPoints,
                                cv::Mat originalDescriptors,
                                cv::Rect_<float> originalRectangle){
    // Save all the defining data in the object itself
    keyPoints = originalKeyPoints;
    descriptors = originalDescriptors;
    rectangle = originalRectangle;
    rectCorner = cv::Point2f(rectangle.x,rectangle.y);
    // Calculate all the offsets from keyPoints to rectangle corner
    for(int i=0;i<keyPoints.size();i++){
        offsets.push_back(keyPoints[i].pt - rectCorner);
    }
}
/*
 *
 *
 */
std::vector<cv::Point2f> trackedObject::getTrackablePoints(float scale,
                                                            float angle,
                                                            std::vector<cv::DMatch> winningMatches,
                                                            std::vector<cv::KeyPoint> sceneKeyPoints,
                                                            cv::Rect_<float>& newRect){
    std::vector<cv::Point2f> points;

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
