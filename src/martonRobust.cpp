


/* This method takes one or more pixel locations, undistorts them, and calculate alpha, beta, and gamma angles
    alpha: apparent roll angle of uav if anchor were located directly below it and uav has 0 yaw angle
    beta : apparent pitch angle of uav if anchor were located directly below it and uav has 0 yaw angle
    gamma: apparent yaw of uav if anchor were located in direction of 0 yaw of uav if it has no roll or pitch

    These angles can then be de-rotated using the known current roll and pitch, and the single combined remaining angle
     is then used together with height estimation to estimate horizontal radial distance to anchor.
*/
void robustPositioning::martonRobust::pix2angles(const std::vector<cv::Point2f>& cornerLocations,std::vector<float>& angles){

}


/*Overloaded version of pix2uLOS. If corner locations of anchors are given instead of points directly, the mean of each vector<point2f> is considered
 *  as the anchors location. This is calculated and then the original pix2uLOS is called
 */
void robustPositioning::martonRobust::pix2angles(const std::vector<std::vector<cv::Point2f>>& cornerLocations,std::vector<float>& angles){
    std::vector<cv::Point2f> anchorLocations;
    for(int i=0;i<cornerLocations.size();i++){//Go through all anchors
        std::vector<cv::Point2f>::const_iterator cornerIt = cornerLocations[i].begin();
        cv::Point2f location(0,0);
        while(cornerIt != cornerLocations[i].end()){
            location += *cornerIt;
            cornerIt++;
        }
        location.x /= cornerLocations[i].size();
        location.y /= cornerLocations[i].size();
        anchorLocations.push_back(location);
    }
    //Do standard function call to calculate
    return pix2angle(anchorLocations,angles);
}


/* method to perform in-place correction of barrel distortion of pixel coordinates
 * Using the following formula: https://docs.opencv.org/3.4.0/d4/d94/tutorial_camera_calibration.html
 */
cv::Point2f robustPositioning::martonRobust::unDistort(const cv::Point2f& point){
    //Shift coordinate system center to middle of image
    float x = point.x - K(0,2);
    float y = point.y - K(1,2);
    //Perform undistortion
    float r2 = x*x + y+y;//Radius squared
    float A = (1 + k1_barrel*r2 + k2_barrel*r2*r2 + k3_barrel*r2*r2*r2);
    cv::Point2f undistortedPoint(x/A,y/A);//SHOULD IT BE MULTIPLIED? docs are not definitive. Different in diferent version of docs
    //Shift coordinates back to image
    undistortedPoint.x += K(0,2);
    undistortedPoint.y += K(1,2);
    return undistortedPoint;
}
/* Interface mathod used to set K mat and calculate its inverse
 */
void robustPositioning::martonRobust::setKmat(cv::Mat_<float>K_){
    K = K_;
    K_inv = K.inv();
}
/* Interface method used to set T mat. T is transformation fro  uav frame to camera frame
 */
void robustPositioning::martonRobust::setTmat(cv::Mat_<float> T_){
    T = T_;
    T_inv = T.inv();
}
/* Interface method to set distortion coefficients for barrel distortion compensation
*/
void robustPositioning::martonRobust::setDistortCoefficents(float k1, float k2, float k3){
    k1_barrel = k1;
    k2_barrel = k2;
    k3_barrel = k3;
}
