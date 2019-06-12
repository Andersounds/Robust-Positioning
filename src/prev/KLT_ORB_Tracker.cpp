#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv2/tracking.hpp>
#include "KLT_ORB_Tracker.hpp"
//#include "houghDetector.cpp"


KLT_ORB_Tracker::KLT_ORB_Tracker(void){


}//Is the constructor needed?
int KLT_ORB_Tracker::init(void){
    /*Initialize the ORB detector and descriptor computer itself*/
    orbObject = cv::ORB::create(
                    ORBsettings.nfeatures,
                    ORBsettings.scalefactor,
                    ORBsettings.nlevels,
                    ORBsettings.edgeThreshold,
                    ORBsettings.firstLevel,
                    ORBsettings.WTA_K,
                    ORBsettings.scoreType,
                    ORBsettings.patchSize,
                    ORBsettings.fastThreshold);
    /*Initialize the ORBmatcher*/
    matcherObject = cv::BFMatcher::create(
                    ORBsettings.normType,
                    ORBsettings.crosscheck);
    return 1;
}
/*
 * getFeatures
 * Takes an image
 * Returns keypoints found according to ORB
*/
int KLT_ORB_Tracker::getFeatures(cv::Mat frame, std::vector<cv::KeyPoint>& keypoints){//Alternativt: ge hela bilden och en rect som specificerar RoI
    orbObject->detect(frame,keypoints);
    return 1;
}
std::vector<cv::KeyPoint> KLT_ORB_Tracker::getFeatures(cv::Mat frame, cv::Mat mask){//Alternativt: ge hela bilden och en rect som specificerar RoI
    std::vector<cv::KeyPoint> newKeypoints;
    orbObject->detect(frame,newKeypoints,mask);
    return newKeypoints;
}
std::vector<cv::KeyPoint> KLT_ORB_Tracker::getFeatures(cv::Mat frame, cv::Rect_<float> mask){//Alternativt: ge hela bilden och en rect som specificerar RoI
    std::vector<cv::KeyPoint> newKeypoints;
    orbObject->detect(cv::Mat(frame,mask),newKeypoints);
    std::vector<cv::KeyPoint>::iterator it=newKeypoints.begin();
    //Offset all KP to the coordinate system of the original frame
    while(it!=newKeypoints.end()){
        it->pt.x+=mask.x;
        it->pt.y+=mask.y;
        it++;
    }
    return newKeypoints;
}


/*
 * calcORBDescriptors
 * Takes an image and a set of keypoints found in the image.
 * Returns the ORB descriptors
 * Just a wrapper around the compute function
*/
int KLT_ORB_Tracker::calcORBDescriptors(cv::Mat RoI, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    orbObject ->compute(RoI,keypoints, descriptors);
    return 1;
}
/*
 * findClusters
 * Takes a set of keypoints and corrseponding image together with some additional parameters
 * Returns the specified number of rects that correspond to the most keypoint-dense areas of the original frames
 * IMPROVEMENT IDEAS:
 * - Return up-to the amout of specified rects. Handle the situations when not enough clusters are found
*/
std::vector<cv::Rect_<float>> KLT_ORB_Tracker::findClusters(cv::Mat frame, cv::Mat invMask, std::vector<cv::KeyPoint> keypoints, int noOfClusters, int kernelSize, int minDistance){
    //cv::imshow("VideoStream",invMask);
    //std::cout << "Invmask upon entry" << std::endl;
    //cv::waitKey(0);
    /*Init the return variable*/
    std::vector<cv::Rect_<float>> rects;
    cv::Mat keypointMat = cv::Mat::zeros(frame.size(), CV_16UC1);    //Create mat where keypoint coordinates are set to non-zero
    cv::Mat borderMask = cv::Mat::zeros(frame.size(),CV_8UC1);      //Create a mask that is used to remove the borders so that all rects are completely contained within the original frame
    float border = ceil((float)kernelSize/2.0);
    cv::rectangle(borderMask, cv::Point(border,border),cv::Point(frame.cols-border,frame.rows-border),cv::Scalar(255),CV_FILLED,cv::LINE_8,0);
    for(cv::KeyPoint kp:keypoints){//Set KP coordinates to nonzero value
      keypointMat.at<ushort>(kp.pt) = 1;//Maybe use other value 1 is enough? maybe use keypoint response here?
    }
    cv::Mat kernel = cv::Mat::ones(kernelSize,kernelSize,CV_16UC1);//Kernel for filtering
    cv::Mat filteredMat;
    //Calc. convolution (actuallly correlation) i.e. find point of maximum mean over kernel
    cv::filter2D(keypointMat, filteredMat, -1 , kernel, cv::Point( -1, -1 ), 0, cv::BORDER_CONSTANT);// anchor is kernel center
    //filter2D: 26.69 fps, sepFilter2D: 24.86fps. If I have sepfilter the cluster dues not have to be quadratic

    for(int i=0;i<noOfClusters;i++){
        double min, max;
        cv::Point min_loc, max_loc;
        cv::Mat completeMask = borderMask & invMask;
        cv::minMaxLoc(filteredMat, &min, &max, &min_loc, &max_loc,completeMask);
        cv::Rect_<float> RoI(max_loc.x-border,max_loc.y-border,kernelSize,kernelSize);
        rects.push_back(RoI);

        blackOutMask(invMask,RoI,minDistance);
        //cv::Rect_<float> blackOut(RoI.x-minDistance,RoI.y-minDistance,(kernelSize+2*minDistance),(kernelSize+2*minDistance));
        //cv::rectangle(invMask, blackOut,0,CV_FILLED,cv::LINE_8,0);
    }
    return rects;
}
/*
 * trackOpticalFlow
 * Takes the newest, previous frame, and a rect within which a vector of points to be tracked between are located.
 * Returns a new region of interest (in-place edited rect) of new estimated location of rect
 * If region moves out of bounds the return integer is zero, which can be used to trigger a new RoI search

*/
int KLT_ORB_Tracker::trackOpticalFlow(cv::Mat prevFrame, cv::Mat nextFrame, std::vector<cv::Point2f>& corners, cv::Rect_<float>& rectangle){//Rest of arguments come from class attributes
//This creates coordinate limits for the searching rectangle. If it goes out of bounds tracking is considered to be lost
    static int xMax = nextFrame.cols - rectangle.width/2;
    static int yMax = nextFrame.rows - rectangle.height/2;
    static int xMin = -rectangle.width/2;
    static int yMin = -rectangle.height/2;
    int noOfCorners = corners.size();
    if(noOfCorners<1){return 0;}//If no corners to track then return 0
    std::vector<cv::Point2f> trackedCorners(noOfCorners);// New estimation of corner locations
    std::vector<uchar> status;
    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(prevFrame,nextFrame,corners,trackedCorners,
                            status,
                            errors,
                            cv::Size(KLTsettings.windowSize,KLTsettings.windowSize),
                            KLTsettings.maxLevel,
                            KLTsettings.termcrit,
                            KLTsettings.flags,//For special operation
                            0.001);
    //cv::Point2f roIShift(0,0);
    //cv::Point2f TEMProIShift(0,0);
    float normInteger = 0;
    float pointsInRect = 0;
    float x=0;float y=0;float xT=0;float yT=0;//Cluster center of mass
    //Detta kan göras med en static variable som förskjuts. Om det är samma points som kallas på varje gång
    for(int i=0; i<trackedCorners.size(); i++){
      xT += trackedCorners[i].x;
      x  += corners[i].x;
      yT += trackedCorners[i].y;
      y  += corners[i].y;
      normInteger += 1;
      if(rectangle.contains(corners[i])){pointsInRect++;}//Count how many of the first points that were (are) in the initial rect
    }
    x/=normInteger;
    y/=normInteger;
    xT/=normInteger;
    yT/=normInteger;
    pointsInRect/=normInteger;//Get percentage value
    float offsetX = rectangle.x - x;//Get the offset from old rect to old points center of mass
    float offsetY = rectangle.y - y;
    rectangle.x = xT + offsetX;     //Place the new rectangle with the calculated offset from NEW center of mass
    rectangle.y = yT + offsetY;
    //Check if new rect is within bounds AND that at least 70% of old points were in old rect. Otherwise return 0
    if(rectangle.x <= xMax && rectangle.x>= xMin && rectangle.y <=yMax && rectangle.y >=yMin && pointsInRect >= 0.7){
        corners = trackedCorners;//Shift new corners.
        return 1;
    }else{return 0;} //Out of bound
}
/*
 * getQueryFeatures
 * Takes the queryImg and a rect that is used to create a Mask (with larger area?), within which the queryfeatures are located
 * For each keypoint found, the descriptor is calculated directly
*/
int KLT_ORB_Tracker::getQueryFeatures(cv::Mat frame, cv::Rect maskRect,std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors){
  cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());
  cv::Rect largerMaskRect(maskRect.x-(int)maskRect.width/2, maskRect.y-(int)maskRect.height/2, 2*maskRect.width, 2*maskRect.height);
  cv::rectangle(mask,largerMaskRect,1,CV_FILLED,cv::LINE_8,0);
  orbObject->detectAndCompute(frame, mask, keyPoints, descriptors);
  return 1;
}
/*
 * featureMatching
 *
 *
*/
int KLT_ORB_Tracker::featureMatching(cv::Mat roiDescriptors, cv::Mat queryDescriptors, std::vector<cv::DMatch>& matches){ //Maybe use mask in future?
  matcherObject ->match(roiDescriptors,queryDescriptors,matches);//OBS ordning på descriptors orsakar krasch
  return 1;
}
/*
 * trackMatches
Denna funktion ska ge en ny skattning på vår Roi och uppdatera våra nuvarande keyPoints och deras descriptors
Vi har: keypoints och descriptors från init eller frra gången denna funktionen kördes
Vi har också: Matchningar mellan förra descriptors och nya som har samlats in i ett lite större område
Vi ska: Välja ut de bästa matchningarna, och skriva över våra gamla keypoints och deras descriptors med de nya, matchade, keypoints och deras descriptrs
Vi ska också: definiera om Roi (vår rect) så att den omfattar de nya keypoints
 *
 * -----------Ersätts av Hough Detector??------------
*/
int KLT_ORB_Tracker::trackMatches(std::vector<cv::DMatch> matches,std::vector<cv::DMatch>& goodmatches, std::vector<cv::KeyPoint> objectKeyPoints, std::vector<cv::KeyPoint> sceneKeyPoints, std::vector<cv::Point2f>& newObjectPoints,cv::Rect& roi,std::vector<cv::Point2f>& goodMatchesPoints,std::vector<cv::Point2f>& src,std::vector<cv::Point2f>& dst, cv::Mat& H){
  float dist_max = 0;
  float dist_min = 100;
  for(int i=0;i<matches.size();i++){
    float dist = matches[i].distance;
    if( dist < dist_min ) dist_min = dist;
    if( dist > dist_max ) dist_max = dist;
  }
//std::cout << "min dist: " << dist_min << ", max dist: " << dist_max << std::endl;
  //Keep only good matches whose distance is less than 3*dist_min
  std::vector< cv::DMatch > good_matches;
  for(int i = 0; i < matches.size(); i++){
    if(matches[i].distance < 3*dist_min){
      good_matches.push_back(matches[i]);
    }
  }
  goodmatches = good_matches;
  std::cout << "Featurematching. Good matches: " << good_matches.size() << std::endl;
  //Take the points of the good matches
  std::vector <cv::Point2f> scene, object;
  for(int i=0; i<good_matches.size(); i++){
    object.push_back( objectKeyPoints[ good_matches[i].trainIdx ].pt );//train or query? noone knows.
    scene.push_back( sceneKeyPoints[ good_matches[i].queryIdx ].pt );
  }

  //cv::Mat H;
  H = cv::findHomography(object,scene,cv::RANSAC,1);
  std::cout << "H: " << H << std::endl;

  //Better threshold for ransac?
  //cv::Mat H = cv::Mat::eye(3,3,CV_32FC1);
  if(H.rows && good_matches.size()>6){
    std::vector<cv::Point2f> src2;
    std::vector<cv::Point2f> dst2;
    src2.push_back(cv::Point2f(roi.x,roi.y));
    src2.push_back(cv::Point2f(roi.x+roi.width,roi.y));
    src2.push_back(cv::Point2f(roi.x+roi.width,roi.y+roi.height));
    src2.push_back(cv::Point2f(roi.x,roi.y+roi.height));
    cv::perspectiveTransform(src2,dst2,H);
    src = src2;
    dst = dst2;
    std::cout << "Match!" << std::endl;
  }



  //std::cout << "Tracker rect new pos: " << roi.x << std::endl;
  goodMatchesPoints = scene;
  //newObjectPoints =  scene;
  //if(good_matches.size()<3){return 0;}//Some criteria to discard bad matching.
  return 1;
}
/*
 * drawKeypoints or points
 *
 *
*/
int KLT_ORB_Tracker::drawPoints(cv::Mat inputImg, std::vector<cv::KeyPoint> keyPoints, cv::Mat& outputImg, cv::Scalar color){
  cv::drawKeypoints(inputImg, keyPoints, outputImg,color);
  return 1;
}
int KLT_ORB_Tracker::drawPoints(cv::Mat inputImg, std::vector<cv::Point2f> keyPoints, cv::Mat& outputImg, cv::Scalar color){
  std::vector<cv::KeyPoint> kp;
  cv::KeyPoint::convert(keyPoints,kp);
  cv::drawKeypoints(inputImg, kp, outputImg,color);
  return 1;
}

/*
 * drawTheMatches
 *
 *
*/
int KLT_ORB_Tracker::drawTheMatches(cv::Mat& RoI, std::vector<cv::KeyPoint>&RoIKeypoints, cv::Mat& queryImg, std::vector<cv::KeyPoint>& queryKeypoints, std::vector<cv::DMatch>& matches, cv::Mat& imgOut){
  std::vector< char >  drawMask; //No mask
  cv::drawMatches(RoI,RoIKeypoints,queryImg,queryKeypoints,
                 matches, imgOut, cv::Scalar::all(-1), cv::Scalar::all(-1),
                 drawMask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  return 1;
}
/*
 * Make a mat mask with the rect rigion set to 1
 */
int KLT_ORB_Tracker::getMask(cv::Rect RoI, cv::Mat mask){
    mask.setTo(0);
    //Mask value (0-255) can be set to change how many keypoints shall be detected. If we put 127 than approx half the amount of KP are detected
    cv::rectangle(mask,RoI,255,CV_FILLED);
    return 1;
}
/*
 * In-place edits the given mask and sets a rectangle region to zero
 */
void KLT_ORB_Tracker::blackOutMask(cv::Mat& mask8Bit,cv::Rect_<float> rect,float minDistance){
    cv::Rect_<float> blackOut(rect.x-minDistance,rect.y-minDistance,(rect.width+2*minDistance),(rect.height+2*minDistance));
    cv::rectangle(mask8Bit, blackOut,0,CV_FILLED,cv::LINE_8,0);
}
 /*
  * Settings
  */
int KLT_ORB_Tracker::setNfeatures(int number){
  ORBsettings.nfeatures = number;
  return 1;
}
