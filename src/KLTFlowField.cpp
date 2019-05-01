#include <iostream>
#include <opencv2/opencv.hpp>
/*
This is a class used to extract a flow field from an image sequance
Refactor this to h and c files
*/

class featureVO{
public:
    // Good features to track parameters
    double qualityLevel;
    double minDistance;
    int blockSize;
    double k;
    bool useHarris;
    //KLT track parameters
    int windowSize;
    int maxLevel;
    cv::TermCriteria termcrit;
    int flags;
    cv::Mat_<float> K;
    cv::Mat_<float> K_inv;

    //constructor
    /*featureVO(cv::Mat_<float> K_){
        K = cv::Mat_<float>::zeros(3,3);
        K_inv = K.inv();
    }*/

    //Methods
    void setFeatureSettings(double qualityLevel_, double minDistance_,int blockSize_,bool useHarris_,double k_){
        qualityLevel= qualityLevel_;
        minDistance = minDistance_;
        blockSize = blockSize_;
        useHarris = useHarris_;
        k = k_;
    }
    void setFeatureDefaultSettings(void){
        qualityLevel= 0.3;//0.5;
        minDistance = 10;//20
        blockSize = 4;//3;
        k = 0.04;
        useHarris = false;

    }
    void setKLTSettings(int windowSize_, int maxLevel_,cv::TermCriteria termcrit_,int flags_){
        windowSize = windowSize_;
        maxLevel = maxLevel_;
        termcrit = termcrit_;
        flags = flags_;
    }
    void setKLTDefaultSettings(void){
        flags = 0;//No flag. Use normal L2 norm error
        windowSize = 20;//51;
        maxLevel = 2;
        termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.01);
    }
    // Do this with a static mat that is shifted? so each time just supply the new frame?
    void trackKLT(cv::Mat prevFrame, cv::Mat frame,std::vector<cv::Point2f> corners,std::vector<cv::Point2f>& trackedCorners, std::vector<uchar>& status,std::vector<float>& errors){
        static int init = 0;
        if(!init){
            frame.copyTo(prevFrame);
            init = 1;
        }
        if(!(corners.size()>0)){return;}
        cv::calcOpticalFlowPyrLK(prevFrame,frame,corners,trackedCorners,
                                status,
                                errors,
                                cv::Size(windowSize,windowSize),
                                maxLevel,
                                termcrit,
                                flags,//For special operation
                                0.001);
    }
    //void getCorners(cv::Mat frame, std::vector<cv::Point2f>& corners, int noOfCorners,cv::Rect_<float> maskRect){
    //    cv::Mat mask;
    //    cv::goodFeaturesToTrack(frame(maskRect), corners, noOfCorners, qualityLevel, minDistance,mask,blockSize,useHarris,k);
    //}
    void getCorners(cv::Mat& frame, std::vector<cv::Point2f>& corners,int noOfCorners){
        cv::Mat mask;
        cv::goodFeaturesToTrack(frame, corners, noOfCorners, qualityLevel, minDistance,mask,blockSize,useHarris,k);
    }
    /* This method returns a sub-area of the focusarea
     *  The subarea is the intersection between the focusarea and an equal but shifted rect
     * TODO
     */
    cv::Rect_<float> getDirMask(cv::Rect_<float> focusArea, cv::Point2f dir){
        cv::Rect_<float> newRect = focusArea;//Shift it here
        cv::Rect_<float> dirMask = focusArea & newRect;
        return dirMask;
    }
    /*This method is possibly unnecessary
    */
    cv::Rect_<float> getFocusArea(int frameCols,int frameRows,int width,int height){
        float x = (float)(frameCols-width)/2.0;
        float y = (float)(frameRows-height)/2.0;
        cv::Rect_<float> rect(x,y,(float) width, (float) height);
        return rect;
    }
    /* This method shifts all coordinates related to the focusarea to image coordinates
    */
    void coordShiftFeatures(cv::Rect_<float> focusArea,std::vector<cv::Point2f>& updatedFeatures){
        std::vector<cv::Point2f>::iterator it = updatedFeatures.begin();
        while(it!=updatedFeatures.end()){
            it->x += focusArea.x;
            it->y += focusArea.y;
            it++;
        }
    }
    /*
     * NOTE newFeatures should be empty as the function pushes the points back onto it
     */
    int extractActiveFeatures(std::vector<cv::Point2f> features,std::vector<uchar> status,std::vector<cv::Point2f>& newFeatures,cv::Point2f offset){
        newFeatures.clear();
        std::vector<uchar>::iterator statusIt = status.begin();
        std::vector<cv::Point2f>::iterator featureIt = features.begin();
        int counter = 0;
        while(statusIt!=status.end()){
            if(*statusIt){//If feature is active
                newFeatures.push_back(*featureIt + offset);
                counter++;
            }
            statusIt++;
            featureIt++;
        }
        return counter;
    }
    /*This method concatenates the add-vector onto the base vector
     *
     */
    void conCat(std::vector<cv::Point2f>& base, std::vector<cv::Point2f> add){
        std::vector<cv::Point2f>::iterator it = add.begin();
        while(it!=add.end()){
            base.push_back(*it);
            it++;
        }
    }
};
