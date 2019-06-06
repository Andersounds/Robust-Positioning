#include <iostream>
#include <opencv2/opencv.hpp>
#include "opticalFlow.hpp"




/* Constructor. Calculates some sizes such as actual grid size
 *
 */
of::opticalFlow::opticalFlow(int mode_, int grid_, int roiSize){
    init = false;
    mode = mode_;
    grid = grid_;
    //Calculate integer distance between each flow arrow
    h = floor( (float)roiSize / (float) grid );
    grid_x = floor(( ((float)roiSize) - h *((float)grid) + h)/2); //Calculate center (integer) of first roi
    grid_y = grid_x;

    //Define p1 vector to be used in each function call
    std::vector<cv::Point2f> p1_;
    float y = grid_y;
    for(int i=0;i<grid;i++){
        float x = grid_x;
        for(int j=0;j<grid;j++){
            p1_.push_back(cv::Point2f(x,y));
            x += h;

        }
        y += h;

    }

    //Assign p1 corresponcence points
    p1 = p1_;
    //Init parameters for the chosen optical flow method
    if(mode == USE_KLT){


    }else if(mode == USE_CORR){
        //Define subRoI vector
        std::vector<cv::Rect2f> subRoI_;
        float h_half = floor( h/2 );
        for(int i=0;i<p1.size();i++){
            float x = p1[i].x - h_half;
            float y = p1[i].y - h_half;
            subRoI_.push_back(cv::Rect2f(x,y,h,h));
        }
        subRoI = subRoI_;
        //Create hanning window to reduce edge effects in phase correlation calculation
        cv::Mat hann_;
        cv::createHanningWindow(hann_,cv::Size(h,h), CV_32FC1);
        hann_.copyTo(hann);
    } else{
        std::cout << "Wrong optical flow mode (opticalFlow::opticalFlow)" << std::endl;
        //cv::error();
    }
}
/* Used to set default settings for the optical flow algorithms
 * TODO: maybe check that windowsize is small enough for the grid used?
 */
void of::opticalFlow::setDefaultSettings(void){
    if(mode == USE_KLT){
        flags = 0;//No flag. Use normal L2 norm error
        windowSize = 51;//51;
        maxLevel = 2;
        termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.01);
        init = true;
    } else if(mode == USE_CORR){
        corrQualityLevel = 0.3;
        init = true;
    }


}
/* Wrapper function to calculate the optical flow. Calles either KLT or Corr depending on mode
 *
 */
int of::opticalFlow::getFlow(const cv::Mat& src1, const cv::Mat& src2,
                std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2){
    //Check that initialized
    if(!init){std::cout << "optical flow not initialized. run (of::opticalFlow::setDefaultSettings)"<<std::endl;return 0;}
    //Check that data is fine
    if(src1.rows<1 || src1.cols<1 || src2.rows<1 || src2.cols<1){return false;}//Assert that we actually have two RoI images
    //-----------Assert mat type here?
    //Make sure that they are empty
    points1.clear();
    points2.clear();
    if(mode == USE_KLT){
        return KLTFlow(src1,src2,points1,points2);
    }else if(mode == USE_CORR){
        return corrFlow(src1,src2,points1,points2);
    } else{
        std::cout << "Invalid mode (of::opticalFlow::getFlow)" << std::endl;
        return 0;
    }
}

/* This method implements a variant of Walter18:s phase correlation optical flow
 * TODO: use quality level and return parital flow field if some is not successful
 * TODO: Check that images are converted correctly. When printing they look very white do we loose information?
 * TODO: filter the flow field?
 */
int of::opticalFlow::corrFlow(const cv::Mat& src1, const cv::Mat& src2,
                std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2){
    int success = 0;
    //Convert mats to float
    cv::Mat src1_32C1;
    cv::Mat src2_32C1;
    double scale = 2048;//Not fully scaled, but somewhere inbetween in order to avoid rounding errors
    double offset = 0;
    src1.convertTo(src1_32C1,CV_32FC1,scale,offset);
    src2.convertTo(src2_32C1,CV_32FC1,scale,offset);

    std::vector<cv::Point2f>::iterator cntr = p1.begin();
    for(cv::Rect2f roi:subRoI){
        double response;
//std::cout << "is sub roi converted correctly?" << std::endl;
//cv::imshow("Chess board", src1_32C1);
//cv::waitKey(0);
        cv::Point2d flow = cv::phaseCorrelate(src1_32C1(roi), src2_32C1(roi),hann,&response);//Perform phase correlation calculation
        if(response >= corrQualityLevel){
            points1.push_back(*cntr);
            cv::Point2f flow32 = cv::Point2f((float)flow.x,(float)flow.y) + *cntr;
            points2.push_back(flow32);
            success++;
        }
        cntr++;
    }
    return success;
}

/* This method implements a pyramidal KLT  optical flow algorithm without feature searching
 * i.e features to track is not found with goodFeaturesToTrack or similar. They are specified beforehand
 * to be located on a n-by-n grid. This methods proved to be more robust in low contrast environments in initial tests
 */
int of::opticalFlow::KLTFlow(const cv::Mat& src1, const cv::Mat& src2,
                std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2){
    std::vector<uchar> status;
    std::vector<float> errors;
    std::vector<cv::Point2f> p1_ = p1;
    std::vector<cv::Point2f> p2;

    cv::calcOpticalFlowPyrLK(src1,src2,p1_,p2,
                            status,
                            errors,
                            cv::Size(windowSize,windowSize),
                            maxLevel,
                            termcrit,
                            flags,//For special operation
                            0.001);
    //Only return the flow corresponcences that have no error. Go through the vectors returned by PyrLK
    std::vector<uchar>::iterator it0 = status.begin();
    std::vector<cv::Point2f>::iterator it1 = p1.begin();
    std::vector<cv::Point2f>::iterator it2 = p2.begin();
    int success = 0;
    while( it0 != status.end()){
        if(*it0){ //If tracking was successful
            points1.push_back(*it1);
            points2.push_back(*it2);
            success++;
        }
        it0++;
        it1++;
        it2++;
    }
    return success;
}

/* Draws arrows between the point correspondances and scale them
 *
 */
void of::opticalFlow::drawArrows(cv::Mat& outputImg,std::vector<cv::Point2f> features1,std::vector<cv::Point2f> features2,float scale, cv::Point2f offset){
    std::vector<cv::Point2f>::iterator it1 = features1.begin();
    std::vector<cv::Point2f>::iterator it2 = features2.begin();
    while(it1!=features1.end()){

        cv::Point2f from = *it1 + offset;
        cv::Point2f to = (*it2 - *it1)*scale + *it1 + offset;
        cv::arrowedLine(outputImg,from,to,CV_RGB(200,50,0),2,cv::LINE_8,0,0.1);
        //cv::imshow("Chess board", outputImg);
        //cv::waitKey(0);
        it1++;
        it2++;
    }
}
