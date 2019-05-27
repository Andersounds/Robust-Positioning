#include <iostream>
#include <opencv2/opencv.hpp>
#include "opticalFlow.hpp"




/* Constructor. Calculates some sizes such as actual grid size
 *
 */
of::opticalFlow::opticalFlow(int mode_, int grid_, int roiSize){
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

        thresh = 0;
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
 *
 */
void of::opticalFlow::setDefaultSettings(void){


}
/* Wrapper function to calculate the optical flow. Calles either KLT or Corr depending on mode
 *
 */
bool of::opticalFlow::getFlow(const cv::Mat& src1, const cv::Mat& src2,
                std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2){
    if(mode == USE_KLT){
        return false;
    }else if(mode == USE_CORR){
        return corrFlow(src1,src2,points1,points2);
    } else{
        std::cout << "Invalid mode (of::opticalFlow::getFlow)" << std::endl;
        return false;
    }
}

/* This method implements a variant of Walter18:s phase correlation optical flow
 *
 */
bool of::opticalFlow::corrFlow(const cv::Mat& src1, const cv::Mat& src2,
                std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2){
    if(src1.rows<1 || src1.cols<1 || src2.rows<1 || src2.cols<1){return false;}//Assert that we actually have two RoI images
    //Assert mat type here?
    //Make sure that they are empty
    points1.clear();
    points2.clear();
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
        if(response<thresh){
            std::cout << "Too low threshold (of::opticalFlow::corrFlow)" <<std::endl;
            points1.clear();//Clear them so that VO does not process trash data
            points2.clear();
            return false;
        }
        points1.push_back(*cntr);
        cv::Point2f flow32 = cv::Point2f((float)flow.x,(float)flow.y) + *cntr;

        points2.push_back(flow32);
        cntr++;
    }
    return true;
}
