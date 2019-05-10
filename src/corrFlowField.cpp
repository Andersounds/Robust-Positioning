#include <iostream>
#include <opencv2/opencv.hpp>
/*
Maybe use opencv phase correlation????
Walter18 uses it

https://stackoverflow.com/questions/37143979/what-is-the-difference-between-phase-correlation-and-template-matching-in-opencv
See above. use phase correlation or
Split ROI into 9 or 4 non-overlapping sections. This is what  Walter18 does.

 Step 1: get perspective-adapted downward view. compensated for roll and pitch
 Step 2: Split into 4 or 9 parts.
 Step 3: perform phase correlation between each part to get 9 translations
    https://docs.opencv.org/3.0-alpha/modules/imgproc/doc/motion_analysis_and_object_tracking.html
 Step 4: This will then be a flow field that can be fed to the VO tracker as usual
*/
/*
TODO
 - filter the output. rotation is as usual pretty nice but translation is jumping
 - rewrite as proper hpp + cpp file
 - tilt correction
*/
class corrFlowField{

    // a function to warp away roll and tilt
    // a function to perform correlation and return rotation and translation

/* This function takes two mats and calculates the phase-correlation optical flow field
 *  They must already be pitch/roll compensated and in CV_8UC1 format
 */
public:
    float qualityLevel;

    int trackCorr(const cv::Mat& src1, const cv::Mat& src2, std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2){
        int success = 1;
        static int init = 0;
        if(!init){
            src2.copyTo(src1);
            init = 1;
        }
return 1;
        //success = deRotate()
        //success = corrFlow();
    }

    int corrFlow(const cv::Mat& src1, const cv::Mat& src2,
                    std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2){
        if(src1.rows<1 || src1.cols<1 || src2.rows<1 || src2.cols<1){return 0;}
        float res = 3;
        //Make sure that they are empty
        points1.clear();
        points2.clear();
        double thresh = 0;
        //Convert mats to float
        cv::Mat src1_32C1;
        cv::Mat src2_32C1;
        double scale = 2000;//Not fully scaled, but somewhere inbetween in order to avoid rounding errors
        double offset = 0;
        src1.convertTo(src1_32C1,CV_32FC1,scale,offset);
        src2.convertTo(src2_32C1,CV_32FC1,scale,offset);
//        cv::createHanningWindow(hann, src1_32C1.size(), CV_32FC1);
        //Calculate ROI size. This may be done in class before so that it does not have to be recalculated
        float h = ((float)src1.cols)/res;//h is width of subsquares. Right now make sure that src1 is proportional to 3 in widht/height
        cv::Mat hann;
        cv::createHanningWindow(hann,cv::Size(h,h), CV_32FC1);

        for(float y=0;y<res*h;y+=h){ //eller 2h?
            for(float x=0;x<res*h;x+=h){
                cv::Rect2f roi = cv::Rect2f(x,y,h,h);
                double response;
                cv::Point2d flow = cv::phaseCorrelate(src1_32C1(roi), src2_32C1(roi),hann,&response);
                cv::Point2f center = cv::Point2f(x+h/2,y+h/2);
                cv::Point2f flow32 = cv::Point2f((float)flow.x,(float)flow.y) + center;
                if(response<thresh){
                    std::cout << "Too low thresh" <<std::endl;
                    points1.clear();//Clear them so that VO does not process trash data
                    points2.clear();
                    return 0;}
                points1.push_back(center);                                      //Add the center coordinate of the current ROI
                points2.push_back(flow32);                                      //Add the corresponding flow vector
            }
        }

        return 1;
    }
};
