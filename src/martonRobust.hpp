#ifndef MARTONROBUST_H
#define MARTONROBUST_H
/*
    This class implements a version of the robust trilateration method presented by L Marton et al (isbn: 9781509025916)
    Adaptation is done to fit the use of angulation instead of lateration
    Adaptions include:
        - Angulation to visible anchor point is used together with a separate height estimation
            to find expression for possible circle in 3d
        - The final position estimation is used together with initial angulation data to estimate yaw angle
*/

namespace robustPositioning{
/*

*/
class martonRobust{
        cv::Mat_<float> K;                      //Camera matrix
        cv::Mat_<float> K_inv;                  //Inverted Camera matrix
        cv::Mat_<float> T;                      //uav to camera transformation matrix
        cv::Mat_<float> T_inv;                  //Inverted uav to camera transformation matrix. (=camera to uav in x-y-z order)
        float k1_barrel;                        //Distortion coefficients for barrel distortion
        float k2_barrel;
        float k3_barrel;
    public:
        void setKmat(cv::Mat_<float>);          //Camera matrix
        void setTmat(cv::Mat_<float>);          //uav -to- camera frame transformation matrix
        void setDistortCoefficents(float,float,float);//Set k1,k2,k3 distortion coefficients
        cv::Point2f unDistort(const cv::Point2f&);//Compensate for barrel distortion
        void pix2angles(const std::vector<cv::Point2f>&,std::vector<cv::Mat_<float>>&);//Converts image pixel coordinate(s) to apparent angles in x,y direction and around z
        void pix2angles(const std::vector<std::vector<cv::Point2f>>&,std::vector<cv::Mat_<float>>&);//Overloaded version for corner locations of anchors instead of center location

    };

}
#endif
