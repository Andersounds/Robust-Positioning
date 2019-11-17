#ifndef AZIPE_H
#define AZIPE_H

/*
This class implements the AZIPE-algorithm, as presented in

J. Kim and H. Hmam, \3D Self-Localisation from Angle of Arrival Measurements,"
Weapons Systems Division, 2009.




*/

namespace az{
    const int AZIPE_FAIL = 2;
    const int AZIPE_SUCCESS = 3;
    const float PI = 3.1416;


    int azipe(const std::vector<cv::Mat_<float>>&,
                const std::vector<cv::Mat_<float>>&,
                cv::Mat_<float>&, float&,
                float,float);
    int aipe(const std::vector<cv::Mat_<float>>&,
                const std::vector<cv::Mat_<float>>&,
                cv::Mat_<float>&, float&, float&, float&,
                float);
    bool aipe_solvable(const std::vector<cv::Mat_<float>>&, float);
    float limitYawRange(float);


}
#endif
