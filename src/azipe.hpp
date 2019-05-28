#ifndef AZIPE_H
#define AZIPE_H

/*
This class implements the AZIPE-algorithm, as presented in

J. Kim and H. Hmam, \3D Self-Localisation from Angle of Arrival Measurements,"
Weapons Systems Division, 2009.




*/

namespace az{

    bool azipe(const std::vector<cv::Mat_<float>>&,
                const std::vector<cv::Mat_<float>>&,
                cv::Mat_<float>&, float&,
                float,float);



}
#endif
