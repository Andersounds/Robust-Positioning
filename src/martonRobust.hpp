#ifndef MARTONROBUST_H
#define MARTONROBUST_H
/*
    This class implements a version of the robust trilateration method presented by L Marton et al (isbn: 9781509025916)
    Adaptation is done to fit the use of angulation instead of lateration
    Adaptions include:
        - Sensor measurement is 3d unit vector pointing towards anchor instead if single ToF measurement
        - Polynomial estimations are not only for x and y, but for z and yaw as well
        - The last equality of the cost function is adapted to
            - 3d via cone surface equation instead of 2d circle at known z
            - Another equality for yaw estimation
*/

#include <opencv2/opencv.hpp>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multifit_nlinear.h>

#include <math.h>       /* cos, pow */


namespace robustPositioning{
/*

*/
class martonRobust{
        int polyParameters;//How many parameters? ie what degree should the polynomial be?
        int bufferSize; //How many previous locations are to be saved in circular buffer? (min value dep on polyParameters)
    public:
        void process(void);// template for complete process method. add arguments and return type when clear
    private:
        int nlinear_lsqr_solve_2deg(void);     //Perform the nonlinear least square optimization Add arguments when known
    };


// Free cost and jacobian of cost frunctions
    struct poly2_data; // Data struct for gsl containing n: t: y:
    int poly2_f (const gsl_vector * x, void *data, gsl_vector * f); // Cost function for gsl_multifit_nlinear (2nd order polynomial)
    int poly2_df (const gsl_vector * x, void *data, gsl_matrix * J);// Jacobian of cost function for gsl-multifit_nlinear (2nd order polynomial)

}
#endif
