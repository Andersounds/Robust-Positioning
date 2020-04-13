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


namespace marton{
    const int X = 0;
    const int Y = 1;
    const int Z = 2;
    const int YAW = 3;
    /*
    Small class to act as a circular buffer to store old x,y,z,yaw,timestamps. To be passed to process-function
    */
    class circBuff{
        std::vector<float> p;   //Vector keeping xyz yaw as : [x_oldold,y_oldold,z_oldold,yaw_oldold,x_old,y_old,z_old,yaw_old,x,y,z,yaw]
        std::vector<float> t;   //Vector keeping t as [y_oldold,t_old,t]
        std::vector<float>::iterator p_it;
        std::vector<float>::iterator t_it;
    public:
        circBuff(int);//Constructor
        int add(const cv::Mat_<float>&,float,float); //Add x,y,x, yaw and timestamp
        int read_t(double*);
        int read_t_normed(double*);
        float read_T_offset(void);
        int read_p(double*);    // Functions to fill a c style array with the buffer values as doubles
        int read_p_normed(double*);
        float read_P_offset(int);
    };
    void process(void);// template for complete process method. add arguments and return type when clear
    void process(const std::vector<cv::Mat_<float>>& v,
                    const std::vector<cv::Mat_<float>>& q,
                    cv::Mat_<float>& position,
                    float& yaw,
                    float pitch,float roll, float t,circBuff&);

    int nlinear_lsqr_solve_2deg(void);     //Perform the nonlinear least square optimization Add arguments when known


// Free cost and jacobian of cost frunctions


/*
    Data struct with parameters that are given to solver
*/
struct nlinear_lsqr_param {
    double * x_init;     //Initial guess for variables
    double * weights;    //Variable weights
    double xtol;         //variable convergence criteria
    double gtol;
};
/*
    Data struct with parameters that are given to cost and jacobian functions later
    double p[12] = {x_oldoldold,y_oldoldold,z_oldoldold,yaw_oldoldold,
                    x_oldold,y_oldold,z_oldold,yaw_oldold,
                    x_old,y_old,z_old,yaw_old}
    double alpha[5] = {sx,sy,sz,vx,vy};
    double t[4] = {t_oldoldold,t_oldold,t_old,t_now};
*/
struct poly2_data {
    //size_t n;         //Use this to allow for more than one anchor later. then alpha can be passed as [sx1,sy1,sz1,vx1,vy1,vz1,sx2,sy2,sz2,vx2,vy2,vz2]
    //size_t m;         //Use this to vary number of previous known positions that are passed to solver
    double * p;         //Previous known positions [x1,y1,z1,yaw1,x2,y2,z2,yaw2,...]
    double * alpha;     //[sx,sy,sz,vx,vy] parameters to visible anchor.vx,vy is vector pointing towards anchor, in UAV frame! translate with T before passing.Maybe allow for more than one anchor later?
    double * t;         //Timestamps previous [t1 t2 t3]
    double tf;        //Timestamp current
};


    int poly2_f (const gsl_vector * x, void *data, gsl_vector * f); // Cost function for gsl_multifit_nlinear (2nd order polynomial)
    int poly2_df (const gsl_vector * x, void *data, gsl_matrix * J);// Jacobian of cost function for gsl-multifit_nlinear (2nd order polynomial)
    int nlinear_lsqr_solve_2deg(nlinear_lsqr_param,poly2_data);     //Perform the nonlinear least square optimization Add arguments when known

    cv::Mat getXRot(float);
    cv::Mat getYRot(float);
}
#endif
