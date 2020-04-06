#include <iostream>
#include <opencv2/opencv.hpp>
// Libraries for gsl nonlinear least squared optimization
//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist.h>
//#include <gsl/gsl_matrix.h> // Already defined in header
//#include <gsl/gsl_vector.h> // Already defined in header
//#include <gsl/gsl_blas.h>
//#include <gsl/gsl_multifit_nlinear.h> // Already defined in header

#include <math.h>       /* cos, pow */
#include "martonRobust.hpp"


/* This is the main process method that handles the estimation
 *      q: q vectors (anchor coordinates), of which the first known one as indicated by mask will be used.
 *   mask: Mask that indicates which anchor to use
 * angles: alpha, beta, gamma angles as calculated from pix2angles
 * heightEst: estimated z coordinate in global system. Etimation read from atsam
 * roll, pitch: measured roll and pitch of uav.
 * outputs:
 * yaw:     yaw angle of uav estimation
 * pos:     global coordinate of uav
 */
//void robustPositioning::martonRobust::process(const std::vector<cv::Mat_<float>>& q,const std::vector<bool>& mask,const std::vector<float>& angles,float heightEst, float roll, float pitch, float& yaw, cv::Mat_<float>& pos){

    //Derotate (just subtract roll and pitch)
    //

//}



/* This method takes one or more pixel locations, undistorts them, and calculate alpha, beta, and gamma angles
    alpha: apparent roll angle of uav if anchor were located directly below it and uav has 0 yaw angle
    beta : apparent pitch angle of uav if anchor were located directly below it and uav has 0 yaw angle
    gamma: apparent yaw of uav if anchor were located in direction of 0 yaw of uav if it has no roll or pitch

    These angles can then be de-rotated using the known current roll and pitch, and the single combined remaining angle
     is then used together with height estimation to estimate horizontal radial distance to anchor.
*/
void robustPositioning::martonRobust::pix2angles(const std::vector<cv::Point2f>& cornerLocations,std::vector<cv::Mat_<float>>& angles){

}


/*Overloaded version of pix2uLOS. If corner locations of anchors are given instead of points directly, the mean of each vector<point2f> is considered
 *  as the anchors location. This is calculated and then the original pix2uLOS is called
 */
void robustPositioning::martonRobust::pix2angles(const std::vector<std::vector<cv::Point2f>>& cornerLocations,std::vector<cv::Mat_<float>>& angles){
    std::vector<cv::Point2f> anchorLocations;
    for(int i=0;i<cornerLocations.size();i++){//Go through all anchors
        std::vector<cv::Point2f>::const_iterator cornerIt = cornerLocations[i].begin();
        cv::Point2f location(0,0);
        while(cornerIt != cornerLocations[i].end()){
            location += *cornerIt;
            cornerIt++;
        }
        location.x /= cornerLocations[i].size();
        location.y /= cornerLocations[i].size();
        anchorLocations.push_back(location);
    }
    //Do standard function call to calculate
    return pix2angles(anchorLocations,angles);
}


/* method to perform in-place correction of barrel distortion of pixel coordinates
 * Using the following formula: https://docs.opencv.org/3.4.0/d4/d94/tutorial_camera_calibration.html
 */
cv::Point2f robustPositioning::martonRobust::unDistort(const cv::Point2f& point){
    //Shift coordinate system center to middle of image
    float x = point.x - K(0,2);
    float y = point.y - K(1,2);
    //Perform undistortion
    float r2 = x*x + y+y;//Radius squared
    float A = (1 + k1_barrel*r2 + k2_barrel*r2*r2 + k3_barrel*r2*r2*r2);
    cv::Point2f undistortedPoint(x/A,y/A);//SHOULD IT BE MULTIPLIED? docs are not definitive. Different in diferent version of docs
    //Shift coordinates back to image
    undistortedPoint.x += K(0,2);
    undistortedPoint.y += K(1,2);
    return undistortedPoint;
}
/* Interface mathod used to set K mat and calculate its inverse
 */
void robustPositioning::martonRobust::setKmat(cv::Mat_<float>K_){
    K = K_;
    K_inv = K.inv();
}
/* Interface method used to set T mat. T is transformation fro  uav frame to camera frame
 */
void robustPositioning::martonRobust::setTmat(cv::Mat_<float> T_){
    T = T_;
    T_inv = T.inv();
}
/* Interface method to set distortion coefficients for barrel distortion compensation
*/
void robustPositioning::martonRobust::setDistortCoefficents(float k1, float k2, float k3){
    k1_barrel = k1;
    k2_barrel = k2;
    k3_barrel = k3;
}


/*
Below are function definitions for Marton robust positioning algorithm



*/

/*
    Data struct with parameters that are given to cost and jacobian functions later
*/
struct robustPositioning::poly2_data {
    //size_t n;         //Use this to allow for more than one anchor later. then alpha can be passed as [sx1,sy1,sz1,vx1,vy1,vz1,sx2,sy2,sz2,vx2,vy2,vz2]
    //size_t m;         //Use this to vary number of previous known positions that are passed to solver
    double * p;         //Previous known positions [x1,y1,z1,yaw1,x2,y2,z2,yaw2,...]
    double * alpha;     //[sx,sy,sz,vx,vy] parameters to visible anchor.vx,vy is vector pointing towards anchor, in UAV frame! translate with T before passing.Maybe allow for more than one anchor later?
    double * t;         //Timestamps, current and previous [t1 t2 t3 tf]
};
/*
    Cost function to be passed to solver.
    x: arguments (in this case the polynomial parameters a,b,c in f = a + bt + ct^2)
    data: arbritary parameters given by us to solver that are passed to solver. In this case will it be t in the above equation. Also measuremed values and previous known locations
    f: The results of all cost functions shall be passed back to the solver via this vector
*/
int robustPositioning::poly2_f (const gsl_vector * x, void *data, gsl_vector * f){
// Read parameters from data-struct
double *p = ((struct poly2_data *)data)->p;
double *alpha = ((struct poly2_data *)data)->alpha;
double *t = ((struct poly2_data *)data)->t;
// Get arguments to a normal vector for easy access
double x_[12];
for(size_t i = 0;i<12;i++){
    x_[i] = gsl_vector_get(x,i);
}

/* Equations 0-11 */
// T order: [t1,t2,t3,tf]  kronologisk
// p order: [x1,y1,z1,yaw1,x2,y2,z2,yaw2,...] Yttre ordning: kronologisk, inre ordning: x,y,z,yaw
// x order: [sigmax0,sigmax1,sigmax2,sigmay0,sigmay1,sigmay2]  Yttre ordgning: x,y,z,yaw, inre ordnnig: stigande grad
for (size_t i = 0;i < 3; i++){
    // Calculate time stamp powers
    double t_i = t[i];
    double t_i_sqrd = pow(t_i,(double)2);
    int offset = i*4;
    gsl_vector_set (f, 0+offset, x_[0]+x_[1]*t_i+x_[2]*t_i_sqrd - p[0+offset]); //X for time i
    gsl_vector_set (f, 1+offset, x_[3]+x_[4]*t_i+x_[5]*t_i_sqrd - p[1+offset]); //Y for time i
    gsl_vector_set (f, 2+offset, x_[6]+x_[7]*t_i+x_[8]*t_i_sqrd - p[2+offset]); //Z for time i
    gsl_vector_set (f, 3+offset, x_[9]+x_[10]*t_i+x_[11]*t_i_sqrd - p[3+offset]); //Yaw for time i
}
/*Equation 12 - Hardcoded position 13. keep counter if allow for more previous data points*/
    double c = (pow(alpha[3],(double)2) + pow(alpha[4],(double)2))/(pow(alpha[5],(double)2)); // to use in cone equation x^2+y^2 = c*z^2
    double t_f = t[3];      //Current time stamp (tf = time of failure)
    double t_f_sqrd = pow(t_f,(double)2);
    //Evaluate coordinate with current provided sigma-parameters
    double x_est = x_[0]+x_[1]*t_f+x_[2]*t_f_sqrd;
    double y_est = x_[3]+x_[4]*t_f+x_[5]*t_f_sqrd;
    double z_est = x_[6]+x_[7]*t_f+x_[8]*t_f_sqrd;
    double f12 = pow(x_est-alpha[0],(double)2) + pow(y_est - alpha[1],(double)2) - c*pow(z_est - alpha[2],(double)2);
    gsl_vector_set (f, 12, f12);
/*Equation 13 - Hardcoded position 14. keep counter if allow for more previous data points*/
    double yaw_est = x_[9]+x_[10]*t_f+x_[11]*t_f_sqrd;
    double est_norm = sqrt(pow(x_est,(double)2)+pow(y_est,(double)2));
    double v_norm = sqrt(pow(alpha[3],(double)2) + pow(alpha[4],(double)2));
    double cos_ = cos(yaw_est);
    double sin_ = sin(yaw_est);

    double v_est_x = cos_*alpha[3] - sin_*alpha[4];//v vector rotated with the estimated yaw
    double v_est_y = sin_*alpha[3] + cos_*alpha[4];
    double f13 = pow(x_est/est_norm  + v_est_x/v_norm,(double)2) + pow(y_est/est_norm + v_est_y/v_norm,(double)2);
    gsl_vector_set (f, 12, f13);

    return GSL_SUCCESS;
}
/*
    Analytic jacobian matrix of the above cost function
*/
int robustPositioning::poly2_df (const gsl_vector * x, void *data, gsl_matrix * J){
return GSL_SUCCESS;
}
/*
    The function that sets up the problem and solves it using GSL nonlinear least square optimization method
    Code is adapted from first example at https://www.gnu.org/software/gsl/doc/html/nls.html
*/
int robustPositioning::martonRobust::nlinear_lsqr_solve_2deg(void){

    const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
    gsl_multifit_nlinear_workspace *w;
    gsl_multifit_nlinear_fdf fdf;
    gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();
    const size_t n = 14; //N is number of data points
    const size_t par = 12; // p is i guess number of parameters to solve for

    gsl_vector *f;
    gsl_matrix *J;
    //gsl_matrix *covar = gsl_matrix_alloc (p, p);
    //p: 12 previous data point (3x{x,y,z,yaw}| alpha: one anchor [sz,sy,sz,vx,vy] | t: 4 timestamps ([t_oldoldold,t_oldold,t_old,t_now))
/*
    double p[12] = {x_oldoldold,y_oldoldold,z_oldoldold,yaw_oldoldold,
                    x_oldold,y_oldold,z_oldold,yaw_oldold,
                    x_old,y_old,z_old,yaw_old}
    double alpha[5] = {sx,sy,sz,vx,vy};
    double t[4] = {t_oldoldold,t_oldold,t_old,t_now};
*/
    double p[12] = {1,1,1,
                    1,1,1,
                    1,1,1,
                    1,1,1};
    double alpha[5] = {0,0,0,0.4,0.3};
    double t[4] = {0,0.1,0.4,0.5};


    struct poly2_data d = { p, alpha, t };
    double x_init[12] = { 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0}; /* starting values. Maybe init these as last solution*/
    double weights[12];
    gsl_vector_view x = gsl_vector_view_array (x_init, par);
    gsl_vector_view wts = gsl_vector_view_array(weights, n);
    //gsl_rng * r;
    //double chisq, chisq0;
    //int status, info;
    //size_t i;

    const double xtol = 1e-8;
    const double gtol = 1e-8;
    const double ftol = 0.0;

    //gsl_rng_env_setup();
    //r = gsl_rng_alloc(gsl_rng_default);

    /* define the function to be minimized */
    fdf.f = robustPositioning::poly2_f;
    fdf.df = NULL; //poly2_df;   /* set to NULL for finite-difference Jacobian */
    fdf.fvv = NULL;     /* not using geodesic acceleration */
    fdf.n = n; //the number of functions, i.e. the number of components of the vector f.
    fdf.p = par;  //the number of independent variables, i.e. the number of components of the vector x.
    fdf.params = &d;

    /* this is the data to be fitted */
    //for (i = 0; i < n; i++)
    //  {
    //    double ti = i * TMAX / (n - 1.0);
    //    double yi = 1.0 + 5 * exp (-0.1 * ti);
    //    double si = 0.1 * yi;
    //    double dy = gsl_ran_gaussian(r, si);

    //    t[i] = ti;
    //    y[i] = yi + dy;
    //    weights[i] = 1.0 / (si * si);
    //  };

    /* allocate workspace with default parameters */
    w = gsl_multifit_nlinear_alloc (T, &fdf_params, n, par);

    /* initialize solver with starting point and weights */
    gsl_multifit_nlinear_winit (&x.vector, &wts.vector, &fdf, w);

    /* compute initial cost function */
    //f = gsl_multifit_nlinear_residual(w);
    //gsl_blas_ddot(f, f, &chisq0);

    /* solve the system with a maximum of 100 iterations */
    int status,info;
    status = gsl_multifit_nlinear_driver(100, xtol, gtol, ftol,
                                         NULL, NULL, &info, w);
    return status;

}
