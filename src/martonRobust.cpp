#include <iostream>
#include "martonRobust.hpp"

/*
Methods for defining the input arguments are defined in the ang::angulation class
    src/angulation.hpp
    src/angulation.cpp
*/




/* This is the main process method that handles the estimation
 * Only first of the known anchors will be used in first implementation
 * vector<Mat> v        A set of unit-Line-of-Sight vectors in the vehicle frame (measured) to each of the visible anchors
 * vector<Mat> q        A corresponding set of 3d-coordinates of each anchor in the global frame
 * float    roll   A value representing the roll of the vehicle as related to the global frame expressed in radians.
 * float   pitch   A value representing the pitch of the vehicle as related to the global frame expressed in radians.

 */
//void robustPositioning::martonRobust::process(const std::vector<cv::Mat_<float>>& q,const std::vector<bool>& mask,const std::vector<float>& angles, float roll, float pitch, float& yaw, cv::Mat_<float>& pos){

    //Derotate (just subtract roll and pitch)
    //

//}

/*
Keep same interface as azipe. v,q,pos,yaw,pitch,roll
Possibly keep list of previous positions here. or keep thet externally and give as argument?

*/
void robustPositioning::martonRobust::process(void){


    int status = nlinear_lsqr_solve_2deg();
    std::cout << "Status: " << status << std::endl;
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
// x order: [sigmax0,sigmax1,sigmax2,sigmay0,sigmay1,sigmay2...]  Yttre ordgning: x,y,z,yaw, inre ordnnig: stigande grad
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
    double p[12] = {0.4,1,1,0,
                    9.4,2,2,0,
                    26.4,3,3,0};
    double alpha[5] = {0,0,0,0.4,0.3};
    double t[4] = {0,1,2,3};


    struct poly2_data d = { p, alpha, t };
    double x_init[12] = { 1, 0, 0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0}; /* starting values. Maybe init these as last solution*/
    double weights[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    gsl_vector_view x = gsl_vector_view_array (x_init, par);
    gsl_vector_view wts = gsl_vector_view_array(weights, n);
    //gsl_rng * r;
    //double chisq, chisq0;
    //int status, info;
    //size_t i;

    const double xtol = 1e-2;
    const double gtol = 1e-2;
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


    std::cout << "Polynomial: ";
    for(size_t i=0;i<12;i++){
        double xi = gsl_vector_get(w->x, i);
        std::cout << xi << ", ";
    }
        std::cout << std::endl;

    switch(status){
        case(GSL_SUCCESS):{
            std::cout << " Success" << std::endl;
            std::cout << "Info: " << info <<  std::endl;
            std::cout << "Iterations: " << gsl_multifit_nlinear_niter(w) << std::endl;
            break;
        }
        case(GSL_ENOPROG):{
            std::cout << " Could not find" <<std::endl;
            break;
        }
    }
    return status;

}
