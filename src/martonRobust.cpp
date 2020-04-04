


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
void robustPositioning::martonRobust::process(const std::vector<cv::Mat_<float>>& q,const std::vector<bool>& mask,const std::vector<float>& angles,float heightEst, float roll, float pitch, float& yaw, cv::Mat_<float>& pos){

    //Derotate (just subtract roll and pitch)
    //

}



/* This method takes one or more pixel locations, undistorts them, and calculate alpha, beta, and gamma angles
    alpha: apparent roll angle of uav if anchor were located directly below it and uav has 0 yaw angle
    beta : apparent pitch angle of uav if anchor were located directly below it and uav has 0 yaw angle
    gamma: apparent yaw of uav if anchor were located in direction of 0 yaw of uav if it has no roll or pitch

    These angles can then be de-rotated using the known current roll and pitch, and the single combined remaining angle
     is then used together with height estimation to estimate horizontal radial distance to anchor.
*/
void robustPositioning::martonRobust::pix2angles(const std::vector<cv::Point2f>& cornerLocations,std::vector<float>& angles){

}


/*Overloaded version of pix2uLOS. If corner locations of anchors are given instead of points directly, the mean of each vector<point2f> is considered
 *  as the anchors location. This is calculated and then the original pix2uLOS is called
 */
void robustPositioning::martonRobust::pix2angles(const std::vector<std::vector<cv::Point2f>>& cornerLocations,std::vector<float>& angles){
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
    return pix2angle(anchorLocations,angles);
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
struct robustPositioning::martonRobust::poly2_data {

};
/*
    Cost function to be passed to solver.
    x: arguments (in this case the polynomial parameters a,b,c in f = a + bt + ct^2)
    data: arbritary parameters given by us to solver that are passed to solver. In this case will it be t in the above equation. Also measuremed values and previous known locations
    f: The results of all cost functions shall be passed back to the solver via this vector
*/
int robustPositioning::martonRobust::poly2_f (const gsl_vector * x, void *data, gsl_vector * f){

// Read parameters from data-struct
//Dessa måste ges.
x, x_prev, x_prev_prev
y, y_prev, y_prev_prev
z, z_prev, z_prev_prev
yaw, yaw_prev, yaw_prev_prev
t, t_prev, t_prev_prev

Ekvationer 1-9 definieras på exakt samma sätt. Kan göra med ngn smartare matrismultiplikation eller iteratorer eller så

Samma sak med 10-12 antar jag om jag inte ska göra specialfall

Kom på hur ekvation 13 (kon-surface) och ekvation 14 (yaw)

//arguments
  double sx0 = gsl_vector_get (x, 0);
  double sx1 = gsl_vector_get (x, 1);
  double sx2 = gsl_vector_get (x, 2);

  double sy0 = gsl_vector_get (x, 3);
  double sy1 = gsl_vector_get (x, 4);
  double sy2 = gsl_vector_get (x, 5);

  double sz0 = gsl_vector_get (x, 6);
  double sz1 = gsl_vector_get (x, 7);
  double sz2 = gsl_vector_get (x, 8);

  double syaw0 = gsl_vector_get (x, 9);
  double syaw1 = gsl_vector_get (x, 10);
  double syaw2 = gsl_vector_get (x, 11);


  double f1 =

}
/*
    Analytic jacobian matrix of the above cost function
*/
int robustPositioning::martonRobust::poly2_df (const gsl_vector * x, void *data, gsl_matrix * J){

}
/*
    The function that sets up the problem and solves it using GSL nonlinear least square optimization method
    Code is adapted from first example at https://www.gnu.org/software/gsl/doc/html/nls.html
*/
int robustPositioning::martonRobust::nlinear_lsqr(void){

    const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
    gsl_multifit_nlinear_workspace *w;
    gsl_multifit_nlinear_fdf fdf;
    gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();
    const size_t n = N; //N is number of data points
    const size_t p = 3; // p is i guess number of parameters to solve for

    gsl_vector *f;
    gsl_matrix *J;
    gsl_matrix *covar = gsl_matrix_alloc (p, p);
    double t[N], y[N], weights[N];
    struct data d = { n, t, y };
    double x_init[3] = { 1.0, 1.0, 0.0 }; /* starting values */
    gsl_vector_view x = gsl_vector_view_array (x_init, p);
    gsl_vector_view wts = gsl_vector_view_array(weights, n);
    gsl_rng * r;
    double chisq, chisq0;
    int status, info;
    size_t i;

    const double xtol = 1e-8;
    const double gtol = 1e-8;
    const double ftol = 0.0;

    gsl_rng_env_setup();
    r = gsl_rng_alloc(gsl_rng_default);

    /* define the function to be minimized */
    fdf.f = expb_f;
    fdf.df = expb_df;   /* set to NULL for finite-difference Jacobian */
    fdf.fvv = NULL;     /* not using geodesic acceleration */
    fdf.n = n;
    fdf.p = p;
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
    w = gsl_multifit_nlinear_alloc (T, &fdf_params, n, p);

    /* initialize solver with starting point and weights */
    gsl_multifit_nlinear_winit (&x.vector, &wts.vector, &fdf, w);

    /* compute initial cost function */
    //f = gsl_multifit_nlinear_residual(w);
    //gsl_blas_ddot(f, f, &chisq0);

    /* solve the system with a maximum of 100 iterations */
    status = gsl_multifit_nlinear_driver(100, xtol, gtol, ftol,
                                         callback, NULL, &info, w);


}
