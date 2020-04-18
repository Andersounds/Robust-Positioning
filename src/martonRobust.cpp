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

int marton::process(const std::vector<cv::Mat_<float>>& v,
            const std::vector<cv::Mat_<float>>& q,
            cv::Mat_<float>& position,
            float& yaw,
            float pitch,float roll, float tf,
            const std::vector<float>& pPrev, const std::vector<float>& tPrev){

                /*
                X(0.)    Extract only first v and q. In future maybe allow more
                X 1.     Derotate v using pitch and roll
                X 2.     Construct alpha from v and q (cast to double also)
                X 3.     Norm previous timetamps so that oldest is 0. construct t
                X 4.     Norm x,y,z,yaw so that oldes is 0. This to get more consistent parameters in case we use as start guess construct p
                X 5.     construct poly2_data
                X 6.     Give initial guess via inputOutput argument.
                X 7.     possibly give weights,xtol,ftol, iterations via a struct
                 8.     Perform nonlinear least square optimization
                 9.     If successful, pass back position and yaw estimation, otherwise some fail code
                */
                //1.
                std::cout <<" T4: [" << tPrev[0] << ", " << tPrev[1]<<", " << tPrev[2] <<"]" << std::endl;
                cv::Mat Rx = marton::getXRot(roll);
                cv::Mat Ry = marton::getYRot(pitch);
                cv::Mat_<float> q0 = q[0];
                cv::Mat_<float> v0 = Ry*Rx*v[0];//Derotate v vector back in reverse order. (roll-pitch-yaw instead of yaw-pitch-roll)
                v0.reshape(1);//Reshape to column to make sure that we access corret elements below
                q0.reshape(1);
                double alpha[6];
                alpha[0] = (double)q0(0,0);
                alpha[1] = (double)q0(1,0);
                alpha[2] = (double)q0(2,0);
                alpha[3] = (double)v0(0,0);
                alpha[4] = (double)v0(1,0);
                alpha[5] = (double)v0(2,0);


                // Convert vector<float> of previous values to normalized double array and save offsets
                int size_tPrev = tPrev.size();
                double tPrev_normed[size_tPrev];
                //float t_offset = tPrev[0];
                for(int i=0;i<size_tPrev;i++){
                    float element = tPrev[i]-tPrev[0];//Offset with first value
                    tPrev_normed[i] = (double)element;
                }
                float tf_normed = tf-tPrev[0];
                double tf_d_normed = (double)tf_normed;


                int size_pPrev = pPrev.size();
                double pPrev_normed[size_pPrev];
                //Offset?
                for(int i=0;i<size_pPrev;i++){
                    float element = pPrev[i] - pPrev[i%4];//Offset with first 4 values
                    pPrev_normed[i] = (double)element;
                }

                std::cout <<" T4_normed: [" << tPrev_normed[0] << ", " << tPrev_normed[1]<<", " << tPrev_normed[2] <<"]" << std::endl;
                // Get values from prevBuffer and construct arrays
            //    double tPrev_N[3];double pPrev_N[12];
            //    prevBuffer.read_t_normed(tPrev_N);
            //    prevBuffer.read_p_normed(pPrev_N);
                // Construct poly2_data
                //float tf_normed = tf-prevBuffer.read_T_offset();
                //std::cout << "TF NORMED::::::::::::::::::::::::::: " << tf_normed << std::endl;
                //std::cout << "::::: [" << tPrev_N[0] << ", " << tPrev_N[1]<< ", " << tPrev_N[2] << std::endl;

                //double tf_d_normed = (double)tf_normed;

                struct marton::poly2_data da = {pPrev_normed, alpha, tPrev_normed,tf_d_normed};
                // Construct solver parameters struct
                double x_init[12] = { 1, 0, 0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0}; /* starting values. Maybe init these as last solution*/
                double weights[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                double xtol = 1e-2;
                double gtol = 1e-2;
                struct marton::nlinear_lsqr_param param = {x_init,weights,xtol,gtol};
                // Perform optimization. pass d and param. Hur skicka structs som argument?
                int status = marton::nlinear_lsqr_solve_2deg(param,da,position,yaw);

            //    std::cout << "Adapt offset here if successful" << std::endl;


                // Calculate new position
                return status;
}


/*
Below are function definitions for Marton robust positioning algorithm



*/


/*
    Cost function to be passed to solver.
    x: arguments (in this case the polynomial parameters a,b,c in f = a + bt + ct^2)
    data: arbritary parameters given by us to solver that are passed to solver. In this case will it be t in the above equation. Also measuremed values and previous known locations
    f: The results of all cost functions shall be passed back to the solver via this vector
*/
int marton::poly2_f (const gsl_vector * x, void *data, gsl_vector * f){
    // Read parameters from data-struct
    double *p = ((struct poly2_data *)data)->p;
    double *alpha = ((struct poly2_data *)data)->alpha;
    double *t = ((struct poly2_data *)data)->t;
    double tf = ((struct poly2_data *)data)->tf;

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
    double t_f = tf;      //Current time stamp (tf = time of failure)
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
int marton::poly2_df (const gsl_vector * x, void *data, gsl_matrix * J){
return GSL_SUCCESS;
}



/*
    The function that sets up the problem and solves it using GSL nonlinear least square optimization method
    Code is adapted from first example at https://www.gnu.org/software/gsl/doc/html/nls.html
*/

int marton::nlinear_lsqr_solve_2deg(nlinear_lsqr_param parameters, poly2_data data,cv::Mat_<float>& position,float& yaw){

    const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
    gsl_multifit_nlinear_workspace *w;
    gsl_multifit_nlinear_fdf fdf;
    gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();
    const size_t n = 14; //N is number of data points
    const size_t par = 12; // p is i guess number of parameters to solve for

    //gsl_vector *f;
    //gsl_matrix *J;

    //struct poly2_data d = { p, alpha, t,tf};
    //double x_init[12] = { 1, 0, 0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0}; /* starting values. Maybe init these as last solution*/
    double *x_init = parameters.x_init;
    //double weights[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    double *weights = parameters.weights;
    gsl_vector_view x = gsl_vector_view_array (x_init, par);
    gsl_vector_view wts = gsl_vector_view_array(weights, n);

    //const double xtol = 1e-2;
    double xtol = parameters.xtol;
    //const double gtol = 1e-2;
    double gtol = parameters.gtol;
    const double ftol = 0.0;

    /* define the function to be minimized */
    fdf.f = marton::poly2_f;
    fdf.df = NULL; //poly2_df;   /* set to NULL for finite-difference Jacobian */
    fdf.fvv = NULL;     /* not using geodesic acceleration */
    fdf.n = n; //the number of functions, i.e. the number of components of the vector f.
    fdf.p = par;  //the number of independent variables, i.e. the number of components of the vector x.
    fdf.params = &data;
    /* allocate workspace with default parameters */
    w = gsl_multifit_nlinear_alloc (T, &fdf_params, n, par);

    /* initialize solver with starting point and weights */
    gsl_multifit_nlinear_winit (&x.vector, &wts.vector, &fdf, w);

    /* solve the system with a maximum of 100 iterations */
    int status,info;
    status = gsl_multifit_nlinear_driver(100, xtol, gtol, ftol,
                                         NULL, NULL, &info, w);




    /*std::cout << "Polynomial: ";
    for(size_t i=0;i<12;i++){
        double xi = gsl_vector_get(w->x, i);
        std::cout << xi << ", ";
    }
        std::cout << std::endl;
*/
    switch(status){
        /* Calculate new position using the polynomial */
        case(GSL_SUCCESS):{
            std::cout << " Success" << std::endl;
            std::cout << "Info: " << info <<  std::endl;
            std::cout << "Iterations: " << gsl_multifit_nlinear_niter(w) << std::endl;
            double tf_d = data.tf;
            double tf_d2 = tf_d*tf_d;
            double xf = gsl_vector_get(w->x, 0) + gsl_vector_get(w->x, 1)*tf_d + gsl_vector_get(w->x, 2)*tf_d2;
            double yf = gsl_vector_get(w->x, 3) + gsl_vector_get(w->x, 4)*tf_d + gsl_vector_get(w->x, 5)*tf_d2;
            double zf = gsl_vector_get(w->x, 6) + gsl_vector_get(w->x, 7)*tf_d + gsl_vector_get(w->x, 8)*tf_d2;
            double yawf = gsl_vector_get(w->x, 9) + gsl_vector_get(w->x, 10)*tf_d + gsl_vector_get(w->x, 11)*tf_d2;
            position(0,0) = (float)xf;
            position(1,0) = (float)yf;
            position(2,0) = (float)zf;
            yaw = (float)yawf;
            break;
        }
        case(GSL_ENOPROG):{
            std::cout << " Could not find" <<std::endl;
            break;
        }
    }
    return status;

}



/*
    Circular buffer implementation
*/

        std::vector<float> X;
        std::vector<float>::iterator X_it;
        std::vector<float>::iterator X_end;

        marton::circBuff::circBuff(int size_){
            size = size_;
            x_it = 0;
            for(int i=0;i<size;i++){
                X.push_back(0);
            }
        }
        void marton::circBuff::add(float value){
            X[x_it%size] = value;
            x_it++;
        }
        void marton::circBuff::read(std::vector<float>& ordered){
            ordered.clear();
            for(int i=0;i<size;i++){
                float element = X[(x_it+i)%size];
                ordered.push_back(element);
            }

        }

    /*    void marton::circBuff2::read(int siz_, double* ordered){
            int readSize = std::min(siz_,size);//To prevent writing outside given array
            for(int i=0;i<readSize;i++){
                float element = X[(x_it+i)%size];
                ordered[i] = (double)element;
            }
        }
*/














/* Functions for defining roll, pitch, and yaw rotation matrices
 * Increase speed by passing reference and edit in place?
 */
cv::Mat marton::getXRot(float roll){
    float sinX = std::sin(roll);
    float cosX = std::cos(roll);
    cv::Mat_<float> R_x = cv::Mat_<float>::zeros(3,3);
    R_x(0,0) = 1;
    R_x(1,1) = cosX;
    R_x(1,2) = -sinX;
    R_x(2,1) = sinX;
    R_x(2,2) = cosX;
    return R_x;
}
cv::Mat marton::getYRot(float pitch){
    float sinY = std::sin(pitch);
    float cosY = std::cos(pitch);
    cv::Mat_<float> R_y = cv::Mat_<float>::zeros(3,3);
    R_y(0,0) = cosY;
    R_y(0,2) = sinY;
    R_y(1,1) = 1;
    R_y(2,0) = -sinY;
    R_y(2,2) = cosY;
    return R_y;
}





/*
Test process function for tesitng


*/
void marton::process(void){

//Conbstruct data struct
    double p[12] = {0.4,1,1,0,
                    9.4,2,2,0,
                    26.4,3,3,0};
    double alpha[5] = {0,0,0,0.4,0.3};
    double t[4] = {0,1,2,3};
    double tf = 4;
    struct marton::poly2_data da = { p, alpha, t,tf};


    // Construct solver parameters struct
    double x_init[12] = { 1, 0, 0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0}; /* starting values. Maybe init these as last solution*/
    double weights[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    double xtol = 1e-2;
    double gtol = 1e-2;
    struct marton::nlinear_lsqr_param param = {x_init,weights,xtol,gtol};


    cv::Mat_<float> position = cv::Mat_<float>::ones(3,1);
    float yaw = 0;
    int status = marton::nlinear_lsqr_solve_2deg(param,da,position,yaw);
    std::cout << "Status: " << status << std::endl;
}
