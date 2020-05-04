#include <iostream>
#include "martonRobust.hpp"

/*
Methods for defining the input arguments are defined in the ang::angulation class
    src/angulation.hpp
    src/angulation.cpp
*/

//Prototype
double singleRegressor(double *x_array, double *y_array,int,int,int);

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
                /*
                    Rotate V is same as Derotate UAV. To undo (+)pitch->(+)roll of UAV we do (-)roll -> (-)pitch.
                    I.e to derotate v we apply (-)(-)roll -> (-)(-)pitch = (+)roll->(+)pitch
                */

                /* Descide size of problem. Number of parameters is constant, but number of cost equations is 4*bufferSize+2*anchorSize*/
                int bufferSize = tPrev.size();                      //Number of previously logged points
                int anchorSize = std::min((int)v.size(),(int)2);    //Number of uLOS vector available. Use max two anchors
                int costEquationSize = 4*bufferSize+2*anchorSize; //x,y,z,yaw (=4) times bufferSize, cone equation and yaw equation and symmetric cone penalty for FS (=3)


                cv::Mat Rx = marton::getXRot(roll);
                cv::Mat Ry = marton::getYRot(pitch);


                double offset[3] = {(double)pPrev[0], (double)pPrev[1], (double)pPrev[2]};
                double alpha[6*anchorSize];//Make space for anchorSize anchors. 6 numbers each (anchor coordinate + uLos vector)
                for(int i=0;i<anchorSize;i++){
                    /* Define q and v vectors, (and adapt v vector to current roll/pitch) */
                    cv::Mat_<float> qi = q[i];qi.reshape(1);          //Reshape to column vector so be sure that direct element access below is correct
                    cv::Mat_<float> v_i_unrot = v[i];v_i_unrot.reshape(1);
                    cv::Mat_<float> vi = Ry*Rx*v_i_unrot;//Apply from right
                    /*Set values in alpha array.*/
                    alpha[i*6+0] = (double)qi(0,0)-offset[0];//Norm shift marker position
                    alpha[i*6+1] = (double)qi(1,0)-offset[1];
                    alpha[i*6+2] = (double)qi(2,0)-offset[2];
                    alpha[i*6+3] = (double)vi(0,0);
                    alpha[i*6+4] = (double)vi(1,0);
                    alpha[i*6+5] = (double)vi(2,0);
                }
                /* Timeshift t vector so that oldest buffer sample has time stamp 0*/
                double tPrev_normed[bufferSize];
                for(int i=0;i<bufferSize;i++){
                    float element = tPrev[i]-tPrev[0];//Offset with first value
                    tPrev_normed[i] = (double)element;
                }
                /* Also timeshift time of failure (tf) with same amount*/
                float tf_normed = tf-tPrev[0];
                double tf_d_normed = (double)tf_normed;

                /* Shift x,y,z values from buffer so that oldest point is at [0,0,0]. NOTE: do not shift yaw, ie every fourth value of pPrev */
                int size_pPrev = pPrev.size();
                double pPrev_normed[size_pPrev];
                //Offset?
                for(int i=0;i<size_pPrev;i++){
                    float element = pPrev[i];
                    int offsetindex = i%4;//The first four values correspond to oldest
                    if(offsetindex!=3){// index 3 i e yaw shall not be offset
                        element -= pPrev[offsetindex];//Offset with first x y z values
                    }
                    pPrev_normed[i] = (double)element;
                }



                struct marton::poly2_data da = {pPrev_normed, alpha, tPrev_normed,tf_d_normed,bufferSize,anchorSize};
                /* ########Construct solver parameters struct####### */
                // Slope calculation from previous data for initial polynomial guess
                double slopeX = singleRegressor(tPrev_normed, pPrev_normed, bufferSize, 0, 4);
                double slopeY = singleRegressor(tPrev_normed, pPrev_normed, bufferSize, 1, 4);
                double slopeZ = singleRegressor(tPrev_normed, pPrev_normed, bufferSize, 2, 4);
                double slopeYAW = singleRegressor(tPrev_normed, pPrev_normed, bufferSize, 3, 4);
                double x_init[12] = { 0, slopeX, 0, 0,slopeY, 0, 0, slopeZ, 0, 0, slopeYAW, 0};



                //double x_init[12] = { 0.1, 0.01, 0, 0.1, 0.01, 0, 0.1, 0.01, 0, 0.0, 0.001, 0};//3*x, 3*y, 3*z, 4*yaw
                //double x_init[12] = { 1, 0, 0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
                /* Construct weight array. different weights for different equation types*/
                //Weights:  xold,yold,zold,yawolsd, x,y,z,yaw
                double pPrevWeight = 1;
                double coneWeight = 2;//2/anchorSize;
                double yawWeight = 1/anchorSize;
                double symmetryWeight = 0;
                double weights[costEquationSize];
                for(int i=0;i<costEquationSize;i++){
                    if(i<4*bufferSize){
                        weights[i]=pPrevWeight;
                    }else if(i<(4*bufferSize+anchorSize)){
                        weights[i]=coneWeight;
                    }else if(i<(4*bufferSize+2*anchorSize)){
                        weights[i]=yawWeight;
                    }
                }

                //double weights[14] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1};
                //double weights[14] = {1,1,1,0,1,1,1,0,1,1,1,0,0,0};//only x,y,z
                //double weights[14] = {1,1,1,0,1,1,1,0,1,1,1,0,1,1};
                double xtol = 1e-4;
                double gtol = 1e-4;
                struct marton::nlinear_lsqr_param param = {x_init,weights,xtol,gtol};
                // Perform optimization. pass d and param. Hur skicka structs som argument?
                std::vector<float> x_est(12); // Estimated polynomial parameters
                int status = marton::nlinear_lsqr_solve_2deg(param,da, x_est);

                if(status==GSL_SUCCESS){
                    /*std::cout << "X: ";
                    for(int j=0;j<12;j++){
                        std::cout << x_est[j] << ", ";
                    }
                    std::cout << std::endl;*/

                    float tf_normed_sqrd = tf_normed*tf_normed;
                    cv::Mat_<float> po = cv::Mat_<float>::zeros(3,1);
                    for(int i=0;i<3;i++){
                        float delta_value = x_est[3*i] + x_est[3*i+1]*tf_normed + x_est[3*i+2]*tf_normed_sqrd;//Calculate value from polybnomial. But it is offset!
                        po(i,0) = pPrev[i] + delta_value;// Add offset and update position
                    //    po(i,0) = delta_value;
                    }

                    po.copyTo(position);

                    float delta_yaw = x_est[9] + x_est[10]*tf_normed + x_est[11]*tf_normed_sqrd;
                    //float yaw_ = pPrev[3] + delta_yaw;
                    float yaw_ = delta_yaw;
                    //double fractpart,intpart;
                    //fractpart = modf(yaw_/6.2832,&intpart);
                    //yaw = 6.2832*(float)fractpart;

                    // Cast yaw to +-pi
                    while(yaw_>marton::PI){
                        yaw_-=2*marton::PI;
                    }
                    while(yaw_<-marton::PI){
                        yaw_+=2*marton::PI;
                    }
                    yaw = yaw_;
                    //std::cout << "TF: " << tf_normed << std::endl;
                    //std::cout << "Marto1: X: "<< position(0,0) << ", Y: "<< position(1,0) << ", Z: " << position(2,0) << ", yaw: " << yaw<< std::endl;
                }else{
                    std::cout << "Failed Marton." << std::endl;
                }

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
    // Read parameters from data-struct (arrays are given as pointers)
    double *p = ((struct poly2_data *)data)->p;
    double *alpha = ((struct poly2_data *)data)->alpha;
    double *t = ((struct poly2_data *)data)->t;
    double tf = ((struct poly2_data *)data)->tf;
    int bufferSize = ((struct poly2_data *)data)->bufferSize;
    int anchorSize = ((struct poly2_data *)data)->anchorSize;

    // Get arguments to a normal vector for easy access
    double x_[12];
    for(size_t i = 0;i<12;i++){
        x_[i] = gsl_vector_get(x,i);
    }

/* First cost equations. Distance between polynomial estimations and measured points */
// T order: [t1,t2,t3,tf]  kronologisk
// p order: [x1,y1,z1,yaw1,x2,y2,z2,yaw2,...] Yttre ordning: kronologisk, inre ordning: x,y,z,yaw
// x order: [sigmax0,sigmax1,sigmax2,sigmay0,sigmay1,sigmay2...]  Yttre ordgning: x,y,z,yaw, inre ordnnig: stigande grad
int f_nmbr=0;//Counter to keep track of f vector position
    for (int i=0;i < bufferSize; i++){
        // Calculate time stamp powers
        double t_i = t[i];
        double t_i_sqrd = pow(t_i,(double)2);

        int offset = i*4;
        gsl_vector_set (f, 0+offset, x_[0]+x_[1]*t_i+x_[2]*t_i_sqrd - p[0+offset]); //X for time i
        gsl_vector_set (f, 1+offset, x_[3]+x_[4]*t_i+x_[5]*t_i_sqrd - p[1+offset]); //Y for time i
        gsl_vector_set (f, 2+offset, x_[6]+x_[7]*t_i+x_[8]*t_i_sqrd - p[2+offset]); //Z for time i
        gsl_vector_set (f, 3+offset, x_[9]+x_[10]*t_i+x_[11]*t_i_sqrd - p[3+offset]); //Yaw for time i

    }
/* Cone equation */
    f_nmbr = bufferSize*4;
    double t_f = tf;      //Current time stamp (tf = time of failure)
    double t_f_sqrd = pow(t_f,(double)2);
    //Evaluate coordinate with current provided sigma-parameters
    double x_est = x_[0]+x_[1]*t_f+x_[2]*t_f_sqrd;
    double y_est = x_[3]+x_[4]*t_f+x_[5]*t_f_sqrd;
    double z_est = x_[6]+x_[7]*t_f+x_[8]*t_f_sqrd;
    for(int i=0;i<anchorSize;i++){
        int offset = i*6; //Every anchor uses 6 elements in alpha
        double c_sqr = (alpha[3+offset]*alpha[3+offset]+alpha[4+offset]*alpha[4+offset])/(alpha[5+offset]*alpha[5+offset]); // to use in cone equation sqrt(x^2+y^2) = c*z
        //double c = sqrt(c_sqr);
        //double f12_sqr = pow(x_est-alpha[0+offset],(double)2) + pow(y_est - alpha[1+offset],(double)2) - c_sqr*pow(z_est - alpha[2+offset],(double)2);//Original cost function. Vary x-y-z
        double z_last =p[0] + 0.3*(p[(bufferSize-1)*4+2]-p[2])*(t_f-t[0])/(t[bufferSize-1]-t[0]); //Choose Z as linear continuation of buffered z values at tf. Scaled down with 0.3
        //double z_last = p[(bufferSize-1)*4+2];
        double f12_sqr = pow(x_est-alpha[0+offset],(double)2) + pow(y_est - alpha[1+offset],(double)2) - c_sqr*pow(z_last - alpha[2+offset],(double)2);//Can only modify in x-y to optimize this
        double f12 = sqrt(abs(f12_sqr));//f12_sqr;//sqrt(abs(f12_sqr));
        gsl_vector_set (f, f_nmbr, f12);
        f_nmbr++;
    }
/*Equation 13 - Hardcoded position 14. keep counter if allow for more previous data points*/

//x-y of v vector in uav frame towards anchor, rotated wit estimated yaw UNIT
//x-y of vector between anchor and estimated UAV global pos UNIT
//norm of difference should be zero.

    double yaw_est = x_[9]+x_[10]*t_f+x_[11]*t_f_sqrd;//Yaw est is constant for all anchors f course
    double cos_ = cos(yaw_est);
    double sin_ = sin(yaw_est);
    double x_last = p[4*(bufferSize-1)];// vector from uav to anchor is calculated from last known position. Thus this cost equation wll not affect pos estimation
    double y_last = p[4*bufferSize];
    for(int i=0;i<anchorSize;i++){
        int offset = i*6;
        double est_norm = sqrt(pow(x_last-alpha[0+offset],(double)2)+pow(y_last-alpha[1+offset],(double)2));//norm of vector from anchor to last known position (only x,y)
        double v_norm = sqrt(pow(alpha[3+offset],(double)2) + pow(alpha[4+offset],(double)2));          //norm of ulos vector from uav to anchor (only x,y)

        double v_est_x = cos_*alpha[3+offset] - sin_*alpha[4+offset];//v(x,y) vector rotated with the estimated yaw
        double v_est_y = sin_*alpha[3+offset] + cos_*alpha[4+offset];
        double f13 = pow((x_last-alpha[0+offset])/est_norm  + (v_est_x/v_norm),(double)2) + pow((y_last-alpha[1+offset])/est_norm + v_est_y/v_norm,(double)2);
        gsl_vector_set (f, f_nmbr, sqrt(f13));
        f_nmbr++;
    }
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

int marton::nlinear_lsqr_solve_2deg(nlinear_lsqr_param parameters, poly2_data data,std::vector<float>& returnX){

    const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
    gsl_multifit_nlinear_workspace *w;
    gsl_multifit_nlinear_fdf fdf;
    gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();
    const size_t n = 4*data.bufferSize+2*data.anchorSize; //n is number of data points
    const size_t par = 12; // p is i guess number of parameters to solve for

    double *x_init = parameters.x_init;

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
    size_t maxiter = 100;
    status = gsl_multifit_nlinear_driver(maxiter, xtol, gtol, ftol,
                                         NULL, NULL, &info, w);




/*    std::cout << "Polynomial: ";
    for(size_t i=0;i<12;i++){
        double xi = gsl_vector_get(w->x, i);
        std::cout << xi << ", ";
    }
        std::cout << std::endl;
*/
    switch(status){
        /* Calculate new position using the polynomial */
        case(GSL_SUCCESS):{
            //std::cout << " Success" << std::endl;
            //std::cout << "Info: " << info <<  std::endl;
            //std::cout << "Iterations: " << gsl_multifit_nlinear_niter(w) << std::endl;
        /*    double tf_d = data.tf;
            double tf_d2 = tf_d*tf_d;
            double xf = gsl_vector_get(w->x, 0) + gsl_vector_get(w->x, 1)*tf_d + gsl_vector_get(w->x, 2)*tf_d2;
            double yf = gsl_vector_get(w->x, 3) + gsl_vector_get(w->x, 4)*tf_d + gsl_vector_get(w->x, 5)*tf_d2;
            double zf = gsl_vector_get(w->x, 6) + gsl_vector_get(w->x, 7)*tf_d + gsl_vector_get(w->x, 8)*tf_d2;
            double yawf = gsl_vector_get(w->x, 9) + gsl_vector_get(w->x, 10)*tf_d + gsl_vector_get(w->x, 11)*tf_d2;
        */
            returnX.clear();
            for(int i=0;i<12;i++){
                double element = gsl_vector_get(w->x, i);
                returnX.push_back((float)element);
            }


            //std::cout << "Performed Marton. No update." << std::endl;
            //std::cout << "TF: " << tf_d << std::endl;
            //std::cout << "delta Pos: " << xf << ", " << yf << ", " << zf << ", "<< yawf << std::endl;
            /*position(0,0) = (float)xf;
            position(1,0) = (float)yf;
            position(2,0) = (float)zf;
            yaw = (float)yawf;
            */
            break;
        }
        case(GSL_ENOPROG):{
            std::cout << " Could not find" <<std::endl;
            std::cout << "Info: " << info <<  std::endl;
            break;
        }case(GSL_EMAXITER):{
            std::cout << "Max iterations" << std::endl;
            std::cout << "Iterations: " << gsl_multifit_nlinear_niter(w) << std::endl;
            std::cout << "Info: " << info <<  std::endl;
            break;
        }
        default:{
            std::cout << "nlinear other status code: " << status << std::endl;
            std::cout << "Iterations: " << gsl_multifit_nlinear_niter(w) << std::endl;
            std::cout << "Info: " << info <<  std::endl;
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


// Code for calculating slope of points for initial direction of polynomial.
//      Only calculate linear regression slope
double singleRegressor(double *x_array, double *y_array,int size, int pStart, int pStep){
    //Calculate mean value of x and y
    double x_mean = 0;
    double y_mean = 0;
    for(int i=0;i<size;i++){
        x_mean += x_array[i];
        y_mean += y_array[pStart+i*pStep];
    }
    x_mean/=(double)size;
    y_mean/=(double)size;

    //Calculate lin regression slope
    double slope_num = 0;
    double slope_den = 0;
    for(int i=0;i<size;i++){
        slope_num += (x_array[i]-x_mean)*(y_array[pStart+i*pStep]-y_mean);
        slope_den += (x_array[i]-x_mean)*(x_array[i]-x_mean);
    }
    double slope = slope_num/slope_den;
    return slope;
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
    int n=12;
    struct marton::poly2_data da = { p, alpha, t,tf,n};


    // Construct solver parameters struct
    double x_init[12] = { 1, 0, 0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0}; /* starting values. Maybe init these as last solution*/

    double weights[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    double xtol = 1e-2;
    double gtol = 1e-2;
    struct marton::nlinear_lsqr_param param = {x_init,weights,xtol,gtol};


    cv::Mat_<float> position = cv::Mat_<float>::ones(3,1);
    float yaw = 0;
    std::vector<float> Xest(12);
    int status = marton::nlinear_lsqr_solve_2deg(param,da,Xest);
    std::cout << "Status: " << status << std::endl;
}
