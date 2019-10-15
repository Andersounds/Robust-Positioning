#include <iostream>
#include <opencv2/opencv.hpp>
#include "azipe.hpp"
#include "../include/Quartic-master/quartic.h"


/*
AZIPE - AZImuth and Position Estimation
This class implements the AZIPE-algorithm, as presented in

J. Kim and H. Hmam, \3D Self-Localisation from Angle of Arrival Measurements,"
Weapons Systems Division, 2009.

It solves the quartic equation using the package

https://github.com/sasamil/Quartic

Inputs:
vector<Mat> v   A set of unit-Line-of-Sight vectors in the vehicle frame (measured) to each of the visible anchors
vector<Mat> q   A corresponding set of 3d-coordinates of each anchor in the global frame
Mat  position   An inputoutput array of the vehicle position. Its input value is used to estimate distances to observed anchors
                -If localisation is successful, its value is updated with the new estimation
float    yaw    An inputoutput value where the azimuth (yaw) angle is returned if estimation is successful. Input value is not used
float    roll   A value representing the roll of the vehicle as related to the global frame expressed in radians.
float   pitch   A value representing the pitch of the vehicle as related to the global frame expressed in radians.

Outputs:
Mat position    An inputoutput array that is updated with the new localisation estimation if it is successful
float    yaw    An inputoutput value where the azimuth (yaw) angle is returned if estimation is successful. Input value is not used
bool  RETURN    A success-value indicating successful or failed localization
Notes:
Roll and pitch are to be expressed as Euler angles from the rotational sequence yaw-pitch-roll, where yaw is unknown
yaw     - rotation about the z-axis.    (=Azimuth angle)
pitch   - rotation about the y-axis.
roll    - rotation about the x-axis.
*/




void p(std::string out_){
    std::cout << out_ << std::endl;
}

int az::azipe(const std::vector<cv::Mat_<float>>& v,
            const std::vector<cv::Mat_<float>>& q,
            cv::Mat_<float>& position,
            float& yaw,
            float pitch,float roll,
            cv::Mat_<float>& t_opt){
  std::cout << "TODO: Test fullangulation and give known pitch roll." << std::endl;
            //Yaw - psi
            //pitch - theta
            //roll - phi

        //Define some per-position-constant quantities
        float phi = -roll; //Note the sign on this!
        float theta = -pitch;
        float h = cos(theta);
        float k = sin(phi)*sin(theta);
        float l = cos(phi);
        float u = cos(phi)*sin(theta);
        float v_factor = sin(phi);
        float a = sin(theta);
        float b = sin(phi)*cos(theta);
        float c = cos(phi)*cos(theta);
        cv::Mat_<float> A_mat = cv::Mat_<float>::zeros(3,3);
        cv::Mat_<float> AQ = cv::Mat_<float>::zeros(3,2);
        cv::Mat_<float> As = cv::Mat_<float>::zeros(3,1);
        //Save incremental Q_i, s_i, and A_mat_i in these vectors so that they need not be recalculated
        std::vector<cv::Mat_<float>> Q_saved;
        std::vector<cv::Mat_<float>> s_saved;
        std::vector<cv::Mat_<float>> A_mat_saved;
        for(int i=0;i<v.size();i++){
            // Equation 12. Calculate Qi, Si
            cv::Mat_<float> Q_i = cv::Mat_<float>::zeros(3,2);
            float q_ix = q[i](0,0);
            float q_iy = q[i](1,0);
            float q_iz = q[i](2,0);
            Q_i(0,0) = h*q_ix;          Q_i(0,1) = h*q_iy;
            Q_i(1,0) = k*q_ix + l*q_iy; Q_i(1,1) = -l*q_ix + k*q_iy;
            Q_i(2,0) = u*q_ix - v_factor*q_iy; Q_i(2,1) = v_factor*q_ix + u*q_iy;
            cv::Mat_<float> s_i = cv::Mat_<float>::zeros(3,1);
            s_i(0,0) = -q_iz*a; s_i(1,0) = q_iz*b;  s_i(2,0) = q_iz*b;
            //Save Q_i and s_i
            Q_saved.push_back(Q_i);
            s_saved.push_back(s_i);
            //Calculate A_mat
            //Calc V. (Mentioned between equation 4 and 5)
            cv::Mat_<float> V_i = v[i]*(v[i].t());
            //Approximate d_i using last vehicle position (last row before 2.3)
//std::cout << "position: " << position << std::endl;
//std::cout << "q[i]: " << q[i] << std::endl;
            cv::Mat_<float> delta = (position - q[i]);
//std::cout << "delta: " << delta << std::endl;
            cv::Mat_<float> d_i_sqrd = delta.t()*delta;//Distance squared
//std::cout << "d_i_sqrd: " << d_i_sqrd << std::endl;
            if(d_i_sqrd(0,0)<1e-5){ d_i_sqrd = 0.1;}

            //Equation 6
            cv::Mat_<float> A_mat_i = (cv::Mat_<float>::eye(3,3)-V_i)/(d_i_sqrd(0,0));
            //Save A_mat_i
            A_mat_saved.push_back(A_mat_i);
            //Equation 8
            A_mat += A_mat_i;
            //Sum-terms of expressions in equation 14
            AQ += A_mat_i*Q_i;
            As += A_mat_i*s_i;
        }


        //Equation 14
        cv::Mat_<float> F = -A_mat.inv()*AQ;
        cv::Mat_<float> w = -A_mat.inv()*As;
        //Second loop through to calculate M,m from equation 18,19
        cv::Mat_<float> M = cv::Mat_<float>::zeros(2,2);
        cv::Mat_<float> m = cv::Mat_<float>::zeros(2,1);
        for(int i=0;i<v.size();i++){
            //G_i, g_i from equation 16
            cv::Mat_<float> G_i = F + Q_saved[i];
            cv::Mat_<float> g_i = w + s_saved[i];
            cv::Mat_<float> G_I_A_I_temp = G_i.t()*A_mat_saved[i];
            M += (G_I_A_I_temp*G_i);
            m += (G_I_A_I_temp*g_i);
        }
        //Coefficients for quartic Function. Equation 21
        float A = M(0,0) - M(1,1);
        float B = 2*M(0,1);
        float C = 2*m(0,0);
        float S = 2*m(1,0);
        //Finally define coefficents. Equation 24
        float A4 = 4*(A*A + B*B);
        float A3 = 4*(A*C + B*S);
        float A2 = S*S + C*C - A4;
        float A1 = -4*A*C - 2*B*S;
        float A0 = B*B - C*C;
        //Solve quartic function for x: cos(azimuth)
        double a_quart = ((double)A3)/((double)A4);
        double b_quart = ((double)A2)/((double)A4);
        double c_quart = ((double)A1)/((double)A4);
        double d_quart = ((double)A0)/((double)A4);
        std::complex<double>*  solutions = solve_quartic(a_quart, b_quart, c_quart, d_quart);
        //std::complex<double>*  solutions = solve_quartic(d_quart, c_quart, b_quart, a_quart);
        //Sort out any non-viable solutions (>1)
        float lowestCost = 1000;
        int solutionID = 0;
//std::cout << "-------" << std::endl;
        for(int i=0;i<4;i++){
            float x = (float) solutions[i].real();//x = cos(aximuth)
            //Check if solution is even viable |cos(x)|<=1
            if(x<=1 && x>=-1){
//std::cout << "x" << i << ": " << x << ", ";
                //Calculate azimuth angle and e
                float azimuth = std::acos(x);
                //Calculate cost E (Equation 17) Disregarded constant d
                cv::Mat_<float> e = cv::Mat_<float>::zeros(2,1);
                e(0,0) = x;
                e(1,0) = std::sin(azimuth);
                //Cost with positive angle (cos(x)=cos(-x))
                cv::Mat_<float> E_pos = e.t()*M*e + 2*m.t()*e;
//                e(1,0) = std::sin(-azimuth);
//                cv::Mat_<float> E_neg = e.t()*M*e + 2*m.t()*e;
//std::cout << "cost+: " << E_pos(0,0) << "        "<< "cost-: "<< E_neg(0,0)<<std::endl;//", angle: "<< azimuth <<std::endl;
                //Check if current solution is best
                if(E_pos(0,0) < lowestCost){
                    lowestCost = E_pos(0,0);
                    solutionID = i;
                }
            }
        }
        /*  The cost E of the

        */
        float x = (float) solutions[solutionID].real();
        float azimuth_pos = std::acos(x);
        float azimuth_neg = azimuth_pos+PI;//-std::acos(-x);//Other possible angle corresponding to conjugate solution with same lowest cost
        //Choose the correct angle
        cv::Mat_<float> e_pos = cv::Mat_<float>::zeros(2,1);
        e_pos(0,0) = x;
        e_pos(1,0) = std::sin(azimuth_pos);
        cv::Mat_<float> e_neg = cv::Mat_<float>::zeros(2,1);
        e_neg(0,0) = -x;
        e_neg(1,0) = std::sin(azimuth_neg);
        //Calculate optimal translation in vehicle frame (Equation 13)
        cv::Mat_<float> t_opt_pos = F*e_pos + w;
        cv::Mat_<float> t_opt_neg = F*e_neg + w;
        //Calculate rotational matrix (Equation 9)
        //For positive angle candidate
        cv::Mat_<float> R_pos = cv::Mat_<float>::zeros(3,3);
        R_pos(0,0) = h*e_pos(0,0);                      R_pos(0,1) = h*e_pos(1,0);                      R_pos(0,2) = -a;
        R_pos(1,0) = k*e_pos(0,0) - l*e_pos(1,0);       R_pos(1,1) = l*e_pos(0,0) + k*e_pos(1,0);       R_pos(1,2) = b;
        R_pos(2,0) = u*e_pos(0,0) + v_factor*e_pos(1,0);R_pos(2,1) = -v_factor*e_pos(0,0)+u*e_pos(1,0); R_pos(2,2) = c;
        //R for neg angle candidate
        cv::Mat_<float> R_neg = cv::Mat_<float>::zeros(3,3);
        R_neg(0,0) = h*e_neg(0,0);                      R_neg(0,1) = h*e_neg(1,0);                      R_neg(0,2) = -a;
        R_neg(1,0) = k*e_neg(0,0) - l*e_neg(1,0);       R_neg(1,1) = l*e_neg(0,0) + k*e_neg(1,0);       R_neg(1,2) = b;
        R_neg(2,0) = u*e_neg(0,0) + v_factor*e_neg(1,0);R_neg(2,1) = -v_factor*e_neg(0,0)+u*e_neg(1,0); R_neg(2,2) = c;
        //Calculate new vehicle position (Equation 25. Without mean shift offset)
        cv::Mat_<float> P_vehicle_pos = -R_pos.t()*t_opt_pos;
        cv::Mat_<float> P_vehicle_neg = -R_neg.t()*t_opt_neg;
        //Choose the correct angle based on z-coordinate
        float delta_pos = std::abs((P_vehicle_pos(2,0)-position(2,0)));
        float delta_neg = std::abs((P_vehicle_neg(2,0)-position(2,0)));

        //float delta_pos = std::abs((P_vehicle_pos(0,0)-position(0,0))) + std::abs((P_vehicle_pos(1,0)-position(1,0))) + std::abs((P_vehicle_pos(2,0)-position(2,0)));
        //float delta_neg = std::abs((P_vehicle_neg(0,0)-position(0,0))) + std::abs((P_vehicle_neg(1,0)-position(1,0))) + std::abs((P_vehicle_neg(2,0)-position(2,0)));


        if(!isnan(delta_pos)){
            if(!isnan(delta_neg)){
                if(delta_pos < delta_neg){
                    P_vehicle_pos.copyTo(position);
                    t_opt_pos.copyTo(t_opt);
                    yaw = limitYawRange(azimuth_pos);
                    return az::AZIPE_SUCCESS;
                }else{
                    P_vehicle_neg.copyTo(position);
                    t_opt_neg.copyTo(t_opt);
                    yaw = limitYawRange(azimuth_neg);
                    return az::AZIPE_SUCCESS;
                }
            }else{
                P_vehicle_pos.copyTo(position);
                t_opt_pos.copyTo(t_opt);
                yaw = limitYawRange(azimuth_pos);
                return az::AZIPE_SUCCESS;
            }
        }else if(!isnan(delta_neg)){
            P_vehicle_neg.copyTo(position);
            t_opt_neg.copyTo(t_opt);
            yaw = limitYawRange(azimuth_neg);
            return az::AZIPE_SUCCESS;
        }
/*
        if(delta_pos < delta_neg && !isnan(delta_pos)){//Choose the angle that corresponds with smallest difference in z-coordinate since last calculation
            if(v.size()>=2){//Demand a minimum of 2 active anchors
                P_vehicle_pos.copyTo(position);
                yaw = azimuth_pos;
                return true;
            }
        }else if(!isnan(delta_neg)){
            if(v.size()>=2){//Demand a minimum of 2 active anchors
                P_vehicle_neg.copyTo(position);
                yaw = azimuth_neg;
                return true;
            }
        }
*/
        //If return value is Nan
        return az::AZIPE_FAIL; //Should never reach this

    }


/*
AIPE algorithm
  Angle Increments and Position Estimation
  Implemented from same paper as AZIPE

Inputs:
vector<Mat> v   A set of unit-Line-of-Sight vectors in the vehicle frame (measured) to each of the visible anchors
vector<Mat> q   A corresponding set of 3d-coordinates of each anchor in the global frame
Mat  position   An inputoutput array of the initial estimation of vehicle position. Should be close to true (use output of AZIPE)
float    zrot   An inputoutput value with initial estimation of z rotation ( yaw ) [rad]
float    yrot   An inputoutput value with initial estimation of y rotation (pitch) [rad]
float    xrot   An inputoutput value with initial estimation of x rotation (roll ) [rad]
float  thresh   Stop condition of iteration. Function returns if error if iteration step is less than thresh. (or reaches max iter)

Outputs:
Mat  position   Updated optimal position estimate
float    zrot   Updated optimal z rotation ( yaw ) [rad]
float    yrot   Updated optimal y rotation (pitch) [rad]
float    xrot   Updated optimal x rotation (roll ) [rad]
bool  RETURN    A success-value indicating successful or failed localization
Notes:
Roll and pitch are to be expressed as Euler angles from the rotational sequence yaw-pitch-roll, where yaw is unknown
yaw     -  psi  - rotation about the z-axis.    (=Azimuth angle)
pitch   - theta - rotation about the y-axis.
roll    -  phi  - rotation about the x-axis.


ORIENTATION:
q shall be given in global coordinate frame
v,zrot,yrot,xrot shall share frame. I.e they can either be given in the camera frame or UAV frame. Just keep track of which one...

Procedure within each while-lap:
Eq. 29.     Define all Q_i          (from given q and defined c1_1 - c9_3)
Eq. 31.     Define all s_i          (from given q and defined c0)
Eq. 6.      Define all Lambda_i     (from given q, given v, and current position estimation p)
Eq. 14.1    Calculate F             (from Lambda_i (defined above), Q_i (defined above))
Eq. 14.2    Calculate w             (from Lambda_i (defined above), s_i (defined above))
Eq. 16.1    Calculate g             (from w (calculated above), s (defined above))
Eq. 16.2    Calculate G             (from F (calulated above), Q (defined above))
Eq. 33.1    Calculate M             (from G (calculated above), Lamda (defined above))
Eq. 33.2    Calculate m             (from G (calculated above), Lambda (defined above), g (calculated above))
Eq. 35.     Calculate e             (From M (calculated above), m (calculated above))
Eq. 36.     Iterate angles
Eq. 37.     Iterate position
*/
int az::aipe(const std::vector<cv::Mat_<float>>& v,
                const std::vector<cv::Mat_<float>>& q,
                cv::Mat_<float>& position, float& psi, float&theta, float& fi,
                float thresh,
                const cv::Mat_<float>& t_opt){

        //theta*=-1;
        //fi*=-1;


    cv::Mat_<float> err = cv::Mat_<float>::ones(1,1)*(thresh+1);
    int counter = 0;
    int maxIter = 2;
    //Add threshold
    while(counter < maxIter ){//&& err(0,0)>1){
        counter++;
        //Save incremental Q_i, s_i, and Lambda_i in these vectors so that they need not be recalculated
        std::vector<cv::Mat_<float>> Q_i_saved; //Needed for equation 16.
        std::vector<cv::Mat_<float>> s_i_saved; //Needed for equation 16.
        std::vector<cv::Mat_<float>> Lambda_i_saved;//Needed for equation 33.


        cv::Mat_<float> Lambda = cv::Mat_<float>::zeros(3,3);
        cv::Mat_<float> Lambda_Q = cv::Mat_<float>::zeros(3,3);
        cv::Mat_<float> Lambda_s = cv::Mat_<float>::zeros(3,1);

        for(int i=0;i<v.size();i++){
            //Define some cos/sin values to be used below
            float ct = cos(theta);
            float st = sin(theta);
            float cf = cos(fi);
            float sf = sin(fi);
            float cp = cos(psi);
            float sp = sin(psi);
            //Define c1_0 - c9_3 according to appendix C
            float c1_0=ct*cp; float c1_1=0; float c1_2=-sf*cp; float c1_3=-ct*sp;
            float c2_0=ct*sp; float c2_1=0; float c2_2=-st*sp; float c2_3=ct*cp;
            float c3_0=-st;   float c3_1=0; float c3_2=-ct;    float c3_3=0;
            float c4_0=sf*st*cp-cf*sp; float c4_1=cf*st*cp+sf*sp; float c4_2=sf*ct*cp; float c4_3=cf*cp-st*sf*sp;
            float c5_0=sf*st*sp+cf*cp; float c5_1=cf*st*sp-sf*cp; float c5_2=sf*ct*sp; float c5_3=-(cf*sp+sf*st*cp);
            float c6_0=sf*ct; float c6_1=cf*ct; float c6_2=-sf*st; float c6_3=0;
            float c7_0=cf*st*cp+sf*sp; float c7_1=-sf*st*cp+cf*sp; float c7_2=cf*ct*cp; float c7_3=sf*cp-cf*st*sp;
            float c8_0=cf*st*sp-sf*cp; float c8_1=-sf*st*sp-cf*cp; float c8_2=cf*ct*sp; float c8_3=sf*sp+cf*st*cp;
            float c9_0=cf*ct; float c9_1=-sf*ct; float c9_2=-cf*st; float c9_3=0;
            //Equation 29. Calculate Q_i
            cv::Mat_<float> Q_i = cv::Mat_<float>::zeros(3,3);
            float q_ix = q[i](0,0);
            float q_iy = q[i](1,0);
            float q_iz = q[i](2,0);
            Q_i(0,0) = c1_1*q_ix + c2_1*q_iy + c3_1*q_iz;   Q_i(0,1) = c1_2*q_ix + c2_2*q_iy + c3_2*q_iz;   Q_i(0,2) = c1_3*q_ix + c2_3*q_iy + c3_3*q_iz;
            Q_i(1,0) = c4_1*q_ix + c5_1*q_iy + c6_1*q_iz;   Q_i(1,1) = c4_2*q_ix + c5_2*q_iy + c6_2*q_iz;   Q_i(1,2) = c4_3*q_ix + c5_3*q_iy + c6_3*q_iz;
            Q_i(2,0) = c7_1*q_ix + c8_1*q_iy + c9_1*q_iz;   Q_i(2,1) = c7_2*q_ix + c8_2*q_iy + c9_2*q_iz;   Q_i(2,2) = c7_3*q_ix + c8_3*q_iy + c9_3*q_iz;
            Q_i_saved.push_back(Q_i);//Save intermediate Q_i to be used in equation 16
            //Equation 31. Calculate s_i
            cv::Mat_<float> s_i = cv::Mat_<float>::zeros(3,1);
            s_i(0,0) = c1_0*q_ix + c2_0*q_iy + c3_0*q_iz;
            s_i(1,0) = c4_0*q_ix + c5_0*q_iy + c6_0*q_iz;
            s_i(2,0) = c7_0*q_ix + c8_0*q_iy + c9_0*q_iz;
            s_i_saved.push_back(s_i);//Save intermediate a_i to be used in equation 16
            //Equation 6. Calculate Lambda_i
            cv::Mat_<float> V_i = v[i]*(v[i].t());
            cv::Mat_<float> delta = (position - q[i]);
            cv::Mat_<float> d_i_sqrd = delta.t()*delta;//Distance squared
            if(d_i_sqrd(0,0)<1e-5){ d_i_sqrd = 0.1;}
            cv::Mat_<float> Lambda_i = (cv::Mat_<float>::eye(3,3)-V_i)/(d_i_sqrd(0,0));//Eq. 6

            Lambda_i_saved.push_back(Lambda_i);//Save intermediate Lambdas for Equation 33.
            //Equation 8
            Lambda += Lambda_i;
            //Sum-terms of expressions in equation 14
            Lambda_Q += Lambda_i*Q_i;
            Lambda_s += Lambda_i*s_i;
        }
        //Equation 14
        cv::Mat_<float> Lambda_inv = Lambda.inv();
        cv::Mat_<float> F = -Lambda_inv*Lambda_Q;
        cv::Mat_<float> w = -Lambda_inv*Lambda_s;
        //Equation 16 and 33
        cv::Mat_<float> M = cv::Mat_<float>::zeros(3,3);
        cv::Mat_<float> m = cv::Mat_<float>::zeros(3,1);
        for(int i=0;i<v.size();i++){
            //Eq. 16.
            cv::Mat_<float> G_i = F + Q_i_saved[i];
            cv::Mat_<float> g_i = w + s_i_saved[i];
            //Eq. 33.
            M += G_i.t()*Lambda_i_saved[i]*G_i;
            m += G_i.t()*Lambda_i_saved[i]*g_i;
        }
        //Equation 35. Calculate optimal angle-deltas
        cv::Mat_<float> e_op = -M.inv()*m;
        //std::cout << "e_op: " << e_op.t() << std::endl;
        //Equation 36. Update angles.
        psi += e_op(2,0);
        theta += e_op(1,0);
        fi += e_op(0,0);
        //Equation 26. Define R mat
        cv::Mat_<float> R = cv::Mat_<float>::zeros(3,3);
        float ct = cos(theta);
        float st = sin(theta);
        float cf = cos(fi);
        float sf = sin(fi);
        float cp = cos(psi);
        float sp = sin(psi);
        R(0,0) = ct*cp;             R(0,1) = ct*sp;             R(0,2) = -st;
        R(1,0) = -cf*sp+sf*st*cp;   R(1,1) = cf*cp+sf*st*sp;    R(1,2) = ct*sf;
        R(2,0) = sf*sp+cf*cp*st;    R(2,1) = -cp*sf+st*sp*cf;   R(2,2) = cf*ct;
        //Equation 37. Update vehicle position
        cv::Mat_<float> t_op = F*e_op+w;//Give t_op as argument to aipe? Or is t opt F*e+w according to Eq. 13?
        //cv::Mat_<float> t_op; //t_op given as argument instead
        //t_opt.copyTo(t_op); //t_op given as argument instead
        position = -R.t()*t_op;
        err = e_op.t()*e_op;
        //std::cout << "Err " << counter << ": " << err(0,0) << std::endl;
    }
    return 1;
}

/*
    This functions is here to limit yaw range. In experimetn with the 5-okt dataset
    it showed that it limited between 0 and pi. Without it it is already limited to 0 to 2pi.
    The function calls are kept in code to reduce risk of some uncareful tampering error, but the function
    does not do anything except mirror the argument.
*/
float az::limitYawRange(float yawCandidate){
    return yawCandidate;
    float div = yawCandidate/az::PI;
    float yawLim = (div - floor(div))*az::PI;
    //return yawLim;
}
