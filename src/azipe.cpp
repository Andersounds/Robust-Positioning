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

bool az::azipe(const std::vector<cv::Mat_<float>>& v,
            const std::vector<cv::Mat_<float>>& q,
            cv::Mat_<float>& position,
            float& yaw,
            float roll,float pitch){
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
//std::cout << "cost+: " << E_pos(0,0) << ",\t"<< "cost-: "<< E_neg(0,0)<<std::endl;//", angle: "<< azimuth <<std::endl;
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
        float azimuth_neg = azimuth_pos+PI;//-std::acos(-x);//Other possible angle. IS THIS ALWAYS SHIFTED BY PI?
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
        R_neg(0,0) = h*e_neg(0,0);              R_neg(0,1) = h*e_neg(1,0);              R_neg(0,2) = -a;
        R_neg(1,0) = k*e_neg(0,0) - l*e_neg(1,0);   R_neg(1,1) = l*e_neg(0,0) + k*e_neg(1,0);   R_neg(1,2) = b;
        R_neg(2,0) = u*e_neg(0,0) + v_factor*e_neg(1,0);R_neg(2,1) = -v_factor*e_neg(0,0)+u*e_neg(1,0); R_neg(2,2) = c;
        //Calculate new vehicle position (Equation 25. Without mean shift offset)
        cv::Mat_<float> P_vehicle_pos = -R_pos.t()*t_opt_pos;
        cv::Mat_<float> P_vehicle_neg = -R_neg.t()*t_opt_neg;
        //Choose the correct angle based on z-coordinate
        float delta_pos = std::abs((P_vehicle_pos(2,0)-position(2,0)));
        float delta_neg = std::abs((P_vehicle_neg(2,0)-position(2,0)));

        if(!isnan(delta_pos)){
            if(!isnan(delta_neg)){
                if(delta_pos < delta_neg){
                    P_vehicle_pos.copyTo(position);
                    yaw = azimuth_pos;
                    return true;
                }else{
                    P_vehicle_neg.copyTo(position);
                    yaw = azimuth_neg;
                    return true;
                }
            }else{
                P_vehicle_pos.copyTo(position);
                yaw = azimuth_pos;
                return true;
            }
        }else if(!isnan(delta_neg)){
            P_vehicle_neg.copyTo(position);
            yaw = azimuth_neg;
            return true;
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
        return false; //Should never reach this

    }
