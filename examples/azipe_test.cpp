#include <iostream>
#include <opencv2/opencv.hpp>
#include "../src/azipe.hpp"

/*
    This program is a minimal test environment for the Azipe algorithm. Assumes always roll, pitch is zero.
    Specify uav coordinate and yaw, and give it together with specified anchor coordinates q, and function below calculateUlos
    unit line of sight vectors. These can then be passed to azipe function and estimation compared to actual position
*/

std::vector<cv::Mat_<float>> calculateUlos(cv::Mat_<float> pos,float yaw,std::vector<cv::Mat_<float>> q){
    std::vector<cv::Mat_<float>> v;
    for(int i=0;i<q.size();i++){
        cv::Mat_<float> los = q[i]-pos;
        float len = std::sqrt(los(0,0)*los(0,0) + los(1,0)*los(1,0) + los(2,0)*los(2,0));//Calculate length
        los/=len;
        v.push_back(los);

    }
    return v;
}


int main(int argc, char** argv){
    //Define coordinates of anchors
    std::vector<cv::Mat_<float>> q;
    cv::Mat_<float> anchor1 = cv::Mat_<float>::zeros(3,1);q.push_back(anchor1);
    cv::Mat_<float> anchor2 = cv::Mat_<float>::zeros(3,1);anchor2(0,0) = 0.6;q.push_back(anchor2);
    //cv::Mat_<float> anchor3 = cv::Mat_<float>::zeros(3,1);anchor3(1,0) = 0.6;q.push_back(anchor3);
    //cv::Mat_<float> anchor4 = cv::Mat_<float>::zeros(3,1);anchor4(0,0) = 0.6;anchor4(1,0) = 0.6;q.push_back(anchor4);

    //Define true positions
    std::vector<std::vector<float>> pos;
    std::vector<float> pos1{0,0,-2};pos.push_back(pos1);
    std::vector<float> pos2{1,1,-2};pos.push_back(pos2);
    std::vector<float> pos3{1,0,-1.5};pos.push_back(pos3);
    std::vector<float> pos4{0,1,-1.3};pos.push_back(pos4);






    for(std::vector<float> i:pos){
        cv::Mat_<float> pos_mat = cv::Mat_<float>::zeros(3,1);
        pos_mat(0,0) = i[0];
        pos_mat(1,0) = i[1];
        pos_mat(2,0) = i[2];
        float yaw=0.1;
        std::vector<cv::Mat_<float>> v = calculateUlos(pos_mat,yaw,q);


        cv::Mat_<float> pos_mat_2;
        pos_mat.copyTo(pos_mat_2);
        //Add some noise to initial guess
        pos_mat_2(0,0) +=0.15;
        pos_mat_2(1,0) +=0.2;
        pos_mat_2(2,0) -=0.25;
        az::azipe(v,q,pos_mat_2,yaw,0,0);
        std::cout << "Diff: " << pos_mat.t() - pos_mat_2.t() << "Yaw: " << yaw << std::endl;
    }

}
