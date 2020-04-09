#include <iostream>
#include <opencv2/opencv.hpp>
#include "../src/martonRobust.hpp"
/*
g++ -std=c++11  /usr/local/lib/libgsl.a /usr/local/lib/libgslcblas.a `pkg-config --cflags --libs opencv` spike/martonTest.cpp src/martonRobust.cpp -o bin/marton
*/
int main(int argc, char** argv){




    //robustPositioning::martonRobust MR; //Create instance of class
    std::cout << "Hello" << std::endl;
    //MR.process();

    marton::process();


    marton::circBuff C(3);

    cv::Mat_<float> p(3,1);
    p(0,0) = 0;p(1,0) = 1;p(2,0)=2;
    float yaw = 3;
    float t = 0;
    for(int i = 0;i<6;i++){
        std::cout << "Addning t: " << t << std::endl;
        C.add(p,yaw,t);
        p+=4;
        yaw+=4;
        t+=1;
    }


    double p2[16];
    double t2[4];
    C.read_t(t2);
    C.read_p(p2);

    std::cout << "t2: ";
    for(int i =0;i<3;i++){
        std::cout << t2[i] << ", ";
    }
    std::cout << std::endl;

    std::cout << "p2: ";
    for(int i =0;i<12;i++){
        std::cout << p2[i] << ", ";
    }
    std::cout << std::endl;

return 1;

}
