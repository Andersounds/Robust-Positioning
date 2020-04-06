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
return 1;

}
