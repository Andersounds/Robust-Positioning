//#include <boost/lambda/lambda.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <boost/program_options.hpp>
#include <iostream>
//#include <fstream> //Input stream from file
//#include <opencv2/opencv.hpp>



//https://www.pyimagesearch.com/2015/04/27/installing-boost-and-boost-python-on-osx-with-homebrew/
int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    po::options_description parameters("Options");
    parameters.add_options()
        ("help,h",  "Print help messages")
        ("OUT,o",   po::value<std::string>(), "Write output data to specified file. No output is not set")// Single string argument
        //Parameters
        ("RES_XY",  po::value<std::vector<int> >()->multitoken(), "Camera resolution in X and Y direction")
        ("K_MAT",   po::value<std::vector<float> >()->multitoken(), "Camera K matrix specified as float numbers row by row separated by whitespace") //Tänk om man kan definiera denna direkt som en opencv mat och ge 9 argument på rad?
        ("T_MAT",   po::value<std::vector<float> >()->multitoken(), "UAV - camera T matrix specified as float numbers row by row separated by whitespace")
        ("CAMERA_BARREL_DISTORTION",    po::value<std::vector<float> >()->multitoken(), "Barrel distortion coefficients K1, K2, K3 as floats")
        ("OPTICAL_FLOW_GRID",           po::value<int>(),"Sqrt of number of optical flow vectors")//Single int
        ("XYZ_INIT",                    po::value<std::vector<float> >()->multitoken(), "Initial position expressed as X Y Z coordinate floats")
        ("ROLL_INIT", po::value<float>(),"Initial roll of UAV, radians")
//        ("PITCH_INIT", po::value<float>(),"Initial pitch of UAV, radians")
//        ("YAW_INIT", po::value<float>(),"Initial yaw of UAV, radians")
        ("ROLL_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
//        ("PITCH_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
//        ("YAW_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, parameters), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << parameters << "\n";
        std::cout << "HELP" << std::endl;
        return 1;
    }

    if (vm.count("ROLL_COLUMN")) { //Funkar
        std::cout << "Compression level was set to "
     << vm["ROLL_COLUMN"].as<int>() << ".\n";
    } else {
        std::cout << "CRES_XY was not set.\n";
    }
    if (vm.count("OUT")) { //Funkar
        std::cout << "output file: " << vm["OUT"].as<std::string>() << std::endl;
    }else{
        std::cout << "OUT was not set.\n";
    }
    if (vm.count("ROLL_INIT")) { //Funkar
        std::cout << "Float:" << vm["ROLL_INIT"].as<float>() << std::endl;
    }else{
        std::cout << "float not set.\n";
    }

    if (vm.count("RES_XY")) { //funkar
        std::vector<int> resolution = vm["RES_XY"].as<std::vector<int> >();
        for(int i:resolution){
            std::cout << i << ", ";
        }
        std::cout << std::endl;
    }else{
        std::cout << "res int vector not set.\n";
    }


}

//g++ -std=c++11 -fvisibility=hidden /usr/local/lib/libboost_program_options.a examples/boost_compile_test.cpp -o bin/example
