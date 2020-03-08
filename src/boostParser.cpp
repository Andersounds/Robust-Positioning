/*
This is an implementation of a settingsparser using the program_options module of the Boost-library.
It is intended to replace the settingsParser.cpp implementation.
*/
/*
[Documentation](https://www.boost.org/doc/libs/1_72_0/doc/html/program_options.html)
How it works:
    - the ::options_description class is used to store all options. Options are added with
        - ::options_description::add_options with a bit wierd syntax. See below.


        boost installed with homebrew and is in dir  usr/local/Cellar/boost/1.72.0

        -Default values!. How add?
*/


#include "boost/program_options.hpp"


namespace set2{


namespace po = boost::program_options;


    po::options_description parameters("Options"); // Class containing descriptions of all options
    parameters.add_options()
      ("help", "Print help messages")
      ("OUT", po::value<std::string>(), "Write output data to specified file. No output is not set")// Single string argument
      //Parameters
      ("RES_XY", po::value<std::vector<int> >()->multitoken(), "Camera resolution in X and Y direction")
      ("K_MAT", po::value<std::vector<float> >()->multitoken(), "Camera K matrix specified as float numbers row by row separated by whitespace") //Tänk om man kan definiera denna direkt som en opencv mat och ge 9 argument på rad?
      ("T_MAT", po::value<std::vector<float> >()->multitoken(), "UAV - camera T matrix specified as float numbers row by row separated by whitespace")
      ("CAMERA_BARREL_DISTORTION", po::value<std::vector<float> >()->multitoken(), "Barrel distortion coefficients K1, K2, K3 as floats")
      ("OPTICAL_FLOW_GRID",po::value<int>(),"Sqrt of number of optical flow vectors")//Single int
      ("XYZ_INIT", po::value<std::vector<float> >()->multitoken(), "Initial position expressed as X Y Z coordinate floats")
      ("ROLL_INIT", po::value<float>(),"Initial roll of UAV, radians")
      ("PITCH_INIT", po::value<float>(),"Initial pitch of UAV, radians")
      ("YAW_INIT", po::value<float>(),"Initial yaw of UAV, radians")
      ("ROLL_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
      ("PITCH_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
      ("YAW_COLUMN", po::value<int>(),"Specifies which column of csv file that contains roll data (0-indexed)")
      //setDefault("STREAM_IMAGES_BASEPATH", "images/","Path from base path -p to image directory that also contains images info file. ending with /");
      //setDefault("STREAM_IMAGES_INFO_FILE", "imageData.csv","File (located in IMAGES_BASEPATH with image data. (timestamps image name))");
      //setDefault("STREAM_DATA_FILE", "imudata.csv","Path to csv file to be streamed");
      // Modes
       //("OPTICAL_FLOW_MODE")// How set this? Use a define maybe?
       // pos est mode, optical flow mode, triangulation mode?
      ;





    po::variables_map vm;



}
