//#include <boost/lambda/lambda.hpp>
#include <iostream>
#include <fstream> //Input stream from file
//#include <iterator>
//#include <algorithm>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
//#include <iostream>
//#include <fstream> //Input stream from file
#include <opencv2/opencv.hpp>
/*
 PARSE COMMAND LINE:
    (First define all options. As well as the config file name)
    - read argc argv
    - is config file option given?
        - Then pare config file
    - Then read rest of command line.

    When variable are defined from value map. If needed then include conversion function


    - How to write default ini file?
    - How to let command line override ini file?
        MULTIPLE SOURCES: https://www.boost.org/doc/libs/1_72_0/doc/html/program_options/tutorial.html
        DONE.
    - Sectioning. See abit in link above
*/


/*
    How its is to be used.
        - Every program defines their own program options.
        - This is done in a separate function defined in the main file
        - The function defines options, their default values
        - Reads argc argc
        - For each option, one conversion is done to a correct format, and then added to a map with the same keys
        - This map is passed back to main, which accesses values using the same keys as settings.


        - How to write config file?
        - Order options in categories
            -multiple sources
                https://www.boost.org/doc/libs/1_72_0/doc/html/program_options/tutorial.html
        - Write parameter description as comment in config file
            https://www.boost.org/doc/libs/1_55_0/doc/html/program_options/overview.html#idp163379208
        - config file multitoken
            - in ini-file, one line can only be one value.
            - How can multitoken be handled properly? Ideas below.
                1. Do not use multitoken. Instead specify as single string and write parser for it.
                    - Simple parser that
                        -Goes through whole string
                            -if char is numeric
                            -read all numbers in a row, allowing a single point '.' somewhere
                            -Convert this string to either int or float depending on option
                        - This will allow to write pretty much however. [1,2,3;4,5,6] | 1 2 3 4 5 6 etc.


    Additional options:
        - Write default settings file
            - Possibly at some specified path
        - Settings file must be given as argument.
            - Can give just settings file, and its directory will be parsed, and all paths in file are relative to this path
            - Option to specifically specify base path, i.e path to some other dataset. Other paths will then be relative to this instead
        - ARguments given om command line should override arguments in ini file.
*/


/*
    New comments regarding function of settings parser
        - Every program should probably define options in the main function itself. That way relevance is kept
        - No multitoken works due to ini file. But I can write a separate set of overloaded functions that parse strings of matlab style to either float or int mat or vector
        - Should maybe write a class that contains all boost program options including 2 additional methods:
            - conversion methors as stated above
            - function to write default ini file including sections and comments


*/



/*
    Method that takes a string, and converts it to a opencv mat_<float>
    Possibly overload this with int versions
*/

//int string2CVMat(std::string str0,cv::Mat_<float>& cvMat){
int string2CVMat(std::string str0, cv::Mat_<float>& M){
    boost::trim_if(str0,boost::is_any_of("[]"));//Trim brackets
    std::vector<std::string> SplitVec;
    boost::split(SplitVec, str0, boost::is_any_of(";"));//Split into rows
    int rows = SplitVec.size();

    std::vector<float> V;
    int cols = -1;
    for(std::string rowStr:SplitVec){//
        std::vector<std::string> row;//Split row string to string elements
        boost::split(row, rowStr, boost::is_any_of(","));//Must have ',' as column delimiter
        cols = row.size();//Will be redefined for every row but must always be same so whatever
        for(std::string i:row){
            std::cout << "In string2CVMat: Split this into several rows and use try catch to specify what kind of error!" << std::endl;
            std::string elementSTR = boost::trim_copy(i);
            float element = std::stof(elementSTR);
            V.push_back(element);//remove whitespaces, convert to float and push back
        }
    }
    cv::Mat_<float> V2;
    try{
        V2 = cv::Mat(V).reshape(cols);
    }catch(...){
        std::cerr << "ERROR: Specified matrix does not have consistent number of columns" << std::endl;
        std::cerr << "In string2CVMat" << std::endl;
        throw(1);
    }
    try{
        V2.copyTo(M); //Do this inside another try block to catch specific error
        return 0;
    }catch(...){
        std::cerr << "ERROR: Could not copy parsed matrix onto inputoutput cv mat." << std::endl;
        std::cerr << "In string2CVMat" << std::endl;
        throw(1);
    }
return 1;
}

//    std::cout << generic << std::endl; options description can be printed out

/*
    This function is to specify all options. Unique for all programs.
*/
int readCommandLine(int argc, char** argv,boost::program_options::variables_map& vm){

    // Declare a group of options that will be
    // allowed only on command line
    namespace po=boost::program_options;
    po::options_description generic("Command line options");
    generic.add_options()
        ("help,h", "produce help message")
        ("file,f",po::value<std::string>(),"configuration file")// Possibly set this as first positional option?
    ;

    po::options_description parameters("Parameters");
    parameters.add_options()
        //Parameters
        ("RES_XY",  po::value<std::vector<int> >(), "Camera resolution in X and Y direction")
        ("RES_X",  po::value<int>(), "Camera resolution in X direction")
        ("RES_Y",  po::value<int>(), "Camera resolution in X direction")
        ("K_MAT",  po::value<std::string>()->default_value("[607.136353,0,320;0,607.136353,240;0,0,1]"), "Camera K matrix specified as float numbers row by row separated by whitespace") //Tänk om man kan definiera denna direkt som en opencv mat och ge 9 argument på rad?
        ("T_MAT",  po::value<std::string>()->default_value("[0,1,0;-1,0,0,0,0,1]"), "UAV - camera T matrix specified as float numbers row by row separated by whitespace")
        ("CAMERA_BARREL_DISTORTION",    po::value<std::string>()->default_value("[0.2486857357354474,-1.452670730319596,2.638858641887943]"), "Barrel distortion coefficients given as [K1,K2,K3]")
        ("OPTICAL_FLOW_GRID",           po::value<int>()->default_value(4),"Sqrt of number of optical flow vectors")//Single int
        ("ROI_SIZE",po::value<int>()->default_value(150), "Side length of VO ROI. Used to edit K mat of VO alg.")
        ;

    po::options_description initValues("Initial values");
    initValues.add_options()
        ("XYZ_INIT",                    po::value<std::string>()->default_value("[0,0,-1.8]"), "Initial position expressed as [X,Y,Z] coordinate floats")
        ("ROLL_INIT", po::value<float>()->default_value(0),"Initial roll of UAV, radians")
        ("PITCH_INIT", po::value<float>()->default_value(0),"Initial pitch of UAV, radians")
        ("YAW_INIT", po::value<float>()->default_value(0),"Initial yaw of UAV, radians")

        ;
    po::options_description modes("Program settings");
    modes.add_options()
        ("OUT,o",   po::value<std::string>()->default_value("[9,9,9]"), "Write output data to specified file. No output is not set")// Single string argument
        ("TILT_COLUMNS", po::value<std::string>()->default_value("[4,3]"),"Specifies which columns of csv file that contains [roll,pitch] data (0-indexed)")
        ("DIST_COLUMN", po::value<int>()->default_value(1),  "Specifies which column of csv file that contains distance (lidar) data")
        ("PATH_TO_ARUCO_DATABASE", po::value<std::string>()->default_value("anchors.csv"),"Path to anchor database from base path")
        ("BASE_PATH",po::value<std::string>(),"Base path from which I/O paths are relative. Defaults to pwd but may be overridden with this flag.\nGive as either absolute or relative path.")
        //Maybe this one? setDefault("MAX_ID_ARUCO", (int) 50,"Database size. must be able to contain all IDs in ARUCO_DICT_TYPE");

        //Possibly add aruco dict mode
        // VO mode
        // angulation mode
        // positioning mode?
/*
        setDefault("OPTICAL_FLOW_MODE", (int) 0," KLT (1) or correlation (2) based optical flow");
        setDefault("OPTICAL_FLOW_GRID", (int) 4," sqrt of total amount of points used in optical flow.");
        setDefault("VISUAL_ODOMETRY_MODE", (int) 0," Homography (1) or Affine (2) odometry estimation");
        setDefault("POS_EST_MODE", (int) 0,"Azipe+VO (0), AZIPE (1), VO (2), (todo: benchmark, benchmark+azipe)");
        LOG_MODE
*/
        ;





    // Parse command line
    //po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, generic), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << "Allowed parameters:" << std::endl;
        std::cout << generic<< std::endl;
        std::cout << parameters<< std::endl;
        std::cout << initValues<< std::endl;
        std::cout << modes << std::endl;
        std::cout << "---------------" << std::endl;
        std::cout << "Paramaters may be given in initialization file using flag -f. File format:\n";
        std::cout << "Format: \nPARAMETER_FLAG_1 = <value>   #Disregarded comment\nPARAMETER_FLAG_2 = <value>   #Some other comment" << std::endl;
        return 0;
    }
    if(vm.count("file")){
        std::string iniFile = vm["file"].as<std::string>();
        std::cout << "Reading configuration file " << iniFile << "..." << std::endl;
        std::ifstream ini_file(iniFile);//Try catch block?
        po::store(po::parse_config_file(ini_file, parameters, true), vm);
        po::notify(vm);

        po::store(po::parse_config_file(ini_file, initValues, true), vm);
        po::notify(vm);
        po::store(po::parse_config_file(ini_file, modes, true), vm);
        po::notify(vm);
    }
    if(vm.count("K_MAT")){
        std::cout<< "Read K mat as string: " << vm["K_MAT"].as<std::string>() << std::endl;
        cv::Mat_<float> M;
        string2CVMat(vm["K_MAT"].as<std::string>(), M);
        std::cout << "K mat as cv float mat:\n" << M << std::endl;


    }
    return 0;
}

/*
INI file of following syntax


ROLL_COLUMN=4 #Works
OUT=testi 1 2 4     #Works
ROLL_INIT=1.2   #Works



Will add string2Mat which will allow:
K_MAT = [1,2,3;1,2,4;1,2.5,3]
*/


//https://www.pyimagesearch.com/2015/04/27/installing-boost-and-boost-python-on-osx-with-homebrew/
int main(int argc, char** argv)
{

    boost::program_options::variables_map vm;

    readCommandLine(argc, argv,vm);
    std::cout << "done" << std::endl;

}

//g++ -std=c++11 -fvisibility=hidden /usr/local/lib/libboost_program_options.a examples/boost_compile_test.cpp -o bin/example
