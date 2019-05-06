/*
 * Parses arguments to main.
 Improvements:

 Give either one or two arguments. First is mode and second is optional to specify path or cam.
 */
#include <iostream>

struct streamArguments{
  std::string datasetPath;
  int streamMode;
  int cam;
};



int argumentParser(int argc, char** argv, streamArguments& arguments){
    //std::vector<std::string> modes = {"1. USB cam0/[<specified cam>]",
    //                        "2. (rpi cam)",
    //                        "3. Dataset stream with hardcoded path/[<specified path>]"};

    if(argc==1){std::cout << "Not enough input arguments. please state stream mode (1,2,3)" << std::endl; return 0;}
  arguments.datasetPath = "";
  arguments.streamMode = std::stoi(argv[1]);
  arguments.cam = 0;

  if(argc == 2){return 1;}//Only mode given. go with default values
  else if((argc == 3) and (arguments.streamMode == 1)){arguments.cam          = std::stoi(argv[2]);return 1;}
  else if((argc == 3) and (arguments.streamMode == 3)){arguments.datasetPath  = argv[2];return 1;}
  else {
    std::cerr << "Usage: <mode> [<path to dataset>]/[<usb cam number>]" << std::endl;
    //std::cout << "Available modes:" << std::endl;
    //for (std::vector<std::string>::iterator it = modes.begin() ; it != modes.end(); ++it)
    //  {std::cout << " " << *it << std::endl;}
    std::cerr << "Press enter..." << std::endl;
    std::cin.get();
    return 0;
  }
}
