#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include <unistd.h> //For sleep
#include "../src/VisPos.cpp"
#include "../src/videoStream.cpp"
#include "../src/argumentParser.cpp"

using namespace cv;
//using namespace std;


void getRandomColors(vector<Scalar>& colors, int numColors)
{
  RNG rng(0);
  for(int i=0; i < numColors; i++)
    colors.push_back(Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255)));
}


// create tracker by name

Ptr<Tracker> createTrackerByName(string trackerType)
{
  vector<string> trackerTypes = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};

  Ptr<Tracker> tracker;
  if (trackerType ==  trackerTypes[0])
    tracker = TrackerBoosting::create();
  else if (trackerType == trackerTypes[1])
    tracker = TrackerMIL::create();
  else if (trackerType == trackerTypes[2])
    tracker = TrackerKCF::create();
  else if (trackerType == trackerTypes[3])
    tracker = TrackerTLD::create();
  else if (trackerType == trackerTypes[4])
    tracker = TrackerMedianFlow::create();
  else if (trackerType == trackerTypes[5])
    tracker = TrackerGOTURN::create();
  else if (trackerType == trackerTypes[6])
    tracker = TrackerMOSSE::create();
  else if (trackerType == trackerTypes[7])
    tracker = TrackerCSRT::create();
  else {
    cout << "Incorrect tracker name" << endl;
    cout << "Available trackers are: " << endl;
    for (vector<string>::iterator it = trackerTypes.begin() ; it != trackerTypes.end(); ++it)
      std::cout << " " << *it << endl;
  }
  return tracker;
}





int main(int argc, char** argv){
  // check if inputs are correctly given
  streamArguments arguments;
  if(!argumentParser(argc,argv,arguments)){return 0;}

  Streamer* C;
  C = Streamer::make_streamer(arguments.streamMode,arguments.datasetPath,arguments.cam);

  cv::Mat freshFrame, prevFrame, nextFrame;

//****************************************************

C->getImage(prevFrame);


vector<Rect> bboxes;

bool showCrosshair = true;
bool fromCenter = false;
cout << "\n==========================================================\n";
cout << "OpenCV says press c to cancel objects selection process" << endl;
cout << "It doesn't work. Press Escape to exit selection process" << endl;
cout << "\n==========================================================\n";
cv::selectROIs("MultiTracker", prevFrame, bboxes, showCrosshair, fromCenter);

// quit if there are no objects to track
if(bboxes.size() < 1)
  return 0;

vector<Scalar> colors;
getRandomColors(colors, bboxes.size());


// Specify the tracker type
string trackerType = "MIL";
// Create multitracker
Ptr<MultiTracker> multiTracker = cv::MultiTracker::create();

// Initialize multitracker
for(int i=0; i < bboxes.size(); i++)
  multiTracker->add(createTrackerByName(trackerType), prevFrame, Rect2d(bboxes[i]));


//*****************************************


/*
Get features to track from a frame
*/
while(1){
  C->getImage(nextFrame); //Initialize with a frame
  if(nextFrame.empty()){return 1;}
  //cv::cvtColor(nextFrame, nextFrame, COLOR_BGR2GRAY);
//*****************************************************
//Update the tracking result with new frame
  multiTracker->update(nextFrame);

  // Draw tracked objects
  for(unsigned i=0; i<multiTracker->getObjects().size(); i++)
  {
    rectangle(nextFrame, multiTracker->getObjects()[i], colors[i], 2, 1);
  }

  // Show frame
  imshow("MultiTracker", nextFrame);

  // quit on x button
  //if  (waitKey(1) == 27) break;




//******************************************************
  nextFrame.copyTo(prevFrame);
  //cv::imshow("Showit",nextFrame);
  if( cv::waitKey(1) == 27 ) {std::cout << "Bryter"<< std::endl;return 1;}



  }

}
