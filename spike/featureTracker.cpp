#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "featureTracker.hpp"


//goodfeaturesToTrack
//FAST detection
//ORB detection?
//BRIEF + STAR
// Object tracking with libs from opencv.


/*Constructor*/
goodfeatureTracker::goodfeatureTracker(int number)
{
  settings.maxNoOfCorners = number;
  multiTracker = cv::MultiTracker::create();


}

int goodfeatureTracker::getFeaturesToTrack(cv::Mat& frame)
{
  cv::goodFeaturesToTrack(frame,corners,
    settings.maxNoOfCorners,
    settings.qualityLevel,
    settings.minDistance,
    settings.mask,
    settings.blockSize,
    settings.useHarris,
    settings.k);
  return 1;
}
int goodfeatureTracker::drawCorners(cv::Mat& frame)
{
  for(int i=0;i<corners.size();i++)
  {
    circle(frame, corners.at(i), 10,CV_RGB(0,255,0),2, 8,0);
  }
  return 1;
}

int goodfeatureTracker::initTracker(cv::Mat& frame)
{
  std::vector<cv::Rect2i> tempRoi(corners.size());
  for(int i=0;i<corners.size();i++)
  {
    //roi[1].x=3;
    tempRoi[i].x = corners[i].x-15;
    tempRoi[i].y = corners[i].y-15;
    tempRoi[i].width = 30;
    tempRoi[i].height = 30;

    cv::rectangle(frame, tempRoi[i], CV_RGB(0,255,0));
  }
  roi = tempRoi;//Copy like this only works for vector type
  //Initialize multitracker
  for(int i=0; i < roi.size(); i++)
  {
    multiTracker->add(cv::TrackerMIL::create(), frame, roi[i]);//Eller rect2i ?
    //multiTracker->add(cv::TrackerMedianFlow::create(), frame, roi[i]);
    //multiTracker->add(cv::TrackerTLD::create(), frame, roi[i]);
  }
  return 1;
}
int goodfeatureTracker::trackFeatures(cv::Mat& frame)
{
  multiTracker->update(frame);
  for(unsigned i=0; i<multiTracker->getObjects().size(); i++)
  {
    rectangle(frame, multiTracker->getObjects()[i], CV_RGB(0,255,0),2, 8,0);
  }
  return 1;
}

int goodfeatureTracker::update(cv::Mat& frame)
{
  //goodfeatureTracker::getFeaturesToTrack(frame);
  goodfeatureTracker::trackFeatures(frame);

  //goodfeatureTracker::drawCorners(frame);
  return 1;
}
