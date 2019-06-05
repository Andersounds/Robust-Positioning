#ifndef POSITIONING_H
#define POSITIONING_H
/*
This class implements a complete positioning functionality, excluding Kalman fusing with IMU data

Init:
 - Specify which Aruco dictionary is to be used example cv::aruco::DICT_4X4_50
 - Read known locations of markers from specified csv-file
 - Specify what supporting algorithm is to be used (Visual Odometry or continous path assumption). Also specify what flow algorithm is to be used if VO is chosen

Positioning:
1.      Get image and current readings or roll and pitch of vehicle
2.      Locate and decode visible ArUco-markers
2,5.    FUTURE: If not all are known, perform PnP estimation to add unknown
3.      Enough anchors? -> Perform Azipe
3.      Not enough anchors or Azipe failed? -> Perform fallback estimation
*/






#endif
