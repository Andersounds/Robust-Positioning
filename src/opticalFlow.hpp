#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H
/*
This file defines all prototypes for optical flow calculation for use with the Visual Odometry-implementation
Two methods of calculating optical flow is implemented, and the choice between them is done by setting a flag
on instantiation

Method 1. Lukas Kanade pyramidal optical flow. Without corners. an x-by-x grid of points in region of interest is specified
    At each new frame, these points are regarded as "features to track". This method has in preliminary tests shown to be
    much more robust in low contrast environments than actual feature tracking. The inevitable noise is handled by RANSAC
    in the VO algorithm.
    key: USE_KLT
Method 2. Phase-correlation based optical flow. The RoI is divided into x-by-x squares. Each of these are phase correlated
    with the same square from the previous image, and from each square a single flow arrow is obtained
    key: USE_CORR
*/
namespace of{
    #define USE_HOMOGRAPHY 1
    #define USE_AFFINETRANSFORM 2





}
#endif
