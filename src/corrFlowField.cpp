#include <iostream>
#include <opencv2/opencv.hpp>
/*
Maybe use opencv phase correlation????
Walter18 uses it

https://stackoverflow.com/questions/37143979/what-is-the-difference-between-phase-correlation-and-template-matching-in-opencv
See above. use phase correlation or
Split ROI into 9 or 4 non-overlapping sections. This is what  Walter18 does.

 Step 1: get perspective-adapted downward view. compensated for roll and pitch
 Step 2: Split into 4 or 9 parts.
 Step 3: perform phase correlation between each part to get 9 translations
 Step 4: This will then be a flow field that can be fed to the VO tracker as usual
*/

class corrFlowField{

    // a function to warp away roll and tilt
    // a function to perform correlation and return rotation and translation
};
