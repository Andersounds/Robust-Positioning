#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "vopos.hpp"


/*
Master positioning class constructor. Is called after the inherited classes are constructed
Inherited class constructors are called with relevant arguments after the ":" in the initialization list.
Rename to vispos? - Visual Positioning
*/
pos::positioning::positioning(int opticalFlow_mode,
                                int visualOdometry_mode,
                                int arucoDictionary,                            //Example: cv::aruco::DICT_4X4_50
                                int maxID,
                                std::string anchorPath,
                                int flowGrid,
                                cv::Rect2f roi_rect,
                                cv::Mat_<float> K,
                                cv::Mat_<float> T):
        ang::angulation(maxID,anchorPath),                          //Angulation constructor
        of::opticalFlow(opticalFlow_mode,flowGrid,roi_rect.width),  //Optical flow constructor
        vo::planarHomographyVO(visualOdometry_mode),                //Homography constructor
        roi(roi_rect)                                               //Assign argument to positioning attribute
{
    //Set some settings for angulation object
    ang::angulation::setKmat(K);
    ang::angulation::setTmat(T);
    minAnchors = 3;//Descider wether to try angulation or not
    //Initialize the aruco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(arucoDictionary);
    //Set some settings for Optical Flow object
    of::opticalFlow::setDefaultSettings();
    //Set some settings for Visual Odometry object
    vo::planarHomographyVO::setKmat(K,roi_rect);
    vo::planarHomographyVO::setTmat(T);
    vo::planarHomographyVO::setDefaultSettings();
}


//New master positioning functions. With illustration

/*
 *
 * This one is purely azipe. keep it separate so we know that no other estimation is done by accident
 */
int pos::positioning::process_AZIPE(cv::Mat& frame, cv::Mat& outputFrame,cv::Mat_<float>& pos, pos::argStruct& arguments){

//Aruco detect
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    int returnMode = -1;
    std::vector<cv::Mat_<float>> q_m,v_m;// q_masked and v_masked
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);//Detect markers
    std::vector<cv::Mat_<float>> q;//q and q_masked
    std::vector<bool> mask;
    int knownAnchors = dataBase2q(ids,q,mask);
    drawMarkers(outputFrame,corners,ids,mask);                          //Illustrate
    if(knownAnchors>=4){                                                //If enough anchors then do triangulation and break
        std::vector<cv::Mat_<float>> v;//v and v_masked;
        pix2uLOS(corners,v);
        ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
        ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
        az::azipe(v,q,pos,arguments.yaw,arguments.pitch,arguments.roll);
        returnMode = pos::RETURN_MODE_AZIPE;
    }else{
        returnMode = pos::RETURN_MODE_AZIPE_FAILED;
    }
    return returnMode;
}
/*
 * This one is azipe, but VO as fallback if it fails. Either arguments or positoning object initialization states
 *  if we should use homography or affine, and KLT or correlation based flow.
 *  It must be possible to force fallback method
 */
int pos::positioning::process_VO_Fallback(int mode,cv::Mat& frame, cv::Mat& outputFrame, cv::Mat_<float>& pos, pos::VOargStruct& arguments){
    static cv::Mat subPrevFrame; //Static init of previous subframe for optical flow field estimation
///////////// TRY AZIPE
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    int returnMode = -1;
    std::vector<cv::Mat_<float>> q_m,v_m;// q_masked and v_masked
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);//Detect markers
    std::vector<cv::Mat_<float>> q;//q and q_masked
    std::vector<bool> mask;
    int knownAnchors = dataBase2q(ids,q,mask);
    drawMarkers(outputFrame,corners,ids,mask);                          //Illustrate
    if(knownAnchors>=4 && mode != pos::MODE_FALLBACK){                  //If enough anchors then do triangulation unless overridden
        std::vector<cv::Mat_<float>> v;//v and v_masked;
        pix2uLOS(corners,v);
        ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
        ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
        az::azipe(v,q,pos,arguments.yaw,arguments.pitch,arguments.roll);
        returnMode = pos::RETURN_MODE_AZIPE;
    }else{
        /////////// VO Estimation
        std::vector<cv::Point2f> features;                                  //Must declare these before every calculation doe to being manipulated on
        std::vector<cv::Point2f> updatedFeatures;                           //The new positions estimated from KLT
        of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures); //Get flow field
        float scale = 5;                                                    //Illustrate
        cv::Point2f focusOffset(roi.x,roi.y);                               //Illustrate
        drawArrows(outputFrame,features,updatedFeatures,scale,focusOffset); //Illustrate
        cv::rectangle(outputFrame,roi,CV_RGB(255,0,0),2,cv::LINE_8,0);      //Illustrate
        bool vo_success = vo::planarHomographyVO::process(features,updatedFeatures,arguments.roll,arguments.pitch,arguments.dist,pos,arguments.yaw);
        if(!vo_success){returnMode = pos::RETURN_MODE_INERTIA;}
        else{returnMode = pos::RETURN_MODE_VO;}
    }
    frame(roi).copyTo(subPrevFrame);//Copy the newest subframe to subPrevFrame for use in next function call
    return returnMode;
}
/*
 * This one is azipe, but Marton as fallback if it fails. When implemented enough, some variable shall state degree of polynomial etc
 * It must be possible to force fallback method
 */
int pos::positioning::process_Marton_Fallback(int mode,cv::Mat& frame, cv::Mat& outputFrame, cv::Mat_<float>& pos,pos::MartonArgStruct& arguments){
    static marton::circBuff buffer(3); //Create a circular buffer with specified size for marton estimation

    ///////////// TRY AZIPE
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        int returnMode = -1;
        std::vector<cv::Mat_<float>> q_m,v_m;// q_masked and v_masked
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);//Detect markers
        std::vector<cv::Mat_<float>> q;//q and q_masked
        std::vector<bool> mask;
        int knownAnchors = dataBase2q(ids,q,mask);
        drawMarkers(outputFrame,corners,ids,mask);                          //Illustrate
        std::vector<cv::Mat_<float>> v;//v and v_masked;
        pix2uLOS(corners,v);
        ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
        ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
        if(knownAnchors>=4 && mode != pos::MODE_FALLBACK){                  //If enough anchors then do triangulation unless overridden
            az::azipe(v,q,pos,arguments.yaw,arguments.pitch,arguments.roll);
            returnMode = pos::RETURN_MODE_AZIPE;
        }else if(knownAnchors>1){

            int status = marton::process(v_m,q_m,pos,arguments.yaw, arguments.pitch, arguments.roll,arguments.time,buffer);

            //
            //
        }else{
            returnMode = pos::RETURN_MODE_AZIPE_FAILED;
        }

        //Add newest position estimation and yaw and time to circular buffer
        if(returnMode==pos::RETURN_MODE_AZIPE || returnMode==pos::RETURN_MODE_MARTON){
            buffer.add(pos,arguments.yaw,arguments.time);//Dont add if positioning failed
        }

        return returnMode;

}





/*
    A stub function to visualize derotation in order to verify function
*/
void pos::positioning::illustrateDerotation(cv::Mat& frame, cv::Mat& outputFrame,float dist,float& roll, float& pitch,float& yaw){
    //Define dummy position mat
    static cv::Mat_<float> t = cv::Mat_<float>::ones(3,1);

    static cv::Mat subPrevFrame; //Static init of previous subframe for optical flow field
    static float roll_prev = 0;
    static float pitch_prev = 0;

    std::vector<cv::Point2f> features;                                  //Must declare these before every calculation doe to being manipulated on
    std::vector<cv::Point2f> updatedFeatures;                           //The new positions estimated from KLT
    of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures); //Get flow field
//std::cout "Rollprev: " << roll_prev << ", \t Pitchprev: " << pitch_prev << std::endl;
//std::cout "Roll:     " << roll << ", \t Pitch:     " << pitch << std::endl;
    deRotateFlowField(features, roll_prev, pitch_prev);//Derotate
    deRotateFlowField(updatedFeatures, roll, pitch);//Derotate
    std::cout << "Is it correct? vopos:illustrateDerotation" << std::endl;
    float scale = 3;                                                    //Illustrate
    cv::Point2f focusOffset(roi.x,roi.y);                               //Illustrate
    drawArrows(outputFrame,features,updatedFeatures,scale,focusOffset); //Illustrate
    cv::rectangle(outputFrame,roi,CV_RGB(255,0,0),2,cv::LINE_8,0);      //Illustrate


    frame(roi).copyTo(subPrevFrame);//Copy the newest subframe to subPrevFrame for use in next function call
    roll_prev = roll;
    pitch_prev = pitch;
}

/* Draws a closed loop between all given points
 */
void pos::positioning::drawLines(cv::Mat& img,std::vector<cv::Point2f> points,cv::Point2f offset){
    int size = points.size();
    int i = 0;
    while(i<(size-1)){
        cv::line(img,offset+points[i],offset+points[i+1],CV_RGB(255,0,0),2,cv::LINE_8,0);
        i++;
    }
    cv::line(img,offset+points[i],offset+points[0],CV_RGB(255,0,0),2,cv::LINE_8,0);//close loop
}
/* Wrapper method for drawing anchors. Draws all found anchors, known in green and unknown in red
 */

 void pos::positioning::drawMarkers(cv::Mat& outputFrame, std::vector<std::vector<cv::Point2f> > corners,std::vector<int> ids,std::vector<bool> mask){
     std::vector<std::vector<cv::Point2f> > knownCorners;
     std::vector<int> knownIds;
     std::vector<std::vector<cv::Point2f> > unKnownCorners;
     std::vector<int> unKnownIds;
     for(int i=0;i<mask.size();i++){
         if(mask[i]){ //If anchor is known
             knownCorners.push_back(corners[i]);
             knownIds.push_back(ids[i]);
         }else{
             unKnownCorners.push_back(corners[i]);
             unKnownIds.push_back(ids[i]);
         }
     }
     cv::aruco::drawDetectedMarkers(outputFrame, knownCorners, knownIds, CV_RGB(0,250,0));
     cv::aruco::drawDetectedMarkers(outputFrame, unKnownCorners, unKnownIds, CV_RGB(250,0,0));
 }





/* Functions for defining roll, pitch, and yaw rotation matrices
 * Increase speed by passing reference and edit in place?
 */
cv::Mat pos::positioning::getXRot(float roll){
    float sinX = std::sin(roll);
    float cosX = std::cos(roll);
    cv::Mat_<float> R_x = cv::Mat_<float>::zeros(3,3);
    R_x(0,0) = 1;
    R_x(1,1) = cosX;
    R_x(1,2) = -sinX;
    R_x(2,1) = sinX;
    R_x(2,2) = cosX;
    return R_x;
}
cv::Mat pos::positioning::getYRot(float pitch){
    float sinY = std::sin(pitch);
    float cosY = std::cos(pitch);
    cv::Mat_<float> R_y = cv::Mat_<float>::zeros(3,3);
    R_y(0,0) = cosY;
    R_y(0,2) = sinY;
    R_y(1,1) = 1;
    R_y(2,0) = -sinY;
    R_y(2,2) = cosY;
    return R_y;
}
cv::Mat pos::positioning::getZRot(float pitch){
    float sinY = std::sin(pitch);
    float cosY = std::cos(pitch);
    cv::Mat_<float> R_y = cv::Mat_<float>::zeros(3,3);
    R_y(0,0) = cosY;
    R_y(0,1) = -sinY;
    R_y(1,0) = sinY;
    R_y(1,1) = cosY;
    R_y(2,2) = 1;
    return R_y;
}
