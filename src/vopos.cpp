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
        if(knownAnchors>=4 && mode != pos::MODE_FALLBACK){                  //If enough anchors then do triangulation unless overridden
            std::vector<cv::Mat_<float>> v;//v and v_masked;
            pix2uLOS(corners,v);
            ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
            ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
            az::azipe(v,q,pos,arguments.yaw,arguments.pitch,arguments.roll);
            returnMode = pos::RETURN_MODE_AZIPE;
        }else{
            //
            //
            // Marton code here
            //Conbstruct data struct
            double p[12] = {0.4,1,1,0,
                            9.4,2,2,0,
                            26.4,3,3,0};
            double alpha[5] = {0,0,0,0.4,0.3};
            double t[4] = {0,1,2,3};
            double tf = 4;
            struct marton::poly2_data da = { p, alpha, t,tf};
            // Construct solver parameters struct
            double x_init[12] = { 1, 0, 0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0}; /* starting values. Maybe init these as last solution*/
            double weights[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
            double xtol = 1e-2;
            double gtol = 1e-2;
            struct marton::nlinear_lsqr_param param = {x_init,weights,xtol,gtol};
            int status = marton::nlinear_lsqr_solve_2deg(param,da);






            //
            //
        }

        //Add newest position estimation and yaw and time to circular buffer
        if(returnMode==pos::RETURN_MODE_AZIPE || returnMode==pos::RETURN_MODE_MARTON){
            buffer.add(pos,arguments.yaw,arguments.time);//Dont add if positioning failed
        }
        return returnMode;

}





/*
Process the given data and update the position and yaw
*/
int pos::positioning::process(int mode,cv::Mat& frame, float dist,float roll, float pitch,float& yaw, cv::Mat_<float>& pos){
    static bool init = false;
    static cv::Mat subPrevFrame; //Static init of previous subframe for optical flow field
    static cv::Mat_<float> pos_init_est;// Variable that is passed to azipe as initial position guess
    if(!init){
        pos.copyTo(pos_init_est);
        init = true;
    }
    //Vectors containing detected Aruco info
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    int returnMode;//Return integer showing what algorithm was used to calculate new position
    switch (mode){
        case pos::MODE_AZIPE:{
            // Detect markers, extract the known ones using the database
            cv::aruco::detectMarkers(frame, dictionary, corners, ids);
            std::vector<cv::Mat_<float>> q,q_m;//q and q_masked
            std::vector<cv::Mat_<float>> v,v_m;//v and v_masked
            std::vector<bool> mask;
            int knownAnchors = dataBase2q(ids,q,mask);//Count how many known anchors are found and return their coordinates from database (in q)
            if(knownAnchors < minAnchors){
                returnMode = pos::RETURN_MODE_AZIPE_FAILED;
            } else{
                pix2uLOS(corners,v);
                ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
                ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
                az::azipe(v,q,pos,yaw,pitch,roll);
                returnMode = pos::RETURN_MODE_AZIPE;
            }
            break;
        }
        case pos::MODE_AZIPE_AND_VO:{
            cv::aruco::detectMarkers(frame, dictionary, corners, ids);
            //Convert IDs to q-coordinates and count number of known anchors from database
            std::vector<cv::Mat_<float>> q,q_m;//q and q_masked
            std::vector<cv::Mat_<float>> v,v_m;//v and v_masked
            std::vector<bool> mask;
            int knownAnchors = dataBase2q(ids,q,mask);
            if(knownAnchors == 0){//do pure VO
                //Get flow field
                std::vector<cv::Point2f> features;
                std::vector<cv::Point2f> updatedFeatures; //The new positions estimated from KLT
                of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures);
                bool vo_success = vo::planarHomographyVO::process(features,updatedFeatures,roll,pitch,dist,pos,yaw);
                if(!vo_success) returnMode = pos::RETURN_MODE_INERTIA;
                else returnMode = pos::RETURN_MODE_VO;
            } else if(knownAnchors < minAnchors){//Use VO and projectionFusing
                std::vector<cv::Point2f> features;
                std::vector<cv::Point2f> updatedFeatures; //The new positions estimated from KLT
                of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures);
                bool vo_success = vo::planarHomographyVO::process(features,updatedFeatures,roll,pitch,dist,pos,yaw);
                std::vector<cv::Mat_<float>> v;
                pix2uLOS(corners,v);
                //projectionFusing(pos,q,v,mask,yaw,roll,pitch);
                returnMode = pos::RETURN_MODE_PROJ;
            } else{//Do pure Azipe
                pix2uLOS(corners,v);
                ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
                ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
                az::azipe(v,q,pos,yaw,pitch,roll);
                returnMode = pos::RETURN_MODE_AZIPE;
            }
            break;
        }
        case pos::MODE_VO:{
            std::vector<cv::Point2f> features;
            std::vector<cv::Point2f> updatedFeatures; //The new positions estimated from KLT
            of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures);
            bool vo_success = vo::planarHomographyVO::process(features,updatedFeatures,roll,pitch,dist,pos,yaw);
            if(!vo_success) returnMode = pos::RETURN_MODE_INERTIA;
            else returnMode = pos::RETURN_MODE_VO;
            break;
        }
        default:{
            std::cout << "Invalid process mode given to pos::positioning::process" << std::endl;
        }
    }
    frame(roi).copyTo(subPrevFrame);//Copy the newest subframe to subPrevFrame for use in next function call
    return returnMode;
}
/*
Process the given data and update position and yaw. Also illustrate by drawing on the outputFrame mat
dist - Distance from camera to the flow field plane.
Mode pos::AZIPE         -Only do azipe
Mode pos::VO            -Only do VO
Mode pos::AZIPE_AND_VO  -Try Azipe, fallback on VO

*/
int pos::positioning::processAndIllustrate(int mode,cv::Mat& frame, cv::Mat& outputFrame,int illustrate_flag,float dist,float& roll, float& pitch,float& yaw, cv::Mat_<float>& pos,float& noOfAnchors){
    static cv::Mat subPrevFrame; //Static init of previous subframe for optical flow field
    //Aruco detect
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    int knownAnchors;
    int returnMode = -1;

    std::vector<cv::Mat_<float>> q_m,v_m;// q_masked and v_masked
    std::vector<cv::Mat_<float>> q;//q and q_masked
    std::vector<bool> mask;

    if(mode!=pos::MODE_VO){//The only case where we do not start with aruco detection
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);//Detect markers

        knownAnchors = dataBase2q(ids,q,mask);
        noOfAnchors = (float)knownAnchors;
        drawMarkers(outputFrame,corners,ids,mask);                          //Illustrate
    }
    switch(mode){
        case pos::MODE_AZIPE:{  //Perform only AZIPE estimation
            if(knownAnchors>=4){                                                //If enough anchors then do triangulation and break
                std::vector<cv::Mat_<float>> v;//v and v_masked;
                pix2uLOS(corners,v);
                ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
                ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
                az::azipe(v,q,pos,yaw,pitch,roll);
                returnMode = pos::RETURN_MODE_AZIPE;
            }else{
                returnMode = pos::RETURN_MODE_AZIPE_FAILED;
            }
            break;
        }

        case pos::MODE_AZIPE_AND_VO:{//Try Azipe and fall back to VO
            if(knownAnchors>=4){                                                //If enough anchors then do triangulation and break
                /////////// AZIPE Estimation
                std::vector<cv::Mat_<float>> v;//v and v_masked;
                pix2uLOS(corners,v);
                ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
                ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
                az::azipe(v,q,pos,yaw,pitch,roll);
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
                bool vo_success = vo::planarHomographyVO::process(features,updatedFeatures,roll,pitch,dist,pos,yaw);
                if(!vo_success){returnMode = pos::RETURN_MODE_INERTIA;}
                else{returnMode = pos::RETURN_MODE_VO;}
            }
            break;
        }
        case pos::MODE_AZIPE_AND_MARTON:{
            if(knownAnchors>=4){                                                //If enough anchors then do triangulation and break
                /////////// AZIPE Estimation
                std::vector<cv::Mat_<float>> v;//v and v_masked;
                pix2uLOS(corners,v);
                ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
                ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
                az::azipe(v,q,pos,yaw,pitch,roll);
                returnMode = pos::RETURN_MODE_AZIPE;
            }else{
                /*
                 *
                 *  Marton code here
                 *
                 */
            }
            break;
        }
        case pos::MODE_VO:{
            /////////// VO Estimation
            std::vector<cv::Point2f> features;                                  //Must declare these before every calculation doe to being manipulated on
            std::vector<cv::Point2f> updatedFeatures;                           //The new positions estimated from KLT
            of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures); //Get flow field
            float scale = 5;                                                    //Illustrate
            cv::Point2f focusOffset(roi.x,roi.y);                               //Illustrate
            drawArrows(outputFrame,features,updatedFeatures,scale,focusOffset); //Illustrate
            cv::rectangle(outputFrame,roi,CV_RGB(255,0,0),2,cv::LINE_8,0);      //Illustrate
            bool vo_success = vo::planarHomographyVO::process(features,updatedFeatures,roll,pitch,dist,pos,yaw);
            if(!vo_success){returnMode = pos::RETURN_MODE_INERTIA;}
            else{returnMode = pos::RETURN_MODE_VO;}
            break;
        }
        case pos::MODE_MARTON:{
            /*
             *
             *  Marton code here
             *
             */
        }
        default:{
            std::cerr << "INVALID MODE GIVEN TO vopos.cpp processAndIllustrate" << std::endl;
        }





    }


    std::cout << std::endl;
    /* Below example of how proj fallback was written
    case pos::RETURN_MODE_PROJ:{
        //Get flow field
        std::vector<cv::Point2f> features;
        std::vector<cv::Point2f> updatedFeatures; //The new positions estimated from KLT
        of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures);
        vo_success = vo::planarHomographyVO::process(features,updatedFeatures,roll,pitch,dist,pos,yaw);
        std::vector<cv::Mat_<float>> v;
        pix2uLOS(corners,v);
    //    projectionFusing(pos,q,v,mask,yaw,roll,pitch);
        break;
    }

    */
    frame(roi).copyTo(subPrevFrame);//Copy the newest subframe to subPrevFrame for use in next function call
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

/*
    A stub function used to eliminate everything except azipe
*/
int pos::positioning::processAz(int mode,cv::Mat& frame, cv::Mat& outputFrame,int illustrate_flag,float dist,float& roll, float& pitch,float& yaw, cv::Mat_<float>& pos,float& noOfAnchors){
    static bool init = false;
    static cv::Mat subPrevFrame; //Static init of previous subframe for optical flow field
    static cv::Mat_<float> pos_init_est;// Variable that is passed to azipe as initial position guess
    if(!init){
        pos.copyTo(pos_init_est);
        init = true;
    }
    //Aruco detect
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
std::cout << ids.size() << std::endl;
    //Only do angulation if at least two known anchors are visible
    int status = 0;
    int returnMode = pos::RETURN_MODE_AZIPE;
    //Convert IDs to q-coordinates and count number of known anchors from database
    std::vector<cv::Mat_<float>> q,q_m;//q and q_masked;
    std::vector<bool> mask;

    int knownAnchors = dataBase2q(ids,q,mask);
    noOfAnchors = (float)knownAnchors;

    // Draw detected markers and identify known markers
    drawMarkers(outputFrame,corners,ids,mask);

    bool vo_success = false;


    std::vector<cv::Mat_<float>> v,v_m;//v and v_masked;
    pix2uLOS(corners,v);
    //std::cout << "ANCHORS:    " << knownAnchors<< std::endl;
    if(knownAnchors>3){
        //pos_init_est.copyTo(pos);// Set initial position guess va? varför?
        ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
        ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
        az::azipe(v,q,pos,yaw,pitch,roll);
        std::cout << "k" << std::endl;
//        pos_init_est*=0.95;     //slow down movement of pos_init_est
//        pos_init_est+=(pos*0.05);//Used to prevent estimations in wrong sign z from causing panic
        pos_init_est = pos;

    }
    frame(roi).copyTo(subPrevFrame);//Copy the newest subframe to subPrevFrame for use in next function call
    return returnMode;
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
