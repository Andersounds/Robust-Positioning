#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "vopos.hpp"


// LP filter (1st order LP discretized using tustin)
class ZFilter{
    float wc;
    float t_prev;
    float value_prev;
    float value_filt_prev;
    public:
    ZFilter(float cutoff_){
        wc = cutoff_;
        t_prev = 0;
        value_prev = 0;
        value_filt_prev = 0;
    }
    float LPfilter(float value,float t){
        float dt = t-t_prev;
        float value_filt = ( (value + value_prev)*dt*wc + value_filt_prev*(2-dt*wc) ) / (2+dt*wc);
        t_prev = t;
        value_prev = value;
        value_filt_prev = value_filt;
        return value_filt;
    }
    //This method resets prev values and filters one indez in buffer in-place
    //Assumes that we are logging 4 values (x,y,z,yaw) at tPrev.size() buffer size.
    void filtZOnly(std::vector<float>& pPrev,std::vector<float> tPrev){
        int bufferSize = tPrev.size();
        int index = 2; //Which index should be filtered [0,1,2,3] <=> [x,y,z,yaw]
        //Reset start value of filter
        value_prev = pPrev[index];
        value_filt_prev = pPrev[index];
        t_prev = tPrev[0];
        for(int i=1;i<bufferSize;i++){//Start at second (1) index due to reset with first (0)
            int dataIndex= i*4+index;
            float filtered = LPfilter(pPrev[dataIndex],tPrev[i]);
            pPrev[dataIndex] = filtered;
        }
    }
};
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
    T.copyTo(T_vopos);
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
    if(knownAnchors>=minAnchors){                                                //If enough anchors then do triangulation and break
        std::vector<cv::Mat_<float>> v;//v and v_masked;
        pix2uLOS(corners,v);
        ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
        ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
        az::azipe(v_m,q_m,pos,arguments.yaw,arguments.pitch,arguments.roll);
        float thresh = 0.01;
        if(knownAnchors>4 && false){// Do not use AIPE
        //    az::aipe(v_m, q_m, pos, arguments.yaw,arguments.pitch, arguments.roll,thresh);
        //    returnMode = pos::RETURN_MODE_AIPE;
        }else{
            returnMode = pos::RETURN_MODE_AZIPE;
        }
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
    static float prevRoll = arguments.roll;
    static float prevPitch = arguments.pitch;
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
    if(knownAnchors>=minAnchors && mode != pos::MODE_FALLBACK){                  //If enough anchors then do triangulation unless overridden
        std::cout << "  MODE: AZIPE"<< std::endl;
        std::vector<cv::Mat_<float>> v;//v and v_masked;
        pix2uLOS(corners,v);
        ang::angulation::maskOut(q,q_m,mask);//Mask out q so it can be passed to azipe
        ang::angulation::maskOut(v,v_m,mask);//mask out v so it can be passed to azipe
        az::azipe(v_m,q_m,pos,arguments.yaw,arguments.pitch,arguments.roll);
        //az::aipe(v_m,q_m,pos,arguments.yaw,arguments.pitch,arguments.roll,0.01);

        returnMode = pos::RETURN_MODE_AZIPE;
    }else{
        /////////// VO Estimation
        std::vector<cv::Point2f> features;                                  //Must declare these before every calculation doe to being manipulated on
        std::vector<cv::Point2f> updatedFeatures;                           //The new positions estimated from KLT
        of::opticalFlow::getFlow(subPrevFrame,frame(roi),features,updatedFeatures); //Get flow field
        bool vo_success = vo::planarHomographyVO::process(features,updatedFeatures,arguments.roll,arguments.pitch,arguments.dist,pos,arguments.yaw,prevRoll,prevPitch);
        float scale = 5;                                                    //Illustrate
        cv::Point2f focusOffset(roi.x,roi.y);                               //Illustrate
        drawArrows(outputFrame,features,updatedFeatures,scale,focusOffset); //Illustrate
        cv::rectangle(outputFrame,roi,CV_RGB(255,0,0),2,cv::LINE_8,0);      //Illustrate
        if(!vo_success){
            std::cout << "  MODE: VO (failed)"<< std::endl;
            returnMode = pos::RETURN_MODE_INERTIA;}
        else{std::cout << "  MODE: VO"<< std::endl;
            returnMode = pos::RETURN_MODE_VO;}
    }
    frame(roi).copyTo(subPrevFrame);//Copy the newest subframe to subPrevFrame for use in next function call
    prevRoll = arguments.roll;
    prevPitch = arguments.pitch;
    return returnMode;
}
/*
 * This one is azipe, but Marton as fallback if it fails. When implemented enough, some variable shall state degree of polynomial etc
 * It must be possible to force fallback method
 */
int pos::positioning::process_Marton_Fallback(int mode,cv::Mat& frame, cv::Mat& outputFrame, cv::Mat_<float>& pos,pos::MartonArgStruct& arguments){
    static bool init = false;
    static int bufferSize = arguments.bufferSize;
    static marton::circBuff tBuffer(bufferSize);    //For previous time stamps
    if(!init){
        init = true;
        tBuffer.add(-4*TSPAN_MAX);//To prevent that marton starts before we even have azipe estimations
        tBuffer.add(-3*TSPAN_MAX);
        tBuffer.add(-2*TSPAN_MAX);
    }
    static marton::circBuff pBuffer(4*bufferSize);   //For previous positions (4 variables times buffersize)


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
        //std::cout << "Known anchors: " << knownAnchors << std::endl;
        std::vector<float> tPrev(bufferSize);
        tBuffer.read(tPrev);
        float tspan = arguments.time - tPrev[0];
        std::cout << "Tspan: " << tspan << std::endl;
        if((knownAnchors>=minAnchors) && (mode != pos::MODE_FALLBACK)){                  //If enough anchors then do triangulation unless overridden
            std::cout << "  MODE: AZIPE." << std::endl;
            az::azipe(v_m,q_m,pos,arguments.yaw,arguments.pitch,arguments.roll);
            returnMode = pos::RETURN_MODE_AZIPE;
            std::cout << "Azipe:  X: "<< pos(0,0) << ", Y: "<< pos(1,0) << ", Z: " << pos(2,0) << ", yaw: " << arguments.yaw<< std::endl;
        }else if(knownAnchors>=1 && tspan<TSPAN_MAX){//Only try marton if total time span is less than TSPAN_MAX ms
            std::cout << "  MODE: MARTON." << std::endl;
            std::vector<float> pPrev(4*bufferSize);
            pBuffer.read(pPrev);
            ZFilter Zfilt(0.007);
            Zfilt.filtZOnly(pPrev,tPrev);//FILTER JUST Z VALUES
            int _returnMode = marton::process(v_m,q_m,pos,arguments.yaw, arguments.pitch, arguments.roll,arguments.time,pPrev,tPrev,arguments.coneWeight);
            switch(_returnMode){
                case GSL_SUCCESS:{returnMode = pos::RETURN_MODE_MARTON;break;}
                case GSL_ENOPROG:{returnMode = pos::RETURN_MODE_MARTON_FAILED;break;}
                default:{std::cout << "Unknown return value from marton::process in vopos>process_Marton_Fallback" << std::endl; returnMode = RETURN_MODE_MARTON_ERR;}
            }
            std::cout << "Marton: X: "<< pos(0,0) << ", Y: "<< pos(1,0) << ", Z: " << pos(2,0)  << ", yaw: " << arguments.yaw<< std::endl;
        //}else{
        } else if(knownAnchors < minAnchors){
            std::cout << "  MODE: AZIPE (failed)" << std::endl;
            returnMode = pos::RETURN_MODE_AZIPE_FAILED;
        } else{
            std::cout << "  MODE: MARTON (failed)" << std::endl;
            returnMode = pos::RETURN_MODE_MARTON_OLD;
        }

        //int size = 5;
        //std::vector<float> tX(5);
        //double tX[size];
        //buffer2.read(tX);
        //std::cout <<" T3: [" << tX[0] << ", " << tX[1]<<", " << tX[2] << ", " << tX[3] << ", " << tX[4] <<"]" << std::endl;


        //Add newest position estimation and yaw and time to circular buffer
        //maybe add marton estimation as well?
        std::cout << "Returnmode: " << returnMode << std::endl;
        if(returnMode==pos::RETURN_MODE_AZIPE){
//        if((returnMode==pos::RETURN_MODE_AZIPE)||(returnMode==pos::RETURN_MODE_MARTON)){
        //    std::cout << "Adding timestamp: " << arguments.time << " to buffer..." << std::endl;
        //    buffer.add(pos,arguments.yaw,arguments.time);//Dont add if positioning failed
            tBuffer.add(arguments.time); //test also with counter directly if not working
            pBuffer.add(pos(0,0));
            pBuffer.add(pos(1,0));
            pBuffer.add(pos(2,0));
            pBuffer.add(arguments.yaw);
            std::cout << "Added " << arguments.time << " to buffer" << std::endl;
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


/* Method for illustrating the current yaw. The method adds an arrow pointing towards +x in global system
*/
void pos::positioning::illustrateYaw(cv::Mat& img,float yaw){
    cv::Mat_<float> unitImg = cv::Mat_<float>::zeros(3,1);
    unitImg(0,0) = 1;
    float scale = 40;
    cv::Mat_<float> dir = scale*getZRot(-yaw)*T_vopos.t()*unitImg;

    cv::Point2f offset = cv::Point2f(320,50);
    cv::Point2f to =  cv::Point2f(dir(0,0),dir(0,1)) + offset;
    cv::Point2f txtPoint = offset + cv::Point2f(-scale*1.3,scale*1.3+5);
    cv::circle(img, offset, scale*1.2, CV_RGB(200,50,0), 1, cv::LINE_8, 0);
    cv::putText(img, "+X Global", txtPoint, cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(200,50,0), 1, cv::LINE_8, false);
    cv::arrowedLine(img,offset,to,CV_RGB(200,50,0),2,cv::LINE_8,0,0.1);
/* X-create arrow +x
    X-rotate using T
    X-rotate with -yaw
//Draw arrow. Must be 3 channel I guess

*/
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
cv::Mat pos::positioning::getZRot(float yaw){
    float sinZ = std::sin(yaw);
    float cosZ = std::cos(yaw);
    cv::Mat_<float> R_z = cv::Mat_<float>::zeros(3,3);
    R_z(0,0) = cosZ;
    R_z(0,1) = -sinZ;
    R_z(1,0) = sinZ;
    R_z(1,1) = cosZ;
    R_z(2,2) = 1;
    return R_z;
}
