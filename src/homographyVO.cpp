#include <iostream>
#include <opencv2/opencv.hpp>
#include "homographyVO.hpp"
    /*
    This is the "Third" attempt att writing a visual odometry method
    The method will use:
        - Optical flow based on goodfeaturestotrack and KLT
        - (Some outlier rejection algorithm)
        --------- Homography based VO -----
        - Assume planar motion and constant psi,theta between frames (see p. 56 of phd thesis Wadenbäck)
        - de-rotate the using a psi and theta rotation from g-vector. p67 i phd I believe by de-rotating H
        - cv::findHomography using RANSAC
        - Opencv method of decomposing H. This will also be given a plane normal vector est. from IMU
        - Disregarding of invalid solutions to H decomposition
        - Scaling using distance sensor.
    The method will NOT use:
        - Any other method optical flow method of obtaining the correspondance pairs
        - De-rotation of flow field using IMU rotation rates (even though this may simplify decomposition)
        - The "hand-eye calibration problem" to account for the fact that rotational center is not at camera+imu origin
    */
/*
This constructor first edits the given K-matrix to work with the given region of interest.
Assumption! centered roi. If ROI is not centered then instead call the other constructor and define K metrix before
*/
vo::planarHomographyVO::planarHomographyVO(int mode_){
    mode = mode_;
}
/*Constructor. Defines some matrices and settings
 * K_:  Intrinsic camera matrix
 * T_:  Camera frame orientation in relation to UAV coordinate frame
 */
vo::planarHomographyVO::planarHomographyVO(cv::Mat_<float> K_, cv::Mat_<float> T_,int mode_){
    std::cout << "TODO: 1. de-rotate both roll and pitch." <<std::endl;
    std::cout << "      2. Scale flow correctly when camera is tilted."<< std::endl;
    K=K_;
    K_inv = K.inv();
    T=T_;             //Rotation matrix around Z relating UAV with Camera (From UAV to Camera)
    T_inv = T.inv();
    std::cout << "Flow field de-rotation is active (planarHomographyVO::deRotateFlowField). " << std::endl;
    std::cout << "\t deactivate by setting .activateDerotation = false " << std::endl;
    mode = mode_;
}
/*
    Methods for setting K, and T matrix. The roi is used to offset cx,cy in K mat
*/
void vo::planarHomographyVO::setKmat(cv::Mat_<float> K_,cv::Rect2f roi){
    float c_x = roi.width/2;    //New x-offset to fit with the given roi
    float c_y = roi.height/2;   //New y-offset to fit with the given roi
    cv::Mat_<float> K_roi;
    K_.copyTo(K_roi);
    K_roi(0,2) = c_x;
    K_roi(1,2) = c_y;
    K_roi.copyTo(K);
    cv::Mat_<float> K_roi_inv = K_roi.inv();
    K_roi_inv.copyTo(K_inv);
}
void vo::planarHomographyVO::setTmat(cv::Mat_<float> T_){
    T_.copyTo(T);
    cv::Mat_<float> T__inv = T_.inv();
    T__inv.copyTo(T_inv);
}
/*
    Default settings function
*/
void vo::planarHomographyVO::setDefaultSettings(void){
    // findHomography parameters
    homoGraphyMethod = cv::RANSAC;
    ransacReprojThreshold = 3;
    maxIters = 2000;
    confidence = 0.995;
    // getAffineTransform parameters
    method_ = cv::RANSAC;
    ransacReprojThreshold_ = 3;
    maxIters_ = 2000;
    confidence_ = 0.995;
    refineIters_ = 10;
    //Other paramters
    sanityLimit = 10000;//Limit of sum of squares in insanitycheck.
    activateDerotation = true;

}
/*Complete process of odometry. Calls odometry method and updates t and R if possible
 * p1,p2:       Point correspondances (p1 old image, p2 new image)
 * roll,pitch:  Inter-image constant roll and pitch of camera that is to be compensated for
 * height:      height of camera over feature plane at image 1. note height = cos(roll)*(Z_distance) if pitch is 0
 * t:           Camera coordinate in global coordinate system
 * R:           Camera rotation in global coordinate system
 */
bool vo::planarHomographyVO::process(std::vector<cv::Point2f>& p1,
                                        std::vector<cv::Point2f>& p2,
                                        float roll,float pitch,float dist,
                                        cv::Mat_<float>& t,
                                        float& yaw,
                                        float prevRoll,float prevPitch){
                                        //cv::Mat_<float>& R){
    cv::Mat_<double> b_double;//odometry estimation of translation in camera frame
    cv::Mat_<double> A_double;//odometry estimation of rotation in camera frame
    bool success;
    switch(mode){
        case USE_AFFINETRANSFORM:{
            success = odometryAffine(p1,p2,roll,pitch,prevRoll,prevPitch,dist,b_double,A_double);
            break;
        }
        case USE_HOMOGRAPHY:{
            success = odometryHom(p1,p2,roll,pitch,prevRoll,prevPitch,dist,b_double,A_double);
            break;
        }
        default:{
            std::cout << "Invalid homography mode (vo::planarHomographyVO::process)";
            return false;
        }
    }
    //Convert to float
    cv::Mat_<float> b;//odometry estimation of translation in camera frame
    cv::Mat_<float> A;
    b_double.convertTo(b,CV_32FC1);
    A_double.convertTo(A,CV_32FC1);

//New updateGlobalPosition does only regard translation and azimuth. NOT COMPLETE 3D ROTATION MATRIX
    //std::cout << "THIS IS WHERE EG ERROR OCCURS. HomographyVO.cpp line 98" << std::endl;
    float yaw_delta = 0;
    if(!A.empty()){
        yaw_delta = std::atan2(A(0,1),A(0,0));//tan with quadrant checking
    }
    updateGlobalPosition(success,yaw_delta,b,t,yaw);
//    updateGlobalPosition(success,A,b,t,R);

    return success;
}
/* Performs the odometry itself
 * Estimates camera movement between frames as expressed in the coordinate system of the camera of frame 1
 * Accepts point correspondances, instantaneous pitch, roll, height, and the current global pose
 * Uses complete homography for estimation
 */
bool vo::planarHomographyVO::odometryHom(std::vector<cv::Point2f>& p1_,
                std::vector<cv::Point2f>& p2_,
                float roll,float pitch,
                float prevRoll,float prevPitch,
                float dist,
                cv::Mat_<double>& b,
                cv::Mat_<double>& A){
    static float roll_prev = 0;
    static float pitch_prev = 0;

//roll_prev = roll;
//pitch_prev = pitch;
    //Check that there are enough point correspondances
    if((p1_.size()<3)|| (p2_.size()<3)){std::cout << "odometryHom::Not enough point correspondances" << std::endl;
        return false;
    }
    if(activateDerotation){
        deRotateFlowField(p1_, prevRoll, prevPitch);
        deRotateFlowField(p2_, roll, pitch);
    }
    std::vector<cv::Point2f> p1 = p1_;
    std::vector<cv::Point2f> p2 = p2_;
    cv::Mat_<float> H = cv::findHomography(p1,p2,homoGraphyMethod,ransacReprojThreshold);
    if(H.empty()){//std::cout << "No homography found" << std::endl;
        return false;}
    //Init vector for decomposition. the mats are Matx33d. i.e. each element contains a CV_64FC1
    std::vector<cv::Mat> rotations;
    std::vector<cv::Mat> translations;
    std::vector<cv::Mat> normals;
    int n = cv::decomposeHomographyMat(H,K,rotations,translations,normals);
    //printmats(rotations, translations,normals);
    int validIndex = getValidDecomposition(p1,rotations,translations,normals);
    //std::cout << "Valid index " << validIndex << std::endl;
    if(validIndex<0){std::cout <<"Validindex: "<< validIndex<<", No decomposition found" << std::endl;
        return false;
    }
    //Return the valid rotation and translation mats
    b = translations[validIndex]*dist*cos(roll)*cos(pitch); //Maybe flip signs or something? how to express this as camera movement and not scene movement?
    A = rotations[validIndex];
    //Shift roll and pitch static variables
    roll_prev = roll;
    pitch_prev = pitch;
    return true;
}
/* Performs the odometry itself
 * Estimates camera movement between frames as expressed in the coordinate system of the camera of frame 1
 * Accepts point correspondances, instantaneous pitch, roll, height, and the current global pose
 * Uses 4d affine transformation instead of complete homography. Mandatory de-rotation
 */
bool vo::planarHomographyVO::odometryAffine(std::vector<cv::Point2f>& p1_,
                std::vector<cv::Point2f>& p2_,
                float roll,float pitch,
                float prevRoll, float prevPitch,
                float dist,//height should rather be distance. not corresponding to coordinate system height
                cv::Mat_<double>& b,
                cv::Mat_<double>& A){
    static float roll_prev = 0;
    static float pitch_prev = 0;

    //Check that there are enough point correspondances
    if((p1_.size()<3)|| (p2_.size()<3)){std::cout << "odometryAffine::Not enough point correspondances" << std::endl;
        return false;
    }
    //De-rotate
    if(activateDerotation){
        deRotateFlowField(p1_, prevRoll, prevPitch);
        deRotateFlowField(p2_, roll, pitch);
    }
    //Translate flow field to camera coordinate system. ie origin is in the middle
    //cv::Point2f offset(K(0,2),K(1,2));
    std::vector<cv::Point2f> p1 = p1_;
    std::vector<cv::Point2f> p2 = p2_;
    cv::Point2f offset(K(0,2),K(1,2));
    std::vector<cv::Point2f>::iterator it1 = p1.begin();
    std::vector<cv::Point2f>::iterator it2 = p2.begin();
    while(it1 != p1.end()){
        *it1 -= offset;
        *it1 /= K(0,0);
        *it2 -= offset;
        *it2 /= K(1,1);
        it1++;
        it2++;
    }
    cv::Mat At = cv::estimateAffinePartial2D(p1,p2,opa,method_,ransacReprojThreshold_,maxIters,confidence_,refineIters_);
    if(At.empty()){return false;}
    //Calculate scale
    cv::Mat_<double> R_ = At(cv::Rect(0,0,2,2));
    double s = sqrt(cv::determinant(R_));
    R_ /= s;//Scale back by s to obtain rotation matrix
    //std::cout << "Affine scale: " << s << std::endl;
    //std::cout << "Base translation: " << b.t() << std::endl;
    //Update rotation matrix Note: only azimuthal rotation
    //Update coordinates . If  A is not initialized then initialize it. It basically always i not initialized
    if(A.empty()){
        cv::Mat_<double> A_ = cv::Mat_<double>::zeros(3,3);
        cv::Mat_<double> b_ = cv::Mat_<double>::zeros(3,1);
        R_.copyTo(A_(cv::Rect(0,0,2,2)));
        At(cv::Rect(2,0,1,2)).copyTo(b_(cv::Rect(0,0,1,2))); //Set x-y values
        //Scale seems to be not very good at estimating height. do it in some other way?
        //b_(2,0) =dist*(1-s);//dist/s-dist;//Set delta-height according to dist and scale change
        A_.copyTo(A);
        b_.copyTo(b);
    }/*else{
        R_.copyTo(A(cv::Rect(0,0,2,2)));
        At(cv::Rect(0,2,1,2)).copyTo(b(cv::Rect(0,0,1,2))); //Set x-y values
        b(2,0) /=s;//Set height according to scale change
    }*/
    //Scale x-y with height
    //Should we edit dist value??
    b(0,0) *= dist;//*cos(roll)*cos(pitch);
    b(1,0) *= dist;//*cos(roll)*cos(pitch);
    //std::cout << "Scaled translation: " << b.t() << std::endl;
    //Shift roll and pitch static variables
    roll_prev = roll;
    pitch_prev = pitch;
    return true;
}
/* Updates the global coordinate and rotation from the given b and A matrices
 *  If the first argument is true (odometry succeeded) then the global position is
 * updated according to A and b. If not, the UAV is assumed to continue along the current path
 * TODO: scale the inertia-estimation with the time since last measuremnt so that velocity is assumed constant
 */
void vo::planarHomographyVO::updateGlobalPosition(bool VOSuccess,
                            const cv::Mat_<float>& A,
                            const cv::Mat_<float>& b,
                            cv::Mat_<float>& t,
                            cv::Mat_<float>& R){
    static cv::Mat_<float> R_d = cv::Mat_<float>::eye(3,3);     //Instantaneous Rotation-difference
    static cv::Mat_<float> t_d = cv::Mat_<float>::zeros(3,1);   //instantaneous translation-difference
    static float timeStamp = 0;
    cv::Mat_<float> t_new = cv::Mat_<float>::zeros(3,1);
    if(VOSuccess){//Assign the new values to the static variables
        t_d = R.t()*T.t()*b; //Convert translation b from camera frame to UAV frame to global frame
        R_d = A;             //Assume only rotation around z and uav and camera frame is aligned in z. No conversion needed
        //t_new = t+t_d;//No inertia in translation, only rotation
        //t = t_new;
    }
    //Increment

    t_new = t+t_d;
    cv::Mat_<float> R_new = R_d*R;
    //Update pose
    t = t_new;
    R = R_new;
}
/* Updates the global coordinate and rotation from the given b matrix and delta-yaw(given as float)
 *  If the first argument is true (odometry succeeded) then the global position is
 * updated according to yaw and b. If not, the UAV is assumed to continue along the current path
 * TODO: scale the inertia-estimation with the time since last measuremnt so that velocity is assumed constant
 */
void vo::planarHomographyVO::updateGlobalPosition(bool VOSuccess,
                            float delta_yaw,
                            const cv::Mat_<float>& b,
                            cv::Mat_<float>& t,
                            float& yaw){
    static float yaw_d = 0;     //Instantaneous Rotation-difference
    static cv::Mat_<float> t_d = cv::Mat_<float>::zeros(3,1);   //instantaneous translation-difference
    static float timeStamp = 0;
    cv::Mat_<float> t_new = cv::Mat_<float>::zeros(3,1);
    if(VOSuccess){//Assign the new values to the static variables
        cv::Mat_<float> R = getZRot(-yaw);
        t_d = R.t()*T.t()*b; //Convert translation b from camera frame to UAV frame to global frame
        yaw_d = delta_yaw;             //Assume only rotation around z and uav and camera frame is aligned in z. No conversion needed
        //t_new = t+t_d;//No inertia in translation, only rotation
        //t = t_new;
    }
    //Increment

    t_new = t+t_d;
    float yaw_new = yaw + yaw_d;
    // Cast yaw to +-pi
    while(yaw_new>vo::PI){
        yaw_new-=2*vo::PI;
    }
    //Update pose
    t = t_new;
    yaw = yaw_new;
}
/* Convenience method to print out decomposition returns
*/
void vo::planarHomographyVO::printmats(std::vector<cv::Mat> rotations,
                std::vector<cv::Mat> translations,
                std::vector<cv::Mat> normals){
    for(int i=0;i<normals.size();i++){
        std::cout << "Trans. " << i << ": " << translations[i].at<double>(0,0) << ", " << translations[i].at<double>(0,1) << ", " << translations[i].at<double>(0,0) << std::endl;
    }

}
/* This method projects a set of image plane coordinates via the inverse K-matrix to
 * 3d coordinates in camera frame at z=1
 *
 */
cv::Mat_<float> vo::planarHomographyVO::imagePointsToNorm(const std::vector<cv::Point2f> &p){
    //Define ones-mat of right size
    cv::Mat_<float> points = cv::Mat_<float>::ones(3,p.size());//init all to one. z-row set directly
    //Loop through vector<point> and build a mat of their values
    //Iterate through first row
    std::vector<cv::Point2f>::const_iterator it = p.begin();
    float* ptr = points.ptr<float>(0);//x-row
    uchar counter = 0;
    while(it!=p.end()){
        ptr[counter] = it->x;
        counter++;
        it++;
    }
    //Reset pointers and iterate through second row
    it=p.begin();
    ptr = points.ptr<float>(1);//y-row
    counter = 0;
    while(it!=p.end()){
        ptr[counter] = it->y;
        counter++;
        it++;
    }
    cv::Mat_<float> points3D = K_inv*points;//Normalize to 3d coordinates wit inverted K-mat
    return points3D.t();//Return transpose
}
/*This methods checks the translation-values. If any translation is too high, then reject
 * Is used as a safety measure to reject incredible values returned from decomposehomographymat
 */
bool vo::planarHomographyVO::sanityCheck(std::vector<cv::Mat> translations,std::vector<int>& indexes){
    int indexSize = indexes.size();
    int i = 0;
    bool status = false;//init as false. only need one good to return true
    while(i<indexSize){
        cv::Mat_<double> sqrdNorm = translations[indexes[i]].t()*translations[indexes[i]]; //Calculate squared norm
        if(sqrdNorm.at<double>(0,0) > sanityLimit){
            indexes.erase(indexes.begin()+i);    //Remove solution. Do not iterate as the same index is now next point
            indexSize--;
        }else{
            status = true;
            i++;
        }
    }
    return status;
}
/*This method rejects any not-possible decompositions of homography matrix
 * Uses the reference-point-visibility constraint to reject >=2 of the presented 4 solutions
 * If there are still two solutions left, the last one is chosen as the one that is most similar
 * to the last calculated normal, stored as a static variable.
 * In: reference point in image plane together with the inverted K-matrix to calculate 3d-coord at z=1 of camera
 * Also the normals obtained from homography decomposition
 */
int vo::planarHomographyVO::getValidDecomposition(std::vector<cv::Point2f> p_ref,
                            std::vector<cv::Mat> rotations,
                            std::vector<cv::Mat> translations,
                            std::vector<cv::Mat> normals){
    //Set static normal with initial value
    static cv::Mat_<float> lastNormal = cv::Mat_<float>::zeros(3,1);
    static int init = 0;
    if(!init){
        init = 1;
        lastNormal.at<float>(2,0) = 1;}//Assume inital plane is parallell to image plane
    //Make list of all indexes that are to be checked
    std::vector<int> indexes;
    for(int i=0;i<normals.size();i++){
        indexes.push_back(i);
    }
    // perform sanity check of translation vectors
    if(!sanityCheck(translations,indexes)){std::cout << "Insane" << std::endl;
        return -1;
    }
    //Get 3D coordinates of ref points expressed in camera frame (at z=1) and in a suitable matrix
    cv::Mat_<float> m_transpose = imagePointsToNorm(p_ref);         //3d points at z=1 of reference frame
    //Convert the double-precision mats to 32bit float
    std::vector<cv::Mat_<float>> normals32F;
    for(cv::Mat normal:normals){
        cv::Mat mat32F;
        normal.convertTo(mat32F,CV_32FC1);
        normals32F.push_back(mat32F);
    }
    for(int j=0;j<p_ref.size();j++){                                  //Loop through all points
        cv::Mat_<float> point_transpose = m_transpose.row(j);       //Extract current point
        //std::cout<< "---" << std::endl;
        int indexSize = indexes.size();
        int i = 0;
        while(i<indexSize){                             //Try out all available normals. But do not iterate i here
            cv::Mat projection = point_transpose*normals32F[indexes[i]];        //Calculate projection
            //std::cout << "Size: " << normals32F.size() << std::endl;
            if(projection.at<float>(0,0)<0){                        //Compare value
                indexes.erase(indexes.begin()+i);                   //Remove solution. Do not iterate as the same index is now next point
                indexSize--;
                //std::cout << "Erased. Size: " << projection.at<float>(0,0) << std::endl;
            }else{
                i++;                                                //Iterate.
            }
        }
        j++;
    }
    if(indexes.size()==0){std::cout << "All rejected" << std::endl;
        return -1;
    }//This is wierd bahaviour,return -1 to indicate
    if(indexes.size()==1){
        lastNormal = normals32F[indexes[0]];
        return indexes[0];
    }
    if(indexes.size()==2){//If there is still two solutions
        cv::Mat_<float> proj1 = lastNormal.t()*normals32F[indexes[0]];
        cv::Mat_<float> proj2 = lastNormal.t()*normals32F[indexes[1]];
        if(proj1.at<float>(0,0)>proj2.at<float>(0,0)){
            lastNormal = normals32F[indexes[0]];
            //std::cout << "Chose number 1" << std::endl;
            return indexes[0];
        }else{
            lastNormal = normals32F[indexes[1]];
            //std::cout << "Chose number 2" << std::endl;
            return indexes[1];
        }
    }
    std::cout << "Reached too far in getValidDecomposition. check input size." << std::endl;
    return -2; //If we reach this point then something is wrong also. return negative to indicate
}
/* SEE  deRotateFlowField FOR OTHER VERSION OF THIS
 * This method de-rotates a Homography matrix using the supplies tait-bryan angles
 * For ref. see M.Wadenbaeck phd Thesis p.25
 * R(a,b,c) = R_x(a)*R_y(b)*R_z(c)
 *  H = inv(R_(psi,theta)) * H_rot * transpose(inv(R_(psi,theta)))
 * Camera coordinate system:
    Z: perpendicular to image plane -> yaw      (phi)   Innermost rotation, where floor normal and z are parallell
    Y: Right-facing                 -> pitch    ()      Middle rotation
    X: Forward facing               -> roll     ()      Last rotation
 * NOTE: It de-rotates by inverting the roll and pitch rotation matrix.
 * NOTE: I could instead directly define a rotation matrix using negative angles -Faster

 Kind of works? some scaling error
 */
cv::Mat_<float> vo::planarHomographyVO::deRotateHomography(cv::Mat_<float>& H_rot, float roll,float pitch){
    //return H_rot;
    //cv::Mat R_x = getXRot(roll);
    //cv::Mat R_y = getYRot(pitch);
    //cv::Mat_<float> R_left = R_y*R_x;

    cv::Mat_<float> R_left = deRotMat(roll,pitch); //THIS SHOULD BE CORRECT
    cv::Mat_<float> R_right = R_left.t();
    //cv::Mat_<float> H = T*R_y*R_x*H_rot*R_x.t()*R_y.t()*T_inv;
    cv::Mat_<float> H = K*R_left*H_rot*R_right*K_inv;
    return H;
}
/* This method de-rotates (the pitch of) the flow field.
 * It achieves this directly on the coordinates by
 * 1. Perspective transformation. Illustrated as a warp of the coordinates around the x-axis of image
 * 2. Shift of rotation center.
 * 3? adapt height - Not needed. Independent of height

 * NOTE: the function -DE-rotates the given angles. i.e rotates the field by -roll, then -pitch
 */
void vo::planarHomographyVO::deRotateFlowField(std::vector<cv::Point2f>& src, float roll, float pitch){
    std::vector<cv::Point2f> dst;
    cv::Mat R_x = getXRot(-roll);
    cv::Mat R_y = getYRot(-pitch);
    cv::Mat H = K*T* R_y*R_x* T_inv*K_inv;//Right to left: pixel to camera, camera to UAV, roll, pitch, UAV to camera, camera to pixel
//cv::Mat H = T*K *R_y*R_x* K_inv*T_inv;
    VOperspectiveTransform(src,dst,H);
    src.clear();
    for(cv::Point2f point:dst){
        src.push_back(point);
        //std::cout << point.x <<", " <<point.y<<";" << std::endl;
    }
}
/*Own implementation of perspective transform of vector<point2f>. Normalizes to z=1
 *  Points are considered as being column vectors and are right-multiplied with H
 */
void vo::planarHomographyVO::VOperspectiveTransform(std::vector<cv::Point2f> src,std::vector<cv::Point2f>&dst,cv::Mat_<float>H){
    dst.clear();//Make sure that dst is empty
    for(cv::Point2f point:src){
        float x = point.x*H(0,0) + point.y*H(0,1) + H(0,2);
        float y = point.x*H(1,0) + point.y*H(1,1) + H(1,2);
        float z = point.x*H(2,0) + point.y*H(2,1) + H(2,2);
        dst.push_back(cv::Point2f(x/z,y/z));//Normalized to z=1
    }
}
/* This method defines a de-rotation matrix by defining rotation with negative angles
 * Standard rotation sequence is Z-Y-X (yaw-pitch-roll)
 * Retotation is thus done by Z-Y-X-(-X)-(-Y)
 * Derotating R = R_z*R_y(pitch)*R_x(roll) can be done by R*inv(R_y(a)*R_x(b))
 * inv(R_y(pitch)*R_x(roll)) = R_x(-roll)*R_y(-pitch) and thus no invertion is needed
 * Does not significantly affect result...
 */
cv::Mat_<float> vo::planarHomographyVO::deRotMat(float roll, float pitch){
    // Roll rotation matrix
    //std::cout << "SHOULD BE NEG" << std::endl;
    float sinX = std::sin(-roll);
    float cosX = std::cos(-roll);//std::sin(-roll);
    cv::Mat_<float> R_x = cv::Mat_<float>::zeros(3,3);
    R_x(0,0) = 1;
    R_x(1,1) = cosX;
    R_x(1,2) = -sinX;
    R_x(2,1) = sinX;
    R_x(2,2) = cosX;
    //Pitch rotation matrix
    float sinY = std::sin(-pitch);
    float cosY = std::cos(-pitch);//std::sin(-pitch);
    cv::Mat_<float> R_y = cv::Mat_<float>::zeros(3,3);
    R_y(0,0) = cosY;
    R_y(0,2) = sinY;
    R_y(1,1) = 1;
    R_y(2,0) = -sinY; //Rätt?
    R_y(2,2) = cosY;
    cv::Mat_<float> R_derotate = R_x*R_y;
    //cv::Mat_<float> R_derotate = R_y*R_x;
    return R_derotate;
}
/* Functions for defining roll, pitch, and yaw rotation matrices
 * Increase speed by passing reference and edit in place?
 */
cv::Mat vo::planarHomographyVO::getXRot(float roll){
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
cv::Mat vo::planarHomographyVO::getYRot(float pitch){
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
cv::Mat vo::planarHomographyVO::getZRot(float yaw){
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
