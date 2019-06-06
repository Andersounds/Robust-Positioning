#ifndef HOMOGRAPHYVO_H
#define HOMOGRAPHYVO_H
/*
    This is first out-of-spike version of planar homography VO method

    Issues:
        - How should de-rotation be performed? Main idea is to use IMU data to calculate Rx,Ry, and
            use them to de-rotate with.
            -Option one is to apply them to the complete homography matrix
                and thus obtain a cleaner version of H including only Rz and t. But does this actually work?
            -Option two is to de-rotate the actual flow field before even calculating the homography
                This would presumably be achieved by applying a homography transformation by perspectivetransform
                maybe simply matrix multiplication. But will the angle scaling be handled then?? how to compensate?
            -Option three is to actually move the region of interest to always look down. Have not thought this through
                but could be an approach. Will the flow field be trackable over changing RoI positions?
        - In any case. the K matrix could be expressed in terms of the RoI alone. only the cx,cy should change
            Before any illustration is done, the coordinates are shifted back to full image coordinates.
*/

namespace vo{
    const int USE_HOMOGRAPHY = 1;
    const int USE_AFFINETRANSFORM = 2;

    class planarHomographyVO{
    public:
        cv::Mat_<float> K;
        cv::Mat_<float> K_inv;
        cv::Mat_<float> T;              //Transformation matrix from UAV to Camera
        cv::Mat_<float> T_inv;
        // findHomography parameters
        int homoGraphyMethod;
        double ransacReprojThreshold;
        int maxIters;
        double confidence;
        float sanityLimit;
        bool activateDerotation;
        int mode; // 1: use homography, 2: use derotation and estimateRigidTransform
        // getAffineTransform parameters
        cv::Mat opa;
        int method_;
        double ransacReprojThreshold_;
        size_t maxIters_;
        double confidence_;
        size_t 	refineIters_;


        /*Constructor. Define the camera intrinsic parameters and some other settings
         * Arguments:
         * 1: Camera K matrix
         * 2: Transformation (pure rotation) from UAV frame to camera frame
        */
        planarHomographyVO(cv::Mat_<float>,cv::Mat_<float>,int);
        planarHomographyVO(cv::Mat_<float>,cv::Mat_<float>,int,cv::Rect2f);//This constructor edits the given K-mat to fit roi
        /*
        Default settings function
        */
        void setDefaultSettings(void);
        /*Function called to process point correspondances and estimate global movement
        -------SHOULD HAVE CONST POINT INPUT
         */
        bool process(std::vector<cv::Point2f>&,
                        std::vector<cv::Point2f>&,
                            float,float,float,
                            cv::Mat_<float>&,
                            float&);
        /* Convenience method to print out decomposition returns
        */
        void printmats(std::vector<cv::Mat>,
                        std::vector<cv::Mat>,
                        std::vector<cv::Mat>);
        /*
         *This is here to de rotate flow field
         */
        void deRotateFlowField(std::vector<cv::Point2f>&, float,float);
    private:
        /* Performs the odometry itself
         * Accepts point correspondances, instantaneous pitch, roll, height, and the current global pose
         * Performs odometry from a complete homography estimation. Optional derotation
         */
        bool odometryHom(std::vector<cv::Point2f>&,
                        std::vector<cv::Point2f>&,
                        float,float,float,
                        cv::Mat_<double>&,
                        cv::Mat_<double>&);
        /* Performs the odometry itself
         * Accepts point correspondances, instantaneous pitch, roll, height, and the current global pose
         * Performs odometry by calculationg affine 4-dim transform between derotated point-correspondances
         * Uses roll/pitch do de-rotate field
         * Solves for x,y,azimuth,scale
         */
        bool odometryAffine(std::vector<cv::Point2f>&,
                        std::vector<cv::Point2f>&,
                        float,float,float,
                        cv::Mat_<double>&,
                        cv::Mat_<double>&);
        /* Updates the global coordinate and rotation from the given b and A matrices
         *  If the first argument is true (odometry succeeded) then the global position is
         * updated according to A and b. If not, the UAV is assumed to continue along the current path
         * TODO: scale the inertia-estimation with the time since last measuremnt so that velocity is assumed constant
         */
        void updateGlobalPosition(bool,
                                    const cv::Mat_<float>&,
                                    const cv::Mat_<float>&,
                                    cv::Mat_<float>&,
                                    cv::Mat_<float>&);
        /* As above but does only consider yaw rotation. Thus arguments are given as a Mat contining coordinate
         * and a float containing yaw (azimuth) angle
         *
         */
        void updateGlobalPosition(bool,
                                    float,
                                    const cv::Mat_<float>&,
                                    cv::Mat_<float>&,
                                    float&);
        /* This method projects a set of image plane coordinates via the inverse K-matrix to
         * 3d coordinates in camera frame at z=1
         *
         */
        cv::Mat_<float> imagePointsToNorm(const std::vector<cv::Point2f>&);
        /*This methods checks the translation-values. If any translation is too high, then reject
         * Is used as a safety measure to reject incredible values returned from decomposehomographymat
         */
        bool sanityCheck(std::vector<cv::Mat>,std::vector<int>&);
        /*This method rejects any not-possible decompositions of homography matrix
         * Uses the reference-point-visibility constraint to reject >=2 of the presented 4 solutions
         * If there are still two solutions left, the last one is chosen as the one that is most similar
         * to the last calculated normal, stored as a static variable.
         * In: reference point in image plane together with the inverted K-matrix to calculate 3d-coord at z=1 of camera
         * Also the normals obtained from homography decomposition
         */
        int getValidDecomposition(std::vector<cv::Point2f>,
                                    std::vector<cv::Mat>,
                                    std::vector<cv::Mat>,
                                    std::vector<cv::Mat>);
        /* This method de-rotates a Homography matrix using the supplies tait-bryan angles
         * For ref. see M.Wadenbaeck phd Thesis p.25
         * R(a,b,c) = R_x(a)*R_y(b)*R_z(c)
         *  H = inv(R_(psi,theta)) * H_rot * transpose(inv(R_(psi,theta)))
         * Camera coordinate system:
            Z: perpendicular to image plane -> yaw      (phi)   Innermost rotation, where floor normal and z are parallell
            Y: Right-facing                 -> pitch    ()      Middle rotation
            X: Forward facing               -> roll     ()      Last rotation
         * NOTE: It de-rotates by inverting the roll and pitch rotation matrix.
         * NOTE: I could instead directly define a rotation matrix using negative angles -Faster
         */
        cv::Mat_<float> deRotateHomography(cv::Mat_<float>&, float,float);
        cv::Mat getXRot(float);
        cv::Mat getYRot(float);
        /* This method defines a de-rotation matrix by defining rotation with negative angles
         * Standard rotation sequence is Z-Y-X (yaw-pitch-roll)
         * Retotation is thus done by Z-Y-X-(-X)-(-Y)
         * Derotating R = R_z*R_y(pitch)*R_x(roll) can be done by R*inv(R_y(a)*R_x(b))
         * inv(R_y(pitch)*R_x(roll)) = R_x(-roll)*R_y(-pitch) and thus no invertion is needed
         * Does not significantly affect result...
         */
        cv::Mat_<float> deRotMat(float, float);
        void VOperspectiveTransform(std::vector<cv::Point2f>,std::vector<cv::Point2f>&,cv::Mat_<float>);
    };
}
#endif
