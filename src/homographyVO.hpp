#ifndef HOMOGRAPHYVO_H
#define HOMOGRAPHYVO_H
/*
    This is first out-of-spike version of planar homography VO method
*/

namespace vo{

    class planarHomographyVO{
    public:
        cv::Mat_<float> K;
        cv::Mat_<float> K_inv;
        cv::Mat_<float> T;
        // findHomography parameters
        int homoGraphyMethod;
        double ransacReprojThreshold;
        int maxIters;
        double confidence;
        float sanityLimit;

        /*Constructor. Define the camera intrinsic parameters and some other settings
         * Arguments:
         * 1: Camera K matrix
         * 2: Transformation (pure rotation) from UAV frame to camera frame
        */
        planarHomographyVO(cv::Mat_<float>,cv::Mat_<float>);
        /*Function called to process point correspondances and estimate global movement
        -------SHOULD HAVE CONST POINT INPUT
         */
        bool process(std::vector<cv::Point2f>&,
                        std::vector<cv::Point2f>&,
                            float,float,float,
                            cv::Mat_<float>&,
                            cv::Mat_<float>&);
        /* Convenience method to print out decomposition returns
        */
        void printmats(std::vector<cv::Mat>,
                        std::vector<cv::Mat>,
                        std::vector<cv::Mat>);
    private:
        /* Performs the odometry itself
         * Accepts point correspondances, instantaneous pitch, roll, height, and the current global pose
         */
        bool odometry(std::vector<cv::Point2f>&,
                        std::vector<cv::Point2f>&,
                        float,float,float,
                        cv::Mat_<double>&,
                        cv::Mat_<double>&);

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
        /* This method defines a de-rotation matrix by defining rotation with negative angles
         * Standard rotation sequence is Z-Y-X (yaw-pitch-roll)
         * Retotation is thus done by Z-Y-X-(-X)-(-Y)
         * Derotating R = R_z*R_y(pitch)*R_x(roll) can be done by R*inv(R_y(a)*R_x(b))
         * inv(R_y(pitch)*R_x(roll)) = R_x(-roll)*R_y(-pitch) and thus no invertion is needed
         * Does not significantly affect result...
         */
        cv::Mat_<float> deRotMat(float, float);
    };
}
#endif