#ifndef TRACKEDOBJECTCLASS_H
#define TRACKEDOBJECTCLASS_H



namespace to{
    class trackedObject{
        trackedObject(std::vector<cv::KeyPoint>, cv::Mat, cv::Rect_<float>);
        std::vector<cv::Point2f> getTrackablePoints(float,
                                                    float,
                                                    std::vector<cv::DMatch>,
                                                    std::vector<cv::KeyPoint>,
                                                    cv::Rect_<float>&);
        void improve3D(cv::Point3f, float, float);//Position, azimuthal, polar
        std::vector<cv::KeyPoint> keyPoints;
        cv::Mat descriptors;
        cv::Rect_<float> rectangle;
        cv::Point2f rectCorner;// Is used to calculate all KP offsets
        std::vector<cv::Point2f> offsets;
        cv::Point3f coordinate;
        //The class also needs some kind of certainty measure
    }
}
#endif
