#ifndef TRACKEDOBJECTCLASS_H
#define TRACKEDOBJECTCLASS_H



    class trackedObject{
    public:
        trackedObject(std::vector<cv::KeyPoint>, cv::Mat, cv::Rect_<float>);
        std::vector<cv::Point2f> getTrackablePoints(float,
                                                    float,
                                                    std::vector<cv::DMatch>,
                                                    std::vector<cv::KeyPoint>,
                                                    cv::Rect_<float>&);
        std::vector<cv::Point2f> getTrackablePoints(void);
        void improve3D(cv::Point3f, float, float);//Position, azimuthal, polar
        std::vector<cv::KeyPoint> originalKeyPoints;
        cv::Mat originalDescriptors;
        cv::Rect_<float> originalRectangle;
        cv::Point2f rectCorner;// Is used to calculate all KP offsets
        std::vector<cv::Point2f> offsets;
        cv::Point3f coordinate;
        int ID;
        float matches;
        float matchAttempts;
        float rating;
        float matchRate;
        char r;
        char g;
        char b;
        //int active;
        //The class also needs some kind of certainty measure
    };

namespace to{

    class trackedObjectList{
    public:
        trackedObjectList(int);
        void add(trackedObject*);
        void remove(int);
        void clear(void);
        int getReplaceIndex(void);
        void replace(int,trackedObject*);
        std::vector<trackedObject*> list;   //List of not yet defined length containing pointers to all objects
        std::vector<bool> activeIDs;
        int maxNmbr;
        int no_of_tracked;
    };

}
#endif
