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
        cv::Scalar color;

        //int active;
        //The class also needs some kind of certainty measure
    };

namespace to{

    class trackedObjectList{
    public:
        trackedObjectList(int,int);                 //Constructor that sets up tracking vectors and initial values
        void add(trackedObject*);
        void remove(int);
        void clear(void);
        int getReplaceIndex(void);
        void replace(int,trackedObject*);
        bool isActive(int);
        std::vector<trackedObject*> list;           //List of not yet defined length containing pointers to all objects (on heap)

        int maxNmbr;
        int no_of_tracked;
        int no_of_parallel;
        std::vector<cv::Scalar> colors;             // Colors used to draw anchors
        std::vector<int> activeIDs;                 // The anchor(s) that are currently in FoV
        std::vector<int> activeStates;              // The tracking state of each tracked active anchor
        std::vector<cv::Rect_<float>> activeRects;  // The rect of each tracked active anchor
    };

}
#endif
