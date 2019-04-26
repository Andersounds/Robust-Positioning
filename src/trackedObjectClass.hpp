#ifndef TRACKEDOBJECTCLASS_H
#define TRACKEDOBJECTCLASS_H



    class trackedObject{
    public:
        trackedObject(std::vector<cv::KeyPoint>, cv::Mat, cv::Rect_<float>,int,cv::Mat);
        trackedObject(std::vector<cv::KeyPoint>, cv::Mat, cv::Rect_<float>,int);
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
        cv::Rect_<float> trackedRect;           //The constantly changing rect
        std::vector<cv::Point2f> trackedPoints; //The tracked points
        cv::Mat originalFrame;//Used to illustrate the matching
        //int active;
        //The class also needs some kind of certainty measure
    };

namespace to{

    class trackedObjectList{
    public:
        trackedObjectList(int,int);                 //Constructor that sets up tracking vectors and initial values
        int add(trackedObject*);
        void remove(int);
        void clear(void);
        int getReplaceIndex(void);
        void replace(int,trackedObject*);
        bool isActive(int);
        void setState(int,int);
        int getState(int);
        void setDelay(int,int);
        void decrementDelay(int);
        int getDelay(int);
        std::vector<int> getZeroStateIndexes(void);
        void drawAnchor(cv::Mat&,int);

        void assignAnchor(int,int);// Assigns the anchor with the second argument as ID to the active anchor with index of the first argument
        void updateAnchor(int,cv::Rect_<float>,std::vector<cv::Point2f>);//1. current active anchor index, new value of tracked rect, new vector of points
        std::vector<trackedObject*> list;           //List of not yet defined length containing pointers to all objects (on heap)

        int maxNmbr;
        int no_of_tracked;
        int no_of_parallel;
        std::vector<cv::Scalar> colors;             // Colors used to draw anchors
        std::vector<int> activeIDs;                 // The anchor(s) that are currently in FoV
        std::vector<int> activeStates;              // The tracking state of each tracked active anchor
        std::vector<int> activeDelays;
    };



}
#endif
