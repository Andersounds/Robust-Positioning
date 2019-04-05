#ifndef KLT_ORB_TRACKER_H
#define KLT_ORB_TRACKER_H

/*Settings struct*/
struct KLTsettingsStruct{
  //cv::Size winSize(51,51);
  int windowSize = 51;
  int maxLevel = 2;
  cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.01);
  //cv::TermCriteria termcrit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 20, 0.01);
  //cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,20,0.03);
  int flags = 0;//No flag. Use normal L2 norm error
};
struct ORBsettingsStruct{
  /*Settings for ORB object*/
  int nfeatures = 800;
  float scalefactor = 1.2;//1.2;
  int nlevels = 8;
  int nThreshold = 31;
  int firstLevel = 0;//2;//0;//When set this to 2 I sometimes encounter an assertion error crash in ORB detect
  int WTA_K = 2;
  int scoreType = cv::ORB::HARRIS_SCORE;
  int patchSize = 31;
  int fastThreshold = 20;
  /*Settings for Matcher object*/
  int normType = cv::NORM_HAMMING;//cv::NORM_HAMMING;
  bool crosscheck = true;
};

/*Tracker class*/
class KLT_ORB_Tracker{
  public:
    /*Attributes*/
    KLTsettingsStruct KLTsettings;
    ORBsettingsStruct ORBsettings;
    cv::Ptr<cv::ORB> orbObject;             //The ORB Tracker object
    cv::Ptr<cv::BFMatcher> matcherObject;   //The ORB matcher object
    std::vector<cv::DMatch> matches;        //Vector of matches

    /*Methods*/
    KLT_ORB_Tracker(void);//Constructor. Probably give a settings struct
    int init(void);
    int getFeatures(cv::Mat, std::vector<cv::KeyPoint>&);// ORB
    std::vector<cv::KeyPoint> getFeatures(cv::Mat, cv::Mat);
    std::vector<cv::KeyPoint> getFeatures(cv::Mat, cv::Rect_<float>);
    //std::vector<cv::Rect_<float>> findClusters(cv::Mat, std::vector<cv::KeyPoint>, int, int, int);//Without mask
    std::vector<cv::Rect_<float>> findClusters(cv::Mat, cv::Mat, std::vector<cv::KeyPoint>, int, int, int);//With mask
    int calcORBDescriptors(cv::Mat, std::vector<cv::KeyPoint>&, cv::Mat&);
    int calcCenterPoint(std::vector<cv::KeyPoint>&, cv::KeyPoint&);

    int getQueryFeatures(cv::Mat, cv::Rect,std::vector<cv::KeyPoint>&, cv::Mat&);
    int trackOpticalFlow(cv::Mat, cv::Mat, std::vector<cv::Point2f>&, cv::Rect_<float>&);
    int featureMatching(cv::Mat, cv::Mat, std::vector<cv::DMatch>&);
    int trackMatches(std::vector<cv::DMatch>,std::vector<cv::DMatch>&, std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>, std::vector<cv::Point2f>& newObjectPoints,cv::Rect& roi,std::vector<cv::Point2f>&,std::vector<cv::Point2f>&,std::vector<cv::Point2f>&, cv::Mat&);
    int drawPoints(cv::Mat, std::vector<cv::KeyPoint>, cv::Mat&, cv::Scalar);
    int drawPoints(cv::Mat, std::vector<cv::Point2f>, cv::Mat&, cv::Scalar);

    int drawTheMatches(cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat&, std::vector<cv::KeyPoint>&, std::vector<cv::DMatch>&, cv::Mat&);
    int getMask(cv::Rect, cv::Mat);
    void blackOutMask(cv::Mat&,cv::Rect_<float>,float minDistance);
    int repeat(void);

    int setNfeatures(int);
};
#endif
