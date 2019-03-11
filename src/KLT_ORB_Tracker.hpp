

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
  int nfeatures = 300;
  float scalefactor = 1.2;
  int nlevels = 8;
  int nThreshold = 20;//31
  int firstLevel = 0;
  int WTA_K = 2;
  int scoreType = cv::ORB::HARRIS_SCORE;
  int patchSize = 31;
  int fastThreshold = 20;
  /*Settings for Matcher object*/
  int normType = cv::NORM_HAMMING;
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
    std::vector<cv::Rect> findClusters(cv::Mat, std::vector<cv::KeyPoint>, int, int, int,std::vector<std::vector<cv::KeyPoint>>&,std::vector<cv::Mat>&);
    int calcORBDescriptors(cv::Mat, std::vector<cv::KeyPoint>&, cv::Mat&);
    int calcCenterPoint(std::vector<cv::KeyPoint>&, cv::KeyPoint&);

    int getQueryFeatures(cv::Mat, cv::Rect,std::vector<cv::KeyPoint>&, cv::Mat&);
    int trackOpticalFlow(cv::Mat, cv::Mat, std::vector<cv::Point2f>&, cv::Rect&);
    int featureMatching(cv::Mat, cv::Mat, std::vector<cv::DMatch>&);
    int trackMatches(std::vector<cv::DMatch>, std::vector<cv::KeyPoint>&, cv::Rect&);
    int drawPoints(cv::Mat, std::vector<cv::KeyPoint>, cv::Mat&, cv::Scalar);
    int drawPoints(cv::Mat, std::vector<cv::Point2f>, cv::Mat&, cv::Scalar);

    int drawTheMatches(cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat&, std::vector<cv::KeyPoint>&, std::vector<cv::DMatch>&, cv::Mat&);

    int repeat(void);
};
