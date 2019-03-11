/*Settings struct for goodfeatureTracker with inital values*/
struct goodSettings
{
  int maxNoOfCorners = 10;//Initialize as 10 corners This is max I guess
  double qualityLevel= 0.5;
  double minDistance = 20;
  bool useHarris = false;
  int blockSize = 3;
  double k = 0.04;
  cv::Mat mask;
};
//struct trackSettings
//{

//};
class goodfeatureTracker
{
  public:
    goodfeatureTracker(int);/*Constructor*/
    //~featureTracker(void); /*Destructor*/
    int getFeaturesToTrack(cv::Mat&);/*Use the chosen feature detection method to get features to track*/
    int trackFeatures(cv::Mat&); /*Track the features from previous frame in given*/
    int drawCorners(cv::Mat&);/*Draw the found corners. Maybe overload this later so that both points and keypoints can be drawn*/
    int drawROIS(cv::Mat&);

    std::vector<cv::Point2i> corners; /*Corners found by the feature detector*/
    std::vector<cv::Rect2i> roi; /*Regions of interest to track*/
    cv::Ptr<cv::MultiTracker> multiTracker; /*Pointer to multitracker*/

    int update(cv::Mat&);
    int initTracker(cv::Mat&);
    goodSettings settings;
};
