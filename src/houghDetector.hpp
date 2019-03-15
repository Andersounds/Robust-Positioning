/*


*/
#ifndef HOUGHDETECTOR_H
#define HOUGHDETECTOR_H

namespace hd{// hd- Hough Detector image matching
/*Struct to use to access voting infromation from detect function*/
//struct votingData{
//        int numberOfVotes;
//        int winningVotesScale;
//        int winningVotesAngle;
//};
/*Sort out matches and returns good ones*/
int getGoodMatches(std::vector<cv::DMatch> matches,std::vector<cv::DMatch>& goodmatches);
/*Recursive log2 bin finder*/
int getBucketIndex(std::vector<float> ranges, float first_element, float last_element, float value);
/*Recursive log2 bin finder for the circular histogram. User must assure that value is within 0-360*/
int getAngleBucketIndex(std::vector<float> ranges, float first_element, float last_element, float value);
/*Extract the best matches from all matches, using a vector of Points that represent pairs*/
std::vector<cv::DMatch> getBestMatches(std::vector<cv::DMatch>matches, std::vector<cv::Point> xWinningPairs, std::vector<cv::Point> yWinningPairs);
/*Create the ranges (delimited by lower bounds) of a scaled-bin-width-histogram*/
std::vector<float> getScaledHistogramRanges(float scaleFactor, int no_of_buckets);
/*Divides the circle up into the given number of buckets*/
std::vector<float> getAngleHistogramRanges(int number_of_buckets);
/* Returns a list of index pairs to be used with the match-vector*/
std::vector<cv::Point> getIndexPairs(std::vector<cv::DMatch> matches, bool exhaustive);
/*Returns a random list of index pairs*/
std::vector<cv::Point> getIndexPairs(std::vector<cv::DMatch> matches, int samples);
/*Do the detection*/
std::vector<float> detect(std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> objectKeypoints, std::vector<cv::KeyPoint> sceneKeyPoints, std::vector<cv::Point>&scaleWinningPairs, std::vector<cv::Point>&angleWinningPairs);
}
#endif
