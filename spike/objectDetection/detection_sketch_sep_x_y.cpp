#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath> //For sleep
#include <random>//For index sampling
#include "../../src/videoStream.cpp"
#include "../../src/argumentParser.cpp"
#include "../../src/KLT_ORB_Tracker.cpp"

//using namespace cv;
//using namespace std;


int getGoodMatches(std::vector<cv::DMatch> matches,std::vector<cv::DMatch>& goodmatches,std::vector<cv::KeyPoint> objectKeyPoints, std::vector<cv::KeyPoint> sceneKeyPoints){
  float dist_max = 0;
  float dist_min = 100;
  for(int i=0;i<matches.size();i++){
    float dist = matches[i].distance;
    if( dist < dist_min ) dist_min = dist;
    if( dist > dist_max ) dist_max = dist;
  }
  //std::cout << "min dist: " << dist_min << ", max dist: " << dist_max << std::endl;
  //Keep only good matches whose distance is less than 3*dist_min
  std::vector< cv::DMatch > good_matches;
  for(int i = 0; i < matches.size(); i++){
    if(matches[i].distance < 3*dist_min){
      good_matches.push_back(matches[i]);
    }
  }
  goodmatches = good_matches;
  std::cout << "Featurematching. Good matches: " << good_matches.size() << std::endl;
  //Take the points of the good matches
  std::vector <cv::Point2f> scene, object;
  for(int i=0; i<good_matches.size(); i++){
    object.push_back( objectKeyPoints[ good_matches[i].trainIdx ].pt );//train or query? noone knows.
    scene.push_back( sceneKeyPoints[ good_matches[i].queryIdx ].pt );
  }

return 1;
}
/*Recursive log2 bin finder*/
int getBucketIndex(std::vector<float> ranges, float first_element, float last_element, float value){
  int tryBucket = (int) floor((last_element+first_element)/2);
  if(value >= ranges[last_element] || value < ranges[first_element]){return -1;}//Out of bound
  else if((last_element-first_element)==1){return first_element;}//Found it!
  else if(value <  ranges[tryBucket]){return getBucketIndex(ranges, first_element, tryBucket, value);}
  else if(value >= ranges[tryBucket]){return getBucketIndex(ranges, tryBucket, last_element,  value);}
  else {return -1;}
}
/*Recursive log2 bin finder for the circular histogram. User must assure that value is within 0-360*/
int getAngleBucketIndex(std::vector<float> ranges, float first_element, float last_element, float value){
  int tryBucket = (int) floor((last_element+first_element)/2);
  if(value >= ranges[last_element] || value < ranges[first_element]){return ranges.size()-1;}//Found it! the last bin, which covers the overflow at angle 0
  else if((last_element-first_element)==1){return first_element;}//Found it!
  else if(value <  ranges[tryBucket]){return getAngleBucketIndex(ranges, first_element, tryBucket, value);}
  else if(value >= ranges[tryBucket]){return getAngleBucketIndex(ranges, tryBucket, last_element,  value);}
  else {return -1;}
}
/*Extract the best matches from all matches, using a vector of Points that represent pairs*/
std::vector<cv::DMatch> getBestMatches(std::vector<cv::DMatch>matches, std::vector<cv::Point> xWinningPairs, std::vector<cv::Point> yWinningPairs){
    std::vector<cv::DMatch> xBest_matches, yBest_matches, best_matches;
    std::vector<int> x_best_indexes, y_best_indexes, best_indexes;
    for(cv::Point element:xWinningPairs){
        if(std::find(x_best_indexes.begin(), x_best_indexes.end(), element.x) == x_best_indexes.end()){
            x_best_indexes.push_back(element.x);
            xBest_matches.push_back(matches[element.x]);
        }
        if(std::find(x_best_indexes.begin(), x_best_indexes.end(), element.y) == x_best_indexes.end()){
            x_best_indexes.push_back(element.y);
            xBest_matches.push_back(matches[element.y]);
        }
        //Many matches are here many times I guess. because [1,2], [1,3] both vote for same
    }
    for(cv::Point element:yWinningPairs){
        if(std::find(y_best_indexes.begin(), y_best_indexes.end(), element.x) == y_best_indexes.end()){
            y_best_indexes.push_back(element.x);
            yBest_matches.push_back(matches[element.x]);
        }
        if(std::find(y_best_indexes.begin(), y_best_indexes.end(), element.y) == y_best_indexes.end()){
            y_best_indexes.push_back(element.y);
            yBest_matches.push_back(matches[element.y]);
        }
    }
    if(x_best_indexes.size()>=y_best_indexes.size()){
        for(int element:x_best_indexes){
            if(std::find(y_best_indexes.begin(), y_best_indexes.end(), element) != y_best_indexes.end()){
                best_indexes.push_back(element);
                best_matches.push_back(matches[element]);
            }
        }
    }
    //Redo this one. need to loop through everything in order to count points anyway
    return xBest_matches;
}
/*Create the ranges (delimited by lower bounds) of a scaled-bin-width-histogram*/
std::vector<float> getScaledHistogramRanges(float scaleFactor, int no_of_buckets){
    int lowestBucket = (int)floor(no_of_buckets/2)-no_of_buckets;
    std::vector<float> ranges;
    for(int i=0;i<no_of_buckets;i++){
      ranges.push_back(pow(scaleFactor,lowestBucket+i));
    }
 return ranges;
}
/*Divides the circle up into the given number of buckets*/
std::vector<float> getAngleHistogramRanges(int number_of_buckets){
    //degrees
    std::vector<float> angleRanges;
    float bucket_width = 360.0/(float)number_of_buckets;
    for(int i=0;i<number_of_buckets;i++){
        angleRanges.push_back(i*bucket_width+bucket_width/2);//Offset it so that the last bucket is centered around 0
    }
    return angleRanges;
}
/* Returns a list of index pairs to be used with the match-vector*/
std::vector<cv::Point> getIndexPairs(std::vector<cv::DMatch> matches, bool exhaustive){
  if(exhaustive){
    // Create a vector of all possible point-pairs. This can be static later. with a max nmbr of features or something. Or randomly chosen
    int no_of_pairs = (int) floor(matches.size()*(matches.size()-1)/2);
    std::vector<cv::Point> pairs;
    std::cout << "Number of pairs: " << no_of_pairs << std::endl;
    for(int i = 0; i<matches.size();i++){ //Denna kan vara static? Så har man ett maxantal features
      for(int j = i+1; j<matches.size();j++){
        pairs.push_back(cv::Point(i,j));
      }
    }
    return pairs;
  }
  else{std::vector<cv::Point> pairs;return pairs;}
}
/*Returns a random list of index pairs*/
std::vector<cv::Point> getIndexPairs(std::vector<cv::DMatch> matches, int samples){
  std::vector<cv::Point> pairs;
  static std::random_device rd;     // only used once to initialise (seed) engine
  static std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(0,matches.size()-1);
  int counter = 0;
  while(pairs.size()<=samples){
    int index1 = uni(rng);
    int index2 = uni(rng);
    if(index1!=index2){// Dragning med återläggning. Vore bättre utan återläggning
      pairs.push_back(cv::Point(index1,index2));
    }
    counter++;
    if(counter>2000){break;}//Maximum 2000 tries. Otherwise give up
  }
  return pairs;
}
/*Do the detection*/
int detect(cv::Mat object, cv::Mat frame,std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> objectKeyPoints, std::vector<cv::KeyPoint> sceneKeyPoints, std::vector<cv::Point>&xWinningPairs, std::vector<cv::Point>&yWinningPairs){
    //std::vector<cv::Point> pairs = getIndexPairs(matches, true);
    std::vector<cv::Point> pairs = getIndexPairs(matches, 100);
    int no_of_pairs = pairs.size();
    // Calculate alpha x and alpha y
    std::vector<float> alpha, beta, theta, gamma;
    std::vector<cv::Point> votingPairs;
    float d_x_i, s_x_i, d_y_i, s_y_i, d_angle_i, s_angle_i, alpha_i, beta_i, theta_i, gamma_i;
    /*Create the variable-bin-width-histogram and fill it with values*/
    float scaleFactor = 1.4;
    int number_of_buckets = 8;
    int number_of_angle_buckets = 60;
    std::vector<float> scaleRanges = getScaledHistogramRanges(scaleFactor, number_of_buckets);
    std::vector<float> angleRanges = getAngleHistogramRanges(number_of_angle_buckets);
    std::vector<int> xHistogram(number_of_buckets,0);// All zero at beginning
    std::vector<int> yHistogram(number_of_buckets,0);
    std::vector<int> scaleHistogram(number_of_buckets,0);
    std::vector<int> thetaHistogram(number_of_angle_buckets,0);
    std::vector<std::vector<cv::Point>> xHistPairs(number_of_buckets);
    std::vector<std::vector<cv::Point>> yHistPairs(number_of_buckets);
    std::vector<std::vector<cv::Point>> scaleHistPairs(number_of_buckets);
    //Let each match pair do their voting
    for(int i=0;i<pairs.size();i++){//Obs train or query?? who knows
        /*Get the four indexes that are needed for each alpha, beta, theta (object keypoin1,2, scene keypoint 1,2)*/
        int matchID1 = pairs[i].x;int matchID2 = pairs[i].y;
        int object_kpindex1 = matches[matchID1].queryIdx;int object_kpindex2 = matches[matchID2].queryIdx;
        int scene_kpindex1 = matches[matchID1].trainIdx;int scene_kpindex2 = matches[matchID2].trainIdx;
        /*The four keypoints that are collected for the current match pair i*/
        cv::KeyPoint oKP1 = objectKeyPoints[object_kpindex1];
        cv::KeyPoint oKP2 = objectKeyPoints[object_kpindex2];
        cv::KeyPoint sKP1 = sceneKeyPoints[scene_kpindex1];
        cv::KeyPoint sKP2 = sceneKeyPoints[scene_kpindex2];
        /*Perform the calculations of alpha, beta, theta for index i*/
        cv::Point oD_i = oKP2.pt - oKP1.pt;
        cv::Point sD_i = sKP2.pt - sKP1.pt;
        d_x_i = oD_i.x;
        s_x_i = sD_i.x;
        d_y_i = oD_i.y;
        s_y_i = sD_i.y;
        d_angle_i = oKP2.angle - oKP1.angle;
        s_angle_i = sKP2.angle - sKP1.angle;
        /*Calculate theta*/
        theta_i = s_angle_i - d_angle_i; //Direction. note substraction
        theta.push_back(theta_i);
        int theta_int = ((int) floor(theta_i) +360) % 360;//Convert theta to 0-360 instead
        int angleIndex = getAngleBucketIndex(angleRanges, 0, number_of_angle_buckets-1, (float)theta_int);
        thetaHistogram[angleIndex]+=1;
        float o_dist_i = cv::norm(oD_i);
        float s_dist_i = cv::norm(sD_i);

        /*If pairs are really close the estimation will be bad and division might cause problems*/
        if(std::abs(d_x_i) > 1 && std::abs(s_x_i) > 1 && std::abs(d_y_i) > 1 && std::abs(s_y_i) > 1 && o_dist_i>1 && s_dist_i > 1){
            alpha_i = s_x_i/d_x_i;
            alpha.push_back(alpha_i);
            beta_i = s_y_i/d_y_i;
            beta.push_back(beta_i);
            gamma_i = s_dist_i/o_dist_i;
            votingPairs.push_back(pairs[i]);// Save all pairs that do voting
            //Insert in histogram directly
            int index = getBucketIndex(scaleRanges,0,number_of_buckets-1, alpha_i);
            if(index != -1){
                xHistogram[index] += 1;//Increment the histogram by one vote in the right bin
                xHistPairs[index].push_back(pairs[i]);//Save the match pair that are responsible for the vote
            }
            index = getBucketIndex(scaleRanges,0,number_of_buckets-1, beta_i);
            if(index != -1){
                yHistogram[index] += 1;
                yHistPairs[index].push_back(pairs[i]);
            }
            index = getBucketIndex(scaleRanges,0,number_of_buckets-1, gamma_i);
            if(index != -1){
                scaleHistogram[index] += 1;
                scaleHistPairs[index].push_back(pairs[i]);
            }
        }
    }




    int xMaxIndex = std::max_element(xHistogram.begin(),xHistogram.end()) - xHistogram.begin();
    int yMaxIndex = std::max_element(yHistogram.begin(),yHistogram.end()) - yHistogram.begin();
    int gammaMaxIndex = std::max_element(scaleHistogram.begin(),scaleHistogram.end()) - scaleHistogram.begin();
    int thetaMaxIndex = std::max_element(thetaHistogram.begin(),thetaHistogram.end()) - thetaHistogram.begin();
    //std::cout << "Most popular scale guess: X: " << scaleRanges[xMaxIndex] << ", Y: " << scaleRanges[yMaxIndex] << std::endl;
    std::cout << "Most popular angle guess: " << angleRanges[thetaMaxIndex] <<std::endl;
    std::cout << "Most popular scale guess: " << scaleRanges[gammaMaxIndex] <<std::endl;
    float xPercentage = (float)xHistogram[xMaxIndex]/(float)no_of_pairs;
    float yPercentage = (float)yHistogram[yMaxIndex]/(float)no_of_pairs;
    float gammaPercentage = (float)scaleHistogram[gammaMaxIndex]/(float)no_of_pairs;
    float thetaPercentage = (float)thetaHistogram[thetaMaxIndex]/(float)no_of_pairs;
    //std::cout << "Percentage voted: X: " << xPercentage << ", Y: " << yPercentage<< std::endl;
    //std::cout << "Number of voters: " << xHistogram[xMaxIndex] << std::endl;
    std::cout << "Percentage voted orientation: " << thetaPercentage << std::endl;
    std::cout << "Percentage voted scale:       " << gammaPercentage << std::endl;
    xWinningPairs = xHistPairs[xMaxIndex];//These are the matches that are responsible for the winning votes in x
    yWinningPairs = yHistPairs[yMaxIndex];

    return 1;
}

int main(int argc, char** argv){
  // check if inputs are correctly given
  streamArguments arguments;
  if(!argumentParser(argc,argv,arguments)){return 0;}
  //String path = "/Users/Fredrik/Datasets/picsForMatching/objectDetection/images/scenes/scene_%04d.png";
  cv::String scenestr = "scenes/scene_%04d.png";
  cv::String objstr = "objects/obj_%04d.png";
  cv::String Tempscenestr = "scenes/scene_0052.png";
  cv::String Tempobjstr = "objects/obj_0052.png";
  int noOfClusters = 1;
  int clusterSize = 40;
  int minDistance = 100;




  KLT_ORB_Tracker sceneTracker, objectTracker;
  objectTracker.setNfeatures(100);
  sceneTracker.setNfeatures(300);
  objectTracker.init();
  sceneTracker.init();


  cv::Ptr<cv::BFMatcher> matcherObject = cv::BFMatcher::create(cv::NORM_HAMMING,true);
  std::vector<cv::DMatch> matches;
  std::vector< cv::DMatch > good_matches;
  cv::Mat frame;
  cv::Mat object;

  cv::VideoCapture cap;
  cv::VideoCapture cop;
  cap.open(arguments.datasetPath+scenestr);
  cop.open(arguments.datasetPath+objstr);
  //cap.read(frame);//Offset with one image
  int counter = 0;
  while(1){
    std::vector< cv::KeyPoint > sceneKeypoints, objectKeypoints;
    std::vector<cv::Mat> returnDescriptors;
    cv::Mat sceneDescriptors;
    cv::Mat objectDescriptors;


    // Get new scene and object pair
    cap.read(frame);
    cop.read(object);
    if(frame.empty()||object.empty()){std::cout << "Done." << std::endl;return 1;}
    //Get keypoints and descriptors from both scene and object
    objectTracker.getFeatures(object,objectKeypoints);
    sceneTracker.getFeatures(frame,sceneKeypoints);
    objectTracker.drawPoints(object,objectKeypoints,object,CV_RGB(0,200,0));
    objectTracker.drawPoints(frame,sceneKeypoints,frame,CV_RGB(0,200,0));
    if(objectKeypoints.size()&&sceneKeypoints.size()){
        sceneTracker.orbObject->compute(frame, sceneKeypoints, sceneDescriptors);
        objectTracker.orbObject->compute(object, objectKeypoints, objectDescriptors);//Här kraschar den
        matcherObject ->match(objectDescriptors,sceneDescriptors,matches);
        getGoodMatches(matches, good_matches, objectKeypoints, sceneKeypoints);
        std::vector<cv::Point> xWinningPairs, yWinningPairs;
        detect(object, frame, good_matches, objectKeypoints,sceneKeypoints, xWinningPairs, yWinningPairs);
        std::vector<cv::DMatch> best_matches = getBestMatches(good_matches,xWinningPairs, yWinningPairs);
        cv::Mat img_matches;
        cv::drawMatches( object, objectKeypoints, frame, sceneKeypoints,
                   best_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        cv::imshow("Good Matches", img_matches);
        cv::waitKey(0);
    }else{std::cout << "No keypoints. skipping this." << std::endl;}



    counter++;



  }
}
