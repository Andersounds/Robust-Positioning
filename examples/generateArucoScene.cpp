#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
/*
This file is to be used to generate a 2d floor environment with ArUco markers to be used
for triangulation evaluation

Specify:
    -Aruco dictionary size
    -Base scene and its physical width (x)
    for each aruco marker
        specify coordinate, rotation, physical size, code number
*/

/*
TODO:
    - specify output file name upon code run
    - Read parameters from file??
    - Make proper cpp+hpp file to src directory
*/
struct marker{
    int id;                         //Must be within range of the chosen dictionary
    float x;                        //Meter
    float y;                        //Meter
    float size;                     //Meter
    float rotation;                 //Degrees
    marker(int id_, float x_, float y_, float size_, float rotation_){
        id=id_;
        x=x_;
        y=y_;
        size=size_;
        rotation=rotation_;
    }
};


int createArucoScene(cv::Mat& baseScene,float sceneWidth,
                        int dictType,                                           //As specified in aruco. Ex. cv::aruco::DICT_4X4_50
                        std::vector<marker> markers){
    if(baseScene.type() != 16){std::cerr << "Input image must be of type 8UC3" << std::endl;return 0;}
    //Some settings..
    int borderBits = 1;
    //Create dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictType);
    float resolution = ((float) baseScene.cols)/sceneWidth;  //Resolution expressed as pixels/meter


    int counter = 0;
    for(marker M:markers){
        //Create marker
        cv::Mat marker; //init marker mat
        float sidePixels = resolution*M.size;                                   //Marker size expressed in pixles
        dictionary->drawMarker(M.id,(int)sidePixels,marker,borderBits);         //Construct marker
        cv::cvtColor(marker,marker,CV_GRAY2BGR);                                //Convert marker to 3-channel 8 bit image
        //Rotate marker
        cv::RotatedRect rec = cv::RotatedRect(cv::Point2f(sidePixels/2,sidePixels/2),cv::Size2f(sidePixels,sidePixels),M.rotation);
        float w =(float) rec.boundingRect().width;   //Width of the minimal resulting bounding rect [pixles]
        float h =(float) rec.boundingRect().height;  //Height of the minimal resulting bounding rect [pixles]
        if((M.x*resolution-w/2)<0 || (M.x*resolution+w/2)>baseScene.cols || (M.y*resolution-h/2)<0 || (M.y*resolution+h/2)>baseScene.rows){std::cerr <<" Marker "<< counter << " placed too close too edge" << std::endl;return 0;} //Safeguard
        cv::Mat rot =  cv::getRotationMatrix2D(cv::Point2f(w/2,h/2),M.rotation,1);//rotation matrix around marker center
        int vert = (int) (h-sidePixels)/2;           //Border thickness vertical
        int hori = (int) (w-sidePixels)/2;           //Border thickness horizontal
        cv::Mat rotated;                             //container for rotated marker
        cv::copyMakeBorder(marker,rotated,vert,vert,hori,hori,0,cv::Scalar(0,0,0));   //Copy to marker to new mat with necessary border
        cv::warpAffine(rotated,rotated,rot,cv::Size2f(w,h),cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));//Rotate the new marker image
        //Create mask
        cv::Mat mask = cv::Mat::zeros(rotated.size(),CV_8UC1);                        //Create mask and fill with zeros
        cv::Point2f vertices_float[4];
        rec.points(vertices_float);                                                   //Get the vertices of the rotated rect
        cv::Point vertices[4];
        for(int i=0;i<4;i++){vertices[i] = vertices_float[i] + cv::Point2f(hori,vert);}//Convert to point and shift to correct coordinates
        cv::fillConvexPoly(mask,vertices,4,cv::Scalar(255,255,255),4,0);               //Set the area covered by the marker to 255
        //Copy rotated marker over to base scene using mask
        cv::Rect roi(M.x*resolution-w/2,M.y*resolution-h/2,w,h);
        cv::Mat subFrame = baseScene(roi);
        rotated.copyTo(baseScene(roi),mask);
        counter++;
    }

return 1;


}

/*Returns a linSpace sequence starting from start with length no of steps of size step
 *
 */
std::vector<float> linSpace(float start,float stop,float length){
    std::vector<float> path;
    float step = (stop-start)/length;
    for(float i=0;i<length;i++){
        path.push_back(step*i+start);
    }
    return path;
}
std::vector<float> sequence(float start,float step,float max){
    std::vector<float> path;
    float value = 0;
    float i = 0;
    while(value < max){
        value = step*i+start;
        path.push_back(value);
        i++;
    }
    return path;
}
int main(int argc, char** argv){
    // Read base scene image
    std::string scenePath = "/Users/Fredrik/Datasets/FloorTextures/wood-floor-pattern-calculator-ideas-photos_771567.jpg";
    cv::Mat baseScene = cv::imread(scenePath,cv::IMREAD_COLOR);     //Read the scene image as 8uC3


    float sceneWidth = 4;                                   //Scene width in meter
    float number = 10;
    std::vector<float> x1=linSpace(0.4,3.6,number);
    std::vector<float> x2=linSpace(0.3,3,number);
    std::vector<float> x3=linSpace(0.2,3.9,number);
    std::vector<float> x4=linSpace(0.25,3.8,number);
    std::vector<float> y1=linSpace(0.2,0.2,number);
    std::vector<float> y2=linSpace(1,1,number);
    std::vector<float> y3=linSpace(1.6,1.6,number);
    std::vector<float> y4=linSpace(2,2.1,number);
    std::vector<float> markerSizes1=linSpace(0.1,0.2,number);
    std::vector<float> markerSizes2=linSpace(0.25,0.15,number);
    std::vector<float> markerSizes3=linSpace(0.1,0.15,number);
    std::vector<float> markerSizes4=linSpace(0.08,0.12,number);
    std::vector<float> rotations1 = linSpace(0,1000,number);
    std::vector<float> rotations2 = linSpace(170,400,number);
    std::vector<float> rotations3 = linSpace(90,180,number);
    std::vector<float> rotations4 = linSpace(90,1800,number);



    std::vector<marker> markers;
    for(int i=0;i<x1.size();i++){
        markers.push_back(marker(i,x1[i],y1[i],markerSizes1[i], rotations1[i]));
    }
    for(int i=0;i<x2.size();i++){
        markers.push_back(marker(i,x2[i],y2[i],markerSizes2[i], rotations2[i]));
    }
    for(int i=0;i<x2.size();i++){
        markers.push_back(marker(i,x3[i],y3[i],markerSizes3[i], rotations3[i]));
    }
    for(int i=0;i<x2.size();i++){
        markers.push_back(marker(i,x4[i],y4[i],markerSizes4[i], rotations4[i]));
    }

    createArucoScene(baseScene, sceneWidth, cv::aruco::DICT_4X4_50,markers);
    std::string filename = "/Users/Fredrik/Datasets/FloorTextures/test1.png";
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    cv::imwrite(filename,baseScene,compression_params);
    cv::imshow("Base scene with markers",baseScene);
    cv::waitKey(0);


    return 1;
}



//Texture image sources
//  http://tierraeste.com/wp-content/uploads/wood-floor-pattern-calculator-ideas-photos_771567.jpg
