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

int main(int argc, char** argv){
    // Read base scene image
    std::string scenePath = "/Users/Fredrik/Datasets/FloorTextures/wood-floor-pattern-calculator-ideas-photos_771567.jpg";
    float sceneWidth = 4;                                   //Scene width in meter
    cv::Mat baseScene = cv::imread(scenePath,cv::IMREAD_COLOR);     //Read the scene image as 8uC3
    float resolution = ((float) baseScene.cols)/sceneWidth;  //Resolution expressed as pixels/meter
    cv::imshow("mark",baseScene);
    cv::waitKey(0);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> ids;

    cv::Mat marker;
    int id = 34;
    float markerSize  = 1;//20cm
    float sidePixels = resolution*markerSize;
    int borderBits = 1;
    dictionary->drawMarker(id,(int)sidePixels,marker,borderBits);               //Construct marker
    cv::cvtColor(marker,marker,CV_GRAY2BGR);                                    //Convert marker to 3-channel 8 bit image

    double angle = 0;
    cv::RotatedRect rec = cv::RotatedRect(cv::Point2f(sidePixels/2,sidePixels/2),cv::Size2f(sidePixels,sidePixels),(float) angle);
    float w =(float) rec.boundingRect().width;   //Width of the minimal resulting bounding rect
    float h =(float) rec.boundingRect().height;  //Height of the minimal resulting bounding rect

    cv::Mat rot =  cv::getRotationMatrix2D(cv::Point2f(w/2,h/2),angle,1);//rotation matrix around marker center
    int vert = (int) (h-sidePixels)/2;//Border thickness vertical
    int hori = (int) (w-sidePixels)/2;//Border thickness horizontal
    cv::Mat rotated;
    cv::copyMakeBorder(marker,rotated,vert,vert,hori,hori,0,cv::Scalar(200));   //Copy to marker to new mat with necessary border
    cv::warpAffine(rotated,rotated,rot,cv::Size2f(w,h),cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT,250);//Rotate the new marker image
    cv::Mat mask = cv::Mat::zeros(rotated.size(),CV_8UC1);                      //Create mask and fill with zeros
    cv::Point2f vertices_float[4];
    rec.points(vertices_float);                                                       //Get the vertices of the rotated rect
    cv::Point vertices[4];
    for(int i=0;i<4;i++){vertices[i] = vertices_float[i] + cv::Point2f(hori,vert);}//Convert to point and shift to correct coordinates
    cv::fillConvexPoly(mask,vertices,4,cv::Scalar(255),4,0);
    //Create image with rotated marker (enlarged image to fit whole marker) use boundingrect    DONE
    //Create a mask. Create image of same size as rotated mask. Set rotated rectangle?          DONE
        //-fillconvexpoly, rotatedrect,
    //Copy over rotated marker using mask
    cv::Mat rotated2;
    rotated.copyTo(rotated2);
    //rotated.convertTo(rotated2,CV_8UC3);
    std::cout <<"Type: "  << rotated.type() <<", "<<rotated2.type() << ", " << baseScene.type() << std::endl;
    cv::Rect roi(0,0,w,h);
    cv::Mat subFrame = baseScene(roi);
    //std::cout <<"Type subframe: " << subFrame.type() <<std::endl;
    //rotated2.copyTo(subFrame);//,mask);
    rotated2.copyTo(baseScene(roi),mask);



    cv::imshow("mark",marker);
    cv::waitKey(0);
    cv::imshow("mark",rotated);
    cv::waitKey(0);
    cv::imshow("mark",mask);
    cv::waitKey(0);
    cv::imshow("mark",baseScene);
    cv::waitKey(0);
    cv::imshow("mark",subFrame);
    cv::waitKey(0);




}



//Texture image sources
//  http://tierraeste.com/wp-content/uploads/wood-floor-pattern-calculator-ideas-photos_771567.jpg
