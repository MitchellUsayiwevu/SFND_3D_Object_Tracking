
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{

    std::vector<double> prevPoints_x, currPoints_x;
    for(auto & prevLidarPoint : lidarPointsPrev){

        if( abs(prevLidarPoint.y)<= 2 ){
            prevPoints_x.push_back(prevLidarPoint.x);
        }
    }

    for(auto & currLidarPoint : lidarPointsCurr){

        if( abs(currLidarPoint.y)<= 2 ){
            currPoints_x.push_back(currLidarPoint.x);
        }
    }

    if(currPoints_x.empty() || prevPoints_x.empty()){
        TTC = NAN;
        return;
    }

    std::sort(prevPoints_x.begin(), prevPoints_x.end(), [](const double &a, const double &b) {
        return a < b;  // ascending x values
    });

    std::sort(currPoints_x.begin(), currPoints_x.end(), [](const double &a, const double &b) {
        return a < b;  // ascending x values
    });

    double median_prev,median_curr;      // use median values which are more robust to outliers

    if(prevPoints_x.size()%2 == 0){
        median_prev = ( prevPoints_x[prevPoints_x.size()/2] + prevPoints_x[(prevPoints_x.size()/2) -1 ])/2 ;
        //average of the 2 middle numbers if the vector has an even size
    }else{
        median_prev = prevPoints_x[prevPoints_x.size()/2];
        //middle number if the vector has an odd size
    }

    if(currPoints_x.size()%2 == 0){
        median_curr = (currPoints_x[currPoints_x.size()/2] + currPoints_x[(currPoints_x.size()/2) -1 ])/2 ;
        //average of the 2 middle numbers if the vector has an even size
    }else{
        median_curr = currPoints_x[currPoints_x.size()/2];
        //middle number if the vector has an odd size
    }

    TTC  = ( median_curr * (1/frameRate) )/(median_prev - median_curr) ;

}



void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //outer loop over all keypoint matches between current frame and previous frame

    /// using keypoint matches to match bounding boxes
    //loop over bounding boxes in current frame
    //look for which keypoint is located inside the bounding box roi from current frame
    //if you find a bounding box whose roi the keypoint in question, keep track of bounding box ID
    //loop over bounding boxes in previous frame
    //check if matching keypoint of said keypoint in prev frame is contained in the bounding box roi of the prev frame
    //if so keep track of bounding box ID
    // Enter the BoundingBox_ID in prev frame and BoundingBox_ID in current frame in bbmatches if they're both not -1


    // use multimap instead of std::map as std::map only keeps unique keys.

    std::multimap<int, int> bbmatches;                     // prevID:currID;

    for( std::vector<cv::DMatch>::iterator it = matches.begin(); it<matches.end(); it ++){

        int currBoundingBoxID = -1;
        int prevBoundingBoxID = -1;

        for (auto& currBoundingBox: currFrame.boundingBoxes){
            auto curr_keypoint_index = it->trainIdx;
            cv::Point2f currPt = currFrame.keypoints[curr_keypoint_index].pt;
            cv::Point curr_keypoint_location(cvRound(currPt.x), cvRound(currPt.y));
            if (currBoundingBox.roi.contains(curr_keypoint_location)){
                currBoundingBoxID = currBoundingBox.boxID;
                break;
             }
        }

        for (auto& prevBoundingBox: prevFrame.boundingBoxes){
            auto prev_keypoint_index = it->queryIdx;
            cv::Point2f prevPt = prevFrame.keypoints[prev_keypoint_index].pt;
            cv::Point prev_keypoint_location(cvRound(prevPt.x), cvRound(prevPt.y));
            if (prevBoundingBox.roi.contains(prev_keypoint_location)){
                prevBoundingBoxID = prevBoundingBox.boxID;
                break;
            }
        }

        if( (prevBoundingBoxID != -1) && (currBoundingBoxID != -1 ) ){
//            bbmatches[prevBoundingBoxID] = currBoundingBoxID;
            bbmatches.insert({prevBoundingBoxID, currBoundingBoxID});
        }

    }

    /// Find the best bounding box matches

    std::map< std::pair<const int,int>, int > paircounts;

    for(auto& BBpair: bbmatches ){
        paircounts[BBpair]+=1;
    }

    // now find out which pair has the highest frequency

    for (auto & prevBoundingBox: prevFrame.boundingBoxes){

        int max_freq{0};
        int curr_ID = -1;

        for (auto& pair: paircounts ){
            if(prevBoundingBox.boxID == pair.first.first){
                if (max_freq < pair.second){
                    max_freq = pair.second;
                    curr_ID = pair.first.second;
                }
            }
        }

        if(curr_ID !=-1){
            bbBestMatches[prevBoundingBox.boxID] = curr_ID;
        }

    }

}
