
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;
std::vector<int> getBoundingBox(cv::KeyPoint, std::vector<BoundingBox>);
int getElementWithMostOccurences(vector<int>);
std::pair<float, float> getXLimits(std::vector<LidarPoint> &points, float laneWidth);
std::pair<float, float> meanStdev(std::vector<float>);


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
    for(auto match = kptMatches.begin(); match != kptMatches.end(); match++) {
        // get the kp from the current frame and determine the bb it belongs to
        cv::KeyPoint c = kptsCurr[match->trainIdx];
        cv::KeyPoint p = kptsPrev[match->queryIdx];

        if (boundingBox.roi.contains(c.pt) && boundingBox.roi.contains(p.pt))
        {
            boundingBox.kptMatches.push_back(*match);
        }
    }

    std::vector<float> xValuesCurr, xValuesPrev;
    std::vector<float> yValuesCurr, yValuesPrev;
    pair<float, float> msxCurr, msyCurr, msxPrev, msyPrev;

    // check keypoints in Current Frame for outliers
    for(auto match = boundingBox.kptMatches.begin(); match != boundingBox.kptMatches.end(); match++) {
        // std::cout << "match->trainIdx: " << match->trainIdx << std::endl;
        cv::KeyPoint c;
        c = kptsCurr[match->trainIdx];
        xValuesCurr.push_back(c.pt.x);
        yValuesCurr.push_back(c.pt.y);

        c = kptsPrev[match->queryIdx];
        xValuesPrev.push_back(c.pt.x);
        yValuesPrev.push_back(c.pt.y);
    }

    msxCurr = meanStdev(xValuesCurr);
    msyCurr = meanStdev(yValuesCurr);
    msxPrev = meanStdev(xValuesPrev);
    msyPrev = meanStdev(yValuesPrev);

    int threshold = 3;
    for (auto match = boundingBox.kptMatches.begin(); match != boundingBox.kptMatches.end(); )
    {

        cv::KeyPoint c = kptsCurr.at(match->trainIdx);
        if ((
            (msxCurr.first - threshold * msxCurr.second < c.pt.x) && (c.pt.x < msxCurr.first + threshold * msxCurr.second) &&
            (msyCurr.first - threshold * msyCurr.second < c.pt.y) && (c.pt.y < msyCurr.first + threshold * msyCurr.second)
        ) &&
        (
            (msxPrev.first - threshold * msxPrev.second < c.pt.x) && (c.pt.x < msxPrev.first + threshold * msxPrev.second) &&
            (msyPrev.first - threshold * msyPrev.second < c.pt.y) && (c.pt.y < msyPrev.first + threshold * msyPrev.second)
        ))
        {
            match++;
        }
        else 
        {
            match = boundingBox.kptMatches.erase(match);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // Parts of the code are taken from https://github.com/udacity/SFND_Camera
    // Lesson 3 - Engineering a Collision Detection System/Estimating TTC with Camera/solution/compute_ttc_camera.cpp 

    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    float dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


pair<float, float> meanStdev(std::vector<float> sequence) {
    // compute mean and stdev of the distances
    // taken from https://stackoverflow.com/a/7616783
    double sum = std::accumulate(sequence.begin(), sequence.end(), 0.0);
    double mean = sum / sequence.size();
    std::vector<double> diff(sequence.size());
    std::transform(sequence.begin(), sequence.end(), diff.begin(),
                    std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / sequence.size());

    return pair<float, float>(mean, stdev);
}


// return upper and a lower limit for valid x values in the lidar points 
// to get rid of outliers. We first get all the valid x values with respect to 
// the lane width and then compute the mean and std of these points. The lower
// limit is mean-1*sddev and the upper limit is mean+1*stdev
pair<float, float> getXLimits(std::vector<LidarPoint> &points, float laneWidth)
{
    std::vector<float> distances; // contains x values within the ego lane
    for (auto it = points.begin(); it != points.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        {
            distances.push_back(it->x);
        }
    }

    pair<float, float> ms = meanStdev(distances);
    float mean = ms.first;
    float stdev = ms.second;

    return pair<float, float>( // return the lower and upper limits 
        mean - 1 * stdev, 
        mean + 1 * stdev
    );
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Parts of the code are taken from https://github.com/udacity/SFND_Camera
    // SFND_Camera/Lesson 3 - Engineering a Collision Detection System/Estimating TTC with Lidar/solution/compute_ttc_lidar.cpp

    double dT = 1/frameRate; // time between two measurements in seconds
    
    // auxiliary variables
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    pair<float, float> limits;
    limits = getXLimits(lidarPointsPrev, 2.0);
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            if (limits.first < it->x && it->x < limits.second)
            { // use an upper and lower limit to remove outliers.
                minXPrev = minXPrev > it->x ? it->x : minXPrev;
            }
        }
    }

    limits = getXLimits(lidarPointsCurr, 2.0);
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {

        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            if (limits.first < it->x && it->x < limits.second)
            { // use an upper and lower limit to remove outliers.
                minXCurr = minXCurr > it->x ? it->x : minXCurr;
            }
        }
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    map <int, vector<int>> tempMatchesPrevCurr;

    // loop over all the matches between current and previous frame
    for(auto match = matches.begin(); match != matches.end(); match++) {
        // get the kp from the current frame and determine the bb it belongs to
        int matchIndexCurr = match->trainIdx;
        cv::KeyPoint keyPointInCurr = currFrame.keypoints.at(matchIndexCurr);
        // cout << "curr" << endl;
        std::vector<int> boundingBoxIdsCurr = getBoundingBox(keyPointInCurr, currFrame.boundingBoxes);
        
        // get the kp from the previous frame and determine the bb it belongs to
        int matchIndexPrev = match->queryIdx;
        cv::KeyPoint keyPointInPrev = prevFrame.keypoints.at(matchIndexPrev);
        // cout << "prev" << endl;
        std::vector<int> boundingBoxIdsPrev = getBoundingBox(keyPointInPrev, prevFrame.boundingBoxes);
        
        for(int boundingBoxIdCurr: boundingBoxIdsCurr) 
        {
            for(int boundingBoxIdPrev: boundingBoxIdsPrev) 
            {
               tempMatchesPrevCurr[boundingBoxIdPrev].push_back(boundingBoxIdCurr);
            }
        }
    }

    for (auto it = tempMatchesPrevCurr.begin(); it != tempMatchesPrevCurr.end(); ++it)
    {
        int bestBoundingBox = getElementWithMostOccurences(it->second);
        //                                   prev,      curr
        bbBestMatches.insert(pair<int, int> (it->first, bestBoundingBox));
    }
}

// returns the element with max occurrences
int getElementWithMostOccurences(vector<int> v)
{
    map<int, int> occ;
    int elementWithMaxOccurences = -1;
    int maxOccurances = -1;

    for(int e: v)
    {
        occ[e] += 1;
        if (occ[e] > maxOccurances)
        {
            elementWithMaxOccurences = e;
        }
    }

    return elementWithMaxOccurences;
}


// returns the ids of the bounding boxes that enclose the keypoint
std::vector<int> getBoundingBox(cv::KeyPoint keyPoint, std::vector<BoundingBox> boundingBoxes)
{
    std::vector<int> bb;
    for(auto currBoundingBox = boundingBoxes.begin(); currBoundingBox != boundingBoxes.end(); currBoundingBox++) 
    {
        if (currBoundingBox->roi.contains(keyPoint.pt))
        {
            bb.push_back(currBoundingBox->boxID);
        }
    }
    return bb;
}
