#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
// #include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace cv;

class ORB_extractor
{
public:
    ORB_extractor(){};

    void extractAndMatchORB(const Mat &img_1, const Mat &img_2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> matches);  
private:

};

#endif