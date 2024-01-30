#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "common.hpp"
// #include "sensor_msgs/msg/image.hpp"

#include "ORB_extractor.hpp"

using namespace std;
using namespace cv;

class ORB_extractor : public rclcpp::Node
{
public:
    ORB_extractor();
    // ORB_extractor() {};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr Raw_ImagePublisher;
    void extractAndMatchORB(const Mat &img_1, const Mat &img_2, vector<KeyPoint> &keypoints_1, vector<KeyPoint> &keypoints_2, vector<DMatch> &matches);
    void extractORB(const cv::Mat &img, vector<KeyPoint> &keypoints);

    // void featureDetection(Mat img, vector<Point2f> & points);
    // void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr MatchingResultPublisher;  
// private:

};
#endif