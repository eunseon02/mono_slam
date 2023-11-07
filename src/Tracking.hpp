#ifndef TRACKING_H
#define TRACKING_H

#include "frame.hpp"
#include "Visualization.hpp"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>


#include "ORB_extractor.hpp"

using namespace std;
using namespace cv;

class Tracking: public rclcpp::Node
{
public:
    Tracking();

    // void SetLocalMapper(LocalMapping* pLocalMapper);

    // Current Frame
    std::shared_ptr<Frame> mCurrentFrame;
    std::shared_ptr<Frame> mLastFrame;
    // Initialization Variables
    // std::vector<int> mvIniLastMatches;
    // vector<DMatch> mvIniMatches;
    unsigned int mnLastKeyFrameId;
    // std::vector<cv::Point2f> mvbPrevMatched;
    // std::vector<cv::Point3f> mvIniP3D;
    // Frame mInitialFrame;
    void Run();

    std::vector<sensor_msgs::msg::Image::SharedPtr> readImagesFromFolder(const std::string& folderPath);


    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr RawImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr PeocessedImagePublisher;


protected:
    std::shared_ptr<ORB_extractor> mpORBextractor;
    std::shared_ptr<Visualization> mpVisualization;
    Camera* mpcamera;
    int mMaxFrames;

    void TrackPreviousFrame();
    void PoseEstimation(vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> matches, Mat &R, Mat &t);
    void Triangulation(const vector<KeyPoint> keypoints_1, const vector<KeyPoint> keypoints_2, const std::vector<DMatch> &matches, const Mat &R, const Mat &t, vector<Point3d> &points);
    Point2d Projection(const Point2d &p, const Mat &K);
    void ConvertToPose(const cv::Mat& Rcw, const cv::Mat& tcw); 

    // Start
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);


    // bool TrackLocalMap();

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;


    //Current matches in frame
    int mnMatchesInliers;

    // bool NeedNewKeyFrame();

    
    Visualization* pVisualization;

private:
    cv::Mat mLastImage, mCurrentImage;
    vector<KeyPoint> mLastKeypoint, mCurrentKeypoint;
    vector<DMatch> mvIniMatches;
    std::string pkg_directory;


};

#endif