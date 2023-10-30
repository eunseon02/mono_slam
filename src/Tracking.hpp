#ifndef TRACKING_H
#define TRACKING_H

#include "frame.hpp"
#include "visualization.hpp"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>


#include "ORB_extractor.hpp"

using namespace std;
using namespace cv;

class Tracking
{
public:
    Tracking(string strSettingPath);
    // Tracking(FramePublisher* pFramePublisher, MapPublisher* pMapPublisher, Map* pMap, string strSettingPath);

    // void SetLocalMapper(LocalMapping* pLocalMapper);

    // Current Frame
    std::shared_ptr<Frame> mCurrentFrame;


    // Initialization Variables
    // std::vector<int> mvIniLastMatches;
    vector<DMatch> mvIniMatches;
    unsigned int mnLastKeyFrameId;
    // std::vector<cv::Point2f> mvbPrevMatched;
    // std::vector<cv::Point3f> mvIniP3D;
    // Frame mInitialFrame;
    void Run();  
protected:
    std::shared_ptr<ORB_extractor> mpORBextractor;
    Camera* mpcamera;
    int mMaxFrames;
    // void CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw);

    // bool RelocalisationRequested();
    // bool Relocalisation();    

    // void UpdateReference();
    // void UpdateReferencePoints();
    // void UpdateReferenceKeyFrames();

    void TrackPreviousFrame();
    void PoseEstimation(vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> matches, Mat &R, Mat &t);
    void Triangulation(const vector<KeyPoint> keypoints_1, const vector<KeyPoint> keypoints_2, const std::vector<DMatch> &matches, const Mat &R, const Mat &t, vector<Point3d> &points);
    Point2d Projection(const Point2d &p, const Mat &K);
    void Create4x4Transform(const cv::Mat& Rcw, const cv::Mat& tcw);

    // Start
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    std::shared_ptr<Frame> mLastFrame;

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

};

#endif