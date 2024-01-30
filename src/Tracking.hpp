#ifndef TRACKING_H
#define TRACKING_H

#include "frame.hpp"
#include "Visualization.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


//---
// #include <Eigen/Core>
// #include <g2o/core/base_vertex.h>
// #include <g2o/core/base_unary_edge.h>
// #include <g2o/core/sparse_optimizer.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/solver.h>
// #include <g2o/core/optimization_algorithm_gauss_newton.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
// #include <sophus/se3.hpp>
// // #include <chrono>


#include "ORB_extractor.hpp"

#define MAX_FRAME 2000

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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr test_keypoint_Publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mappooint_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mappooint_publisher2;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mappooint_publisher;



    int id;
    int mappoint_id;
    static unsigned long nextId; // for Keyframe ID 
    int validation;

    // char text[100];
    // int fontFace = cv::FONT_HERSHEY_PLAIN;
    // double fontScale = 1;
    // int thickness = 1;
    // cv::Point textOrg(10, 50);

    char* dataset_images_location;
    char* dataset_poses_location;


    std::vector<geometry_msgs::msg::Point> path;
    std::vector<geometry_msgs::msg::Quaternion> orientation_list;

    vector<geometry_msgs::msg::Point> mappoints;
    vector<geometry_msgs::msg::Point> mCurrentMappoints;

    vector<Frame*> keyframes;

    geometry_msgs::msg::Point Point3dToGeometryMsgPoint(const cv::Point3d& point_3d);

    vector<Point3d> current3D;

    // vector<Point3d> path;

protected:
    std::shared_ptr<ORB_extractor> mpORBextractor;
    std::shared_ptr<Visualization> mpVisualization;
    std::shared_ptr<Camera> mpCamera;
    // Camera* mpcamera;
    int mMaxFrames;

    void TrackPreviousFrame(Frame* f);
    void PoseEstimation(vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> matches, Mat &R, Mat &t);
    void Triangulation(Frame* f,const vector<KeyPoint> keypoints_1, const vector<KeyPoint> keypoints_2, const std::vector<DMatch> &matches, const Mat &R, const Mat &t, vector<Point3d> &points);
    Point2d Projection(const Point2d &p, const Mat &K);
    void ConvertToPose(const cv::Mat& Rcw, const cv::Mat& tcw); 

    cv::Mat EigenMatToCvMat(const Eigen::Matrix<double, 3, 3>& eigenMat);

    void publishPointVisualization(Frame* f);
    // void PointVisualization(const geometry_msgs::msg::Point& point);
    void publishMapPointVisualization(Frame* f);
    void publishMapPointVisualization2();
    vector<Point2f> getGreyCamGroundPoses();
    void bundleAdjustmentG2O(Sophus::SE3d &pose);

    // Start
    void GrabImage(Mat msg, Mat traj, int numFrame);

    // bool TrackLocalMap();

    //Calibration matrix
    cv::Mat mK;
    // Mat33 mK;
    cv::Mat mDistCoef;


    //Current matches in frame
    int mnMatchesInliers;

    bool NeedNewKeyFrame(Frame* f);
    vector<double> getAbsoluteScales();
    // auto groundScales = getAbsoluteScales();
    vector<double> groundScales;
    vector<Point2f> groundPoses;
    
    double scale = 1.00;

    Mat Rcw, tcw;
    Visualization* pVisualization;

private:
    cv::Mat mLastImage, mCurrentImage;
    vector<KeyPoint> mLastKeypoint, mCurrentKeypoint;
    vector<DMatch> mvIniMatches;
    std::string pkg_directory;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


};

#endif