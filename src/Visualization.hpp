#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "camera.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>
#include "frame.hpp"

#include "Visualization.hpp"

using namespace std;
using namespace cv;

// Frame frame;
class Frame; //

class Visualization : public rclcpp::Node {
public:
    Visualization();
    void VisualizeCamera(std::shared_ptr<Frame> frame);
    Mat DrawFrame();

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array;
    int marker_id = 0;
    vector<visualization_msgs::msg::Marker> traj;
    geometry_msgs::msg::Pose SE3ToGeometryMsgPose(const SE3& se3Pose);

};
#endif