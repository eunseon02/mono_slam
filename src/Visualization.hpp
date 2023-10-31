#ifndef VISUALIZATION_H
#define VISUALIZATION_H

// #include "frame.hpp"
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

class Visualization {
public:
    Visualization(rclcpp::Node::SharedPtr node);
    void VisualizeCamera(std::shared_ptr<Frame> frame);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array;
    int marker_id = 0;
    std::vector<visualization_msgs::msg::Marker> traj;
    geometry_msgs::msg::Pose SE3ToGeometryMsgPose(const SE3& se3Pose);

};
#endif