#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Tracking.hpp"

#include <iostream>
#include <chrono>

#include "visualization.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

Visualization::Visualization(rclcpp::Node::SharedPtr node) : node(node) {
    pub_marker_array = node->create_publisher<visualization_msgs::msg::MarkerArray>("mono/pose", 10);
}

void Visualization::VisualizeCamera()
{
    RCLCPP_INFO(node->get_logger(), "### start visualization ###");

    // visualize previous camera position
    visualization_msgs::msg::MarkerArray marker_array;


    for(auto &marker : traj) {
        marker.color.a *= 0.7;
        marker_array.markers.push_back(marker);
    }

    // Create SE3 pose from rotation and translation
    Sophus::SE3d estimatedCameraPose(Sophus::SO3d::exp(Eigen::Vector3d(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0))),
                    Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));

    // visualize current camera position
    visualization_msgs::msg::Marker currentCam;
    currentCam.header.frame_id = "";
    currentCam.header.stamp = node->now();
    currentCam.ns = "point";
    currentCam.id = marker_id++;
    currentCam.action = visualization_msgs::msg::Marker::ADD;
    currentCam.pose = SE3ToGeometryMsgPose(estimatedCameraPose);
    currentCam.type = visualization_msgs::msg::Marker::SPHERE;
    currentCam.scale.x = 1.0;
    currentCam.scale.y = 1.0;
    currentCam.scale.z = 0.1;
    currentCam.color.g = 1.0;
    currentCam.color.a = 1.0;  // 불투명도

    // position of current camera
    marker_array.markers.push_back(currentCam);
    pub_marker_array->publish(traj);

    RCLCPP_INFO(ndoe->get_logger(), "complete visualization");
}

