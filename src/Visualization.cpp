#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

// #include "Tracking.hpp"

// #include "common.hpp"

#include <iostream>
#include <chrono>

#include "Visualization.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

Visualization::Visualization() : Node("Visualization") {
    pub_marker_array = this->create_publisher<visualization_msgs::msg::MarkerArray>("mono/pose", 10);
}

void Visualization::VisualizeCamera(Frame* frame)
{
    RCLCPP_INFO(this->get_logger(), "### start visualization ###");

    // visualize previous camera position
    visualization_msgs::msg::MarkerArray marker_array;

    for(auto &marker : traj) {
        marker.color.a *= 0.7;
        marker_array.markers.push_back(marker);
    }

    // visualize current camera position
    visualization_msgs::msg::Marker currentCam;
    currentCam.header.frame_id = "";
    currentCam.header.stamp = this->now();
    currentCam.ns = "point";
    currentCam.id = marker_id++;
    currentCam.action = visualization_msgs::msg::Marker::ADD;
    currentCam.pose = SE3ToGeometryMsgPose(frame->Pose());
    currentCam.type = visualization_msgs::msg::Marker::SPHERE;
    currentCam.scale.x = 1.0;
    currentCam.scale.y = 1.0;
    currentCam.scale.z = 0.1;
    currentCam.color.g = 1.0;
    currentCam.color.a = 1.0;  

    // position of current camera
    marker_array.markers.push_back(currentCam);
    pub_marker_array->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "complete visualization");
}

geometry_msgs::msg::Pose Visualization::SE3ToGeometryMsgPose(const SE3& se3Pose) {
    geometry_msgs::msg::Pose pose_msg;

    // SE3 포즈의 위치 및 방향 정보를 Pose 메시지에 복사
    pose_msg.position.x = se3Pose.translation().x();
    pose_msg.position.y = se3Pose.translation().y();
    pose_msg.position.z = se3Pose.translation().z();

    Eigen::Quaterniond quaternion(Eigen::Matrix3d(se3Pose.rotationMatrix()));
    pose_msg.orientation.x = quaternion.x();
    pose_msg.orientation.y = quaternion.y();
    pose_msg.orientation.z = quaternion.z();
    pose_msg.orientation.w = quaternion.w();

    return pose_msg;
}

cv::Mat Visualization::DrawFrame()
{
    cv::Mat image;
    return image;
}