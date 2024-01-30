#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"



#include "Tracking.hpp"
#include "Visualization.hpp"
// #include "common.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using namespace std::chrono_literals;
using namespace std;
using namespace cv;


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<Pub>();
    // rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());

    // string pkg_directory = "/home/eunseon/ros2_ws/src/mono_slam/data/images";
    
    Tracking Tracker;
    Visualization visualizer();    

    Tracker.Run();

    // rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}