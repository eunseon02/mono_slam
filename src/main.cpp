#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "Tracking.hpp"
#include "Visualization.hpp"
// #include "common.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<Pub>();

    // string pkg_directory = "/home/eunseon/ros2_ws/src/mono_slam/data/images";
    
    Tracking Tracker;
    Visualization visualizer();    

    Tracker.Run();

    // rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}