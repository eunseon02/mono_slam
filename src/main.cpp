#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "Tracking.hpp"
#include "Visualization.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ORB_extractor");

    // Load Settings and Check
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ORB_SLAM2");
    std::string strSettingsFile = package_share_directory + "/" + argv[2];
    Tracking Tracker(strSettingsFile, node);
    Tracker.Run();



    // visualization
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visual_odometry_markers", 1);
    Visualization visualizer(node);

    // publish visual message (1s)
    rclcpp::WallRate rate(1); 
    while (rclcpp::ok()) {
        visualizer.VisualizeCamera(Tracker.mCurrentFrame);
        rate.sleep();
    }

    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}
