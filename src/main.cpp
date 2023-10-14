#include <rclcpp/rclcpp.hpp>
// ... (other includes)

// Declare the function from ORB_extractor.cpp
int extractAndMatchORB(int argc, char **argv);


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ORB_extractor");

    extractAndMatchORB(argc, argv);  // Call the ORB logic

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
