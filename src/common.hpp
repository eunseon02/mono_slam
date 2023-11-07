//std
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
// #include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <opencv2/opencv.hpp>

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

typedef Eigen::Matrix<double, 3, 3> Mat33;