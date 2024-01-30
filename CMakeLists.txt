cmake_minimum_required(VERSION 3.5)
project(mono_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_export_dependencies REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(Threads REQUIRED)


add_executable(mono_slam src/main.cpp src/Tracking.cpp src/ORB_extractor.cpp src/frame.cpp src/Visualization.cpp)

#add_executable(mono_slam src/main.cpp src/Tracking.cpp src/ORB_extractor.cpp src/frame.cpp)

ament_target_dependencies(mono_slam 
  rclcpp 
  cv_bridge 
  image_transport 
  sensor_msgs 
  visualization_msgs 
  OpenCV 
  tf2 
  tf2_ros 
  geometry_msgs
)

install(TARGETS
  mono_slam
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
