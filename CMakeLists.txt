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
#find_package(Threads REQUIRED)


add_executable(mono_slam src/main.cpp src/Tracking.cpp src/ORB_extractor.cpp src/frame.cpp src/Visualization.cpp)

#add_executable(mono_slam src/main.cpp src/Tracking.cpp src/ORB_extractor.cpp src/frame.cpp)

ament_target_dependencies(mono_slam rclcpp cv_bridge image_transport sensor_msgs visualization_msgs OpenCV)


#add_executable(mono_test src/mono.cpp)
#ament_target_dependencies(mono_test rclcpp cv_bridge image_transport sensor_msgs visualization_msgs OpenCV)



install(TARGETS
  mono_slam
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

#target_link_libraries(mono_slam Threads::Threads)

ament_target_dependencies(mono_slam rclcpp cv_bridge)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
