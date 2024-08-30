cmake_minimum_required(VERSION 3.8)
project(gr1-localization-orb-slam3)

set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native -w")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -w")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# Match Opencv version
find_package(OpenCV 4 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
#find_package(backward_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

set(yolov8_ros_DIR "/home/nvidia/yolov8_ros/install/yolov8_ros/share/yolov8_ros/cmake")
find_package(yolov8_ros REQUIRED)


include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include/GR1-Localization-Orb-Slam3
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
  ${DBoW2_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


# RGBD-Yolo
add_executable(rgbd_yolo
  src/rgbd_yolo.cpp
  src/rgbd_yolo_node.cpp
  src/kalman_filter.cpp
)
ament_target_dependencies(rgbd_yolo rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV message_filters nav_msgs tf2_ros tf2 yolov8_ros)
target_link_libraries(rgbd_yolo ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so)

target_include_directories(rgbd_yolo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${yolov8_ros_INCLUDE_DIRS}  # 添加这一行
)


install(TARGETS rgbd_yolo  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
