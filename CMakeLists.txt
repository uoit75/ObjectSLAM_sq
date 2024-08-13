cmake_minimum_required(VERSION 3.10)

project(yolov8_ros VERSION 1.0.0 LANGUAGES C CXX CUDA)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wno-deprecated-declarations")

# 查找依赖包
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)  # 添加此行
find_package(builtin_interfaces REQUIRED)

# 添加自定义消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObjectMsg.msg"
  "msg/DetectionResultMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs 
)

# 包含生成的消息头文件路径
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# 包含路径设置
include_directories(
  "/usr/include/eigen3"
  ${fastrtps_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# TensorRT 库和头文件
set(TensorRT_ROOT /usr/src/tensorrt)
find_library(TRT_NVINFER NAMES nvinfer HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVINFER_PLUGIN NAMES nvinfer_plugin HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVONNX_PARSER NAMES nvonnxparser HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVCAFFE_PARSER NAMES nvcaffe_parser HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_path(TENSORRT_INCLUDE_DIR NAMES NvInfer.h HINTS ${TensorRT_ROOT} PATH_SUFFIXES include)
set(SAMPLES_COMMON_DIR ${TensorRT_ROOT}/samples/common)
include_directories(${TensorRT_ROOT}/samples/common)

# 构建库
file(GLOB CPPS 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*.cu
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*.cpp
  ${TensorRT_ROOT}/samples/common/logger.cpp 
  ${TensorRT_ROOT}/samples/common/sampleOptions.cpp
)
list(REMOVE_ITEM CPPS app_yolov8_seg.cpp)

add_library(yolov8_library SHARED ${CPPS})
target_link_libraries(yolov8_library 
  ${CUDA_LIBRARIES} 
  ${TRT_NVINFER} ${TRT_NVINFER_PLUGIN} ${TRT_NVONNX_PARSER} ${TRT_NVCAFFE_PARSER}
  ${OpenCV_LIBRARIES}
)
set_property(TARGET yolov8_library PROPERTY CUDA_ARCHITECTURES 50 61 72 75)
target_compile_options(yolov8_library PUBLIC 
   $<$<COMPILE_LANGUAGE:CUDA>:--default-stream per-thread -lineinfo --use_fast_math --disable-warnings>)

# 构建可执行文件
add_executable(app_yolov8_seg_ros src/app_yolov8_seg_ros.cpp)


# target_link_libraries(app_yolov8_seg_ros
#   yolov8_library
#   ${CUDA_LIBRARIES} 
#   ${OpenCV_LIBRARIES}
# )

target_link_libraries(app_yolov8_seg_ros
  yolov8_library
  ${CUDA_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${fastrtps_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp  # 添加这行
)

ament_target_dependencies(app_yolov8_seg_ros
  rclcpp
  sensor_msgs
  geometry_msgs 
  std_msgs
  rosidl_typesupport_cpp  # 添加此行
  cv_bridge
)

# 安装
install(TARGETS app_yolov8_seg_ros
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
