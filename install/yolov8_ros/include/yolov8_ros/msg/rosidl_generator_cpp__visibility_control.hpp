// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define YOLOV8_ROS__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_yolov8_ros __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_yolov8_ros __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_yolov8_ros __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_yolov8_ros __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_yolov8_ros
    #define ROSIDL_GENERATOR_CPP_PUBLIC_yolov8_ros ROSIDL_GENERATOR_CPP_EXPORT_yolov8_ros
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_yolov8_ros ROSIDL_GENERATOR_CPP_IMPORT_yolov8_ros
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_yolov8_ros __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_yolov8_ros
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_yolov8_ros __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_yolov8_ros
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // YOLOV8_ROS__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
