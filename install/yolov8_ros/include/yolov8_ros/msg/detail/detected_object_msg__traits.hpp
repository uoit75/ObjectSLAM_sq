// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolov8_ros:msg/DetectedObjectMsg.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__TRAITS_HPP_
#define YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__TRAITS_HPP_

#include "yolov8_ros/msg/detail/detected_object_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'masks_instance'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolov8_ros::msg::DetectedObjectMsg>()
{
  return "yolov8_ros::msg::DetectedObjectMsg";
}

template<>
inline const char * name<yolov8_ros::msg::DetectedObjectMsg>()
{
  return "yolov8_ros/msg/DetectedObjectMsg";
}

template<>
struct has_fixed_size<yolov8_ros::msg::DetectedObjectMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolov8_ros::msg::DetectedObjectMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolov8_ros::msg::DetectedObjectMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__TRAITS_HPP_
