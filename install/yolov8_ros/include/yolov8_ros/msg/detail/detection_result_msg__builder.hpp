// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolov8_ros:msg/DetectionResultMsg.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__BUILDER_HPP_
#define YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__BUILDER_HPP_

#include "yolov8_ros/msg/detail/detection_result_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace yolov8_ros
{

namespace msg
{

namespace builder
{

class Init_DetectionResultMsg_objects
{
public:
  Init_DetectionResultMsg_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::yolov8_ros::msg::DetectionResultMsg objects(::yolov8_ros::msg::DetectionResultMsg::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolov8_ros::msg::DetectionResultMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolov8_ros::msg::DetectionResultMsg>()
{
  return yolov8_ros::msg::builder::Init_DetectionResultMsg_objects();
}

}  // namespace yolov8_ros

#endif  // YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__BUILDER_HPP_
