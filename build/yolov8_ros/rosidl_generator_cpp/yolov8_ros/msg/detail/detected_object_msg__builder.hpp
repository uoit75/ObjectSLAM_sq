// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolov8_ros:msg/DetectedObjectMsg.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__BUILDER_HPP_
#define YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__BUILDER_HPP_

#include "yolov8_ros/msg/detail/detected_object_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace yolov8_ros
{

namespace msg
{

namespace builder
{

class Init_DetectedObjectMsg_masks_instance
{
public:
  explicit Init_DetectedObjectMsg_masks_instance(::yolov8_ros::msg::DetectedObjectMsg & msg)
  : msg_(msg)
  {}
  ::yolov8_ros::msg::DetectedObjectMsg masks_instance(::yolov8_ros::msg::DetectedObjectMsg::_masks_instance_type arg)
  {
    msg_.masks_instance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolov8_ros::msg::DetectedObjectMsg msg_;
};

class Init_DetectedObjectMsg_bbox_points
{
public:
  explicit Init_DetectedObjectMsg_bbox_points(::yolov8_ros::msg::DetectedObjectMsg & msg)
  : msg_(msg)
  {}
  Init_DetectedObjectMsg_masks_instance bbox_points(::yolov8_ros::msg::DetectedObjectMsg::_bbox_points_type arg)
  {
    msg_.bbox_points = std::move(arg);
    return Init_DetectedObjectMsg_masks_instance(msg_);
  }

private:
  ::yolov8_ros::msg::DetectedObjectMsg msg_;
};

class Init_DetectedObjectMsg_confidence
{
public:
  explicit Init_DetectedObjectMsg_confidence(::yolov8_ros::msg::DetectedObjectMsg & msg)
  : msg_(msg)
  {}
  Init_DetectedObjectMsg_bbox_points confidence(::yolov8_ros::msg::DetectedObjectMsg::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_DetectedObjectMsg_bbox_points(msg_);
  }

private:
  ::yolov8_ros::msg::DetectedObjectMsg msg_;
};

class Init_DetectedObjectMsg_label
{
public:
  Init_DetectedObjectMsg_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedObjectMsg_confidence label(::yolov8_ros::msg::DetectedObjectMsg::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_DetectedObjectMsg_confidence(msg_);
  }

private:
  ::yolov8_ros::msg::DetectedObjectMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolov8_ros::msg::DetectedObjectMsg>()
{
  return yolov8_ros::msg::builder::Init_DetectedObjectMsg_label();
}

}  // namespace yolov8_ros

#endif  // YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__BUILDER_HPP_
