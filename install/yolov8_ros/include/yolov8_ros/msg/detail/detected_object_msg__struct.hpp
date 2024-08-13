// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolov8_ros:msg/DetectedObjectMsg.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__STRUCT_HPP_
#define YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'bbox_points'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'masks_instance'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yolov8_ros__msg__DetectedObjectMsg __attribute__((deprecated))
#else
# define DEPRECATED__yolov8_ros__msg__DetectedObjectMsg __declspec(deprecated)
#endif

namespace yolov8_ros
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectedObjectMsg_
{
  using Type = DetectedObjectMsg_<ContainerAllocator>;

  explicit DetectedObjectMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : masks_instance(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = 0l;
      this->confidence = 0.0f;
    }
  }

  explicit DetectedObjectMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : masks_instance(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = 0l;
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _label_type =
    int32_t;
  _label_type label;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _bbox_points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point_<ContainerAllocator>>::other>;
  _bbox_points_type bbox_points;
  using _masks_instance_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _masks_instance_type masks_instance;

  // setters for named parameter idiom
  Type & set__label(
    const int32_t & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__bbox_points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point_<ContainerAllocator>>::other> & _arg)
  {
    this->bbox_points = _arg;
    return *this;
  }
  Type & set__masks_instance(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->masks_instance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolov8_ros__msg__DetectedObjectMsg
    std::shared_ptr<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolov8_ros__msg__DetectedObjectMsg
    std::shared_ptr<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectedObjectMsg_ & other) const
  {
    if (this->label != other.label) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->bbox_points != other.bbox_points) {
      return false;
    }
    if (this->masks_instance != other.masks_instance) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectedObjectMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectedObjectMsg_

// alias to use template instance with default allocator
using DetectedObjectMsg =
  yolov8_ros::msg::DetectedObjectMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolov8_ros

#endif  // YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__STRUCT_HPP_
