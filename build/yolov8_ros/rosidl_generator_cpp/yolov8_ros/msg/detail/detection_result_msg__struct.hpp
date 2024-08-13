// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolov8_ros:msg/DetectionResultMsg.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__STRUCT_HPP_
#define YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'objects'
#include "yolov8_ros/msg/detail/detected_object_msg__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yolov8_ros__msg__DetectionResultMsg __attribute__((deprecated))
#else
# define DEPRECATED__yolov8_ros__msg__DetectionResultMsg __declspec(deprecated)
#endif

namespace yolov8_ros
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectionResultMsg_
{
  using Type = DetectionResultMsg_<ContainerAllocator>;

  explicit DetectionResultMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit DetectionResultMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _objects_type =
    std::vector<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>, typename ContainerAllocator::template rebind<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>>::other>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__objects(
    const std::vector<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>, typename ContainerAllocator::template rebind<yolov8_ros::msg::DetectedObjectMsg_<ContainerAllocator>>::other> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolov8_ros__msg__DetectionResultMsg
    std::shared_ptr<yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolov8_ros__msg__DetectionResultMsg
    std::shared_ptr<yolov8_ros::msg::DetectionResultMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectionResultMsg_ & other) const
  {
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectionResultMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectionResultMsg_

// alias to use template instance with default allocator
using DetectionResultMsg =
  yolov8_ros::msg::DetectionResultMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolov8_ros

#endif  // YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__STRUCT_HPP_
