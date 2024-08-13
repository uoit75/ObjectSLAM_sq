// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from yolov8_ros:msg/DetectionResultMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "yolov8_ros/msg/detail/detection_result_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace yolov8_ros
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DetectionResultMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) yolov8_ros::msg::DetectionResultMsg(_init);
}

void DetectionResultMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<yolov8_ros::msg::DetectionResultMsg *>(message_memory);
  typed_message->~DetectionResultMsg();
}

size_t size_function__DetectionResultMsg__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<yolov8_ros::msg::DetectedObjectMsg> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectionResultMsg__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<yolov8_ros::msg::DetectedObjectMsg> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectionResultMsg__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<yolov8_ros::msg::DetectedObjectMsg> *>(untyped_member);
  return &member[index];
}

void resize_function__DetectionResultMsg__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<yolov8_ros::msg::DetectedObjectMsg> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DetectionResultMsg_message_member_array[1] = {
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<yolov8_ros::msg::DetectedObjectMsg>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolov8_ros::msg::DetectionResultMsg, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectionResultMsg__objects,  // size() function pointer
    get_const_function__DetectionResultMsg__objects,  // get_const(index) function pointer
    get_function__DetectionResultMsg__objects,  // get(index) function pointer
    resize_function__DetectionResultMsg__objects  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DetectionResultMsg_message_members = {
  "yolov8_ros::msg",  // message namespace
  "DetectionResultMsg",  // message name
  1,  // number of fields
  sizeof(yolov8_ros::msg::DetectionResultMsg),
  DetectionResultMsg_message_member_array,  // message members
  DetectionResultMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectionResultMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DetectionResultMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectionResultMsg_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace yolov8_ros


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<yolov8_ros::msg::DetectionResultMsg>()
{
  return &::yolov8_ros::msg::rosidl_typesupport_introspection_cpp::DetectionResultMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, yolov8_ros, msg, DetectionResultMsg)() {
  return &::yolov8_ros::msg::rosidl_typesupport_introspection_cpp::DetectionResultMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
