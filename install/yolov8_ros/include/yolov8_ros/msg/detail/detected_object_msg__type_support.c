// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolov8_ros:msg/DetectedObjectMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolov8_ros/msg/detail/detected_object_msg__rosidl_typesupport_introspection_c.h"
#include "yolov8_ros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolov8_ros/msg/detail/detected_object_msg__functions.h"
#include "yolov8_ros/msg/detail/detected_object_msg__struct.h"


// Include directives for member types
// Member `bbox_points`
#include "geometry_msgs/msg/point.h"
// Member `bbox_points`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `masks_instance`
#include "sensor_msgs/msg/image.h"
// Member `masks_instance`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolov8_ros__msg__DetectedObjectMsg__init(message_memory);
}

void DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_fini_function(void * message_memory)
{
  yolov8_ros__msg__DetectedObjectMsg__fini(message_memory);
}

size_t DetectedObjectMsg__rosidl_typesupport_introspection_c__size_function__Point__bbox_points(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * DetectedObjectMsg__rosidl_typesupport_introspection_c__get_const_function__Point__bbox_points(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * DetectedObjectMsg__rosidl_typesupport_introspection_c__get_function__Point__bbox_points(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

bool DetectedObjectMsg__rosidl_typesupport_introspection_c__resize_function__Point__bbox_points(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_member_array[4] = {
  {
    "label",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolov8_ros__msg__DetectedObjectMsg, label),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolov8_ros__msg__DetectedObjectMsg, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox_points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolov8_ros__msg__DetectedObjectMsg, bbox_points),  // bytes offset in struct
    NULL,  // default value
    DetectedObjectMsg__rosidl_typesupport_introspection_c__size_function__Point__bbox_points,  // size() function pointer
    DetectedObjectMsg__rosidl_typesupport_introspection_c__get_const_function__Point__bbox_points,  // get_const(index) function pointer
    DetectedObjectMsg__rosidl_typesupport_introspection_c__get_function__Point__bbox_points,  // get(index) function pointer
    DetectedObjectMsg__rosidl_typesupport_introspection_c__resize_function__Point__bbox_points  // resize(index) function pointer
  },
  {
    "masks_instance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolov8_ros__msg__DetectedObjectMsg, masks_instance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_members = {
  "yolov8_ros__msg",  // message namespace
  "DetectedObjectMsg",  // message name
  4,  // number of fields
  sizeof(yolov8_ros__msg__DetectedObjectMsg),
  DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_member_array,  // message members
  DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_type_support_handle = {
  0,
  &DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolov8_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolov8_ros, msg, DetectedObjectMsg)() {
  DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_type_support_handle.typesupport_identifier) {
    DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DetectedObjectMsg__rosidl_typesupport_introspection_c__DetectedObjectMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
