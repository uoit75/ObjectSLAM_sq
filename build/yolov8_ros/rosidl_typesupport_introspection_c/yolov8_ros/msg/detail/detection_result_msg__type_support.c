// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolov8_ros:msg/DetectionResultMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolov8_ros/msg/detail/detection_result_msg__rosidl_typesupport_introspection_c.h"
#include "yolov8_ros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolov8_ros/msg/detail/detection_result_msg__functions.h"
#include "yolov8_ros/msg/detail/detection_result_msg__struct.h"


// Include directives for member types
// Member `objects`
#include "yolov8_ros/msg/detected_object_msg.h"
// Member `objects`
#include "yolov8_ros/msg/detail/detected_object_msg__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolov8_ros__msg__DetectionResultMsg__init(message_memory);
}

void DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_fini_function(void * message_memory)
{
  yolov8_ros__msg__DetectionResultMsg__fini(message_memory);
}

size_t DetectionResultMsg__rosidl_typesupport_introspection_c__size_function__DetectedObjectMsg__objects(
  const void * untyped_member)
{
  const yolov8_ros__msg__DetectedObjectMsg__Sequence * member =
    (const yolov8_ros__msg__DetectedObjectMsg__Sequence *)(untyped_member);
  return member->size;
}

const void * DetectionResultMsg__rosidl_typesupport_introspection_c__get_const_function__DetectedObjectMsg__objects(
  const void * untyped_member, size_t index)
{
  const yolov8_ros__msg__DetectedObjectMsg__Sequence * member =
    (const yolov8_ros__msg__DetectedObjectMsg__Sequence *)(untyped_member);
  return &member->data[index];
}

void * DetectionResultMsg__rosidl_typesupport_introspection_c__get_function__DetectedObjectMsg__objects(
  void * untyped_member, size_t index)
{
  yolov8_ros__msg__DetectedObjectMsg__Sequence * member =
    (yolov8_ros__msg__DetectedObjectMsg__Sequence *)(untyped_member);
  return &member->data[index];
}

bool DetectionResultMsg__rosidl_typesupport_introspection_c__resize_function__DetectedObjectMsg__objects(
  void * untyped_member, size_t size)
{
  yolov8_ros__msg__DetectedObjectMsg__Sequence * member =
    (yolov8_ros__msg__DetectedObjectMsg__Sequence *)(untyped_member);
  yolov8_ros__msg__DetectedObjectMsg__Sequence__fini(member);
  return yolov8_ros__msg__DetectedObjectMsg__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_member_array[1] = {
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolov8_ros__msg__DetectionResultMsg, objects),  // bytes offset in struct
    NULL,  // default value
    DetectionResultMsg__rosidl_typesupport_introspection_c__size_function__DetectedObjectMsg__objects,  // size() function pointer
    DetectionResultMsg__rosidl_typesupport_introspection_c__get_const_function__DetectedObjectMsg__objects,  // get_const(index) function pointer
    DetectionResultMsg__rosidl_typesupport_introspection_c__get_function__DetectedObjectMsg__objects,  // get(index) function pointer
    DetectionResultMsg__rosidl_typesupport_introspection_c__resize_function__DetectedObjectMsg__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_members = {
  "yolov8_ros__msg",  // message namespace
  "DetectionResultMsg",  // message name
  1,  // number of fields
  sizeof(yolov8_ros__msg__DetectionResultMsg),
  DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_member_array,  // message members
  DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_type_support_handle = {
  0,
  &DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolov8_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolov8_ros, msg, DetectionResultMsg)() {
  DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolov8_ros, msg, DetectedObjectMsg)();
  if (!DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_type_support_handle.typesupport_identifier) {
    DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DetectionResultMsg__rosidl_typesupport_introspection_c__DetectionResultMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
