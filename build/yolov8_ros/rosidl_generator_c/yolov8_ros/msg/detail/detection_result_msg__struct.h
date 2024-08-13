// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolov8_ros:msg/DetectionResultMsg.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__STRUCT_H_
#define YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'objects'
#include "yolov8_ros/msg/detail/detected_object_msg__struct.h"

// Struct defined in msg/DetectionResultMsg in the package yolov8_ros.
typedef struct yolov8_ros__msg__DetectionResultMsg
{
  yolov8_ros__msg__DetectedObjectMsg__Sequence objects;
} yolov8_ros__msg__DetectionResultMsg;

// Struct for a sequence of yolov8_ros__msg__DetectionResultMsg.
typedef struct yolov8_ros__msg__DetectionResultMsg__Sequence
{
  yolov8_ros__msg__DetectionResultMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolov8_ros__msg__DetectionResultMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__STRUCT_H_
