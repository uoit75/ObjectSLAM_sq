// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolov8_ros:msg/DetectedObjectMsg.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__STRUCT_H_
#define YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bbox_points'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'masks_instance'
#include "sensor_msgs/msg/detail/image__struct.h"

// Struct defined in msg/DetectedObjectMsg in the package yolov8_ros.
typedef struct yolov8_ros__msg__DetectedObjectMsg
{
  int32_t label;
  float confidence;
  geometry_msgs__msg__Point__Sequence bbox_points;
  sensor_msgs__msg__Image masks_instance;
} yolov8_ros__msg__DetectedObjectMsg;

// Struct for a sequence of yolov8_ros__msg__DetectedObjectMsg.
typedef struct yolov8_ros__msg__DetectedObjectMsg__Sequence
{
  yolov8_ros__msg__DetectedObjectMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolov8_ros__msg__DetectedObjectMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLOV8_ROS__MSG__DETAIL__DETECTED_OBJECT_MSG__STRUCT_H_
