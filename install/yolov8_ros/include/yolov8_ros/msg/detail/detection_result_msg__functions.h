// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from yolov8_ros:msg/DetectionResultMsg.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__FUNCTIONS_H_
#define YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "yolov8_ros/msg/rosidl_generator_c__visibility_control.h"

#include "yolov8_ros/msg/detail/detection_result_msg__struct.h"

/// Initialize msg/DetectionResultMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * yolov8_ros__msg__DetectionResultMsg
 * )) before or use
 * yolov8_ros__msg__DetectionResultMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
bool
yolov8_ros__msg__DetectionResultMsg__init(yolov8_ros__msg__DetectionResultMsg * msg);

/// Finalize msg/DetectionResultMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
void
yolov8_ros__msg__DetectionResultMsg__fini(yolov8_ros__msg__DetectionResultMsg * msg);

/// Create msg/DetectionResultMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * yolov8_ros__msg__DetectionResultMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
yolov8_ros__msg__DetectionResultMsg *
yolov8_ros__msg__DetectionResultMsg__create();

/// Destroy msg/DetectionResultMsg message.
/**
 * It calls
 * yolov8_ros__msg__DetectionResultMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
void
yolov8_ros__msg__DetectionResultMsg__destroy(yolov8_ros__msg__DetectionResultMsg * msg);

/// Check for msg/DetectionResultMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
bool
yolov8_ros__msg__DetectionResultMsg__are_equal(const yolov8_ros__msg__DetectionResultMsg * lhs, const yolov8_ros__msg__DetectionResultMsg * rhs);

/// Copy a msg/DetectionResultMsg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
bool
yolov8_ros__msg__DetectionResultMsg__copy(
  const yolov8_ros__msg__DetectionResultMsg * input,
  yolov8_ros__msg__DetectionResultMsg * output);

/// Initialize array of msg/DetectionResultMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * yolov8_ros__msg__DetectionResultMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
bool
yolov8_ros__msg__DetectionResultMsg__Sequence__init(yolov8_ros__msg__DetectionResultMsg__Sequence * array, size_t size);

/// Finalize array of msg/DetectionResultMsg messages.
/**
 * It calls
 * yolov8_ros__msg__DetectionResultMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
void
yolov8_ros__msg__DetectionResultMsg__Sequence__fini(yolov8_ros__msg__DetectionResultMsg__Sequence * array);

/// Create array of msg/DetectionResultMsg messages.
/**
 * It allocates the memory for the array and calls
 * yolov8_ros__msg__DetectionResultMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
yolov8_ros__msg__DetectionResultMsg__Sequence *
yolov8_ros__msg__DetectionResultMsg__Sequence__create(size_t size);

/// Destroy array of msg/DetectionResultMsg messages.
/**
 * It calls
 * yolov8_ros__msg__DetectionResultMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
void
yolov8_ros__msg__DetectionResultMsg__Sequence__destroy(yolov8_ros__msg__DetectionResultMsg__Sequence * array);

/// Check for msg/DetectionResultMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
bool
yolov8_ros__msg__DetectionResultMsg__Sequence__are_equal(const yolov8_ros__msg__DetectionResultMsg__Sequence * lhs, const yolov8_ros__msg__DetectionResultMsg__Sequence * rhs);

/// Copy an array of msg/DetectionResultMsg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov8_ros
bool
yolov8_ros__msg__DetectionResultMsg__Sequence__copy(
  const yolov8_ros__msg__DetectionResultMsg__Sequence * input,
  yolov8_ros__msg__DetectionResultMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // YOLOV8_ROS__MSG__DETAIL__DETECTION_RESULT_MSG__FUNCTIONS_H_
