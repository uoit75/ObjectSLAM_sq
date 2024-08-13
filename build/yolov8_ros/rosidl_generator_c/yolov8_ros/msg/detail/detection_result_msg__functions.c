// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolov8_ros:msg/DetectionResultMsg.idl
// generated code does not contain a copyright notice
#include "yolov8_ros/msg/detail/detection_result_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `objects`
#include "yolov8_ros/msg/detail/detected_object_msg__functions.h"

bool
yolov8_ros__msg__DetectionResultMsg__init(yolov8_ros__msg__DetectionResultMsg * msg)
{
  if (!msg) {
    return false;
  }
  // objects
  if (!yolov8_ros__msg__DetectedObjectMsg__Sequence__init(&msg->objects, 0)) {
    yolov8_ros__msg__DetectionResultMsg__fini(msg);
    return false;
  }
  return true;
}

void
yolov8_ros__msg__DetectionResultMsg__fini(yolov8_ros__msg__DetectionResultMsg * msg)
{
  if (!msg) {
    return;
  }
  // objects
  yolov8_ros__msg__DetectedObjectMsg__Sequence__fini(&msg->objects);
}

bool
yolov8_ros__msg__DetectionResultMsg__are_equal(const yolov8_ros__msg__DetectionResultMsg * lhs, const yolov8_ros__msg__DetectionResultMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // objects
  if (!yolov8_ros__msg__DetectedObjectMsg__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  return true;
}

bool
yolov8_ros__msg__DetectionResultMsg__copy(
  const yolov8_ros__msg__DetectionResultMsg * input,
  yolov8_ros__msg__DetectionResultMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // objects
  if (!yolov8_ros__msg__DetectedObjectMsg__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  return true;
}

yolov8_ros__msg__DetectionResultMsg *
yolov8_ros__msg__DetectionResultMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov8_ros__msg__DetectionResultMsg * msg = (yolov8_ros__msg__DetectionResultMsg *)allocator.allocate(sizeof(yolov8_ros__msg__DetectionResultMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolov8_ros__msg__DetectionResultMsg));
  bool success = yolov8_ros__msg__DetectionResultMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolov8_ros__msg__DetectionResultMsg__destroy(yolov8_ros__msg__DetectionResultMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolov8_ros__msg__DetectionResultMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolov8_ros__msg__DetectionResultMsg__Sequence__init(yolov8_ros__msg__DetectionResultMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov8_ros__msg__DetectionResultMsg * data = NULL;

  if (size) {
    data = (yolov8_ros__msg__DetectionResultMsg *)allocator.zero_allocate(size, sizeof(yolov8_ros__msg__DetectionResultMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolov8_ros__msg__DetectionResultMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolov8_ros__msg__DetectionResultMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
yolov8_ros__msg__DetectionResultMsg__Sequence__fini(yolov8_ros__msg__DetectionResultMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      yolov8_ros__msg__DetectionResultMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

yolov8_ros__msg__DetectionResultMsg__Sequence *
yolov8_ros__msg__DetectionResultMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov8_ros__msg__DetectionResultMsg__Sequence * array = (yolov8_ros__msg__DetectionResultMsg__Sequence *)allocator.allocate(sizeof(yolov8_ros__msg__DetectionResultMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolov8_ros__msg__DetectionResultMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolov8_ros__msg__DetectionResultMsg__Sequence__destroy(yolov8_ros__msg__DetectionResultMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolov8_ros__msg__DetectionResultMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolov8_ros__msg__DetectionResultMsg__Sequence__are_equal(const yolov8_ros__msg__DetectionResultMsg__Sequence * lhs, const yolov8_ros__msg__DetectionResultMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolov8_ros__msg__DetectionResultMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolov8_ros__msg__DetectionResultMsg__Sequence__copy(
  const yolov8_ros__msg__DetectionResultMsg__Sequence * input,
  yolov8_ros__msg__DetectionResultMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolov8_ros__msg__DetectionResultMsg);
    yolov8_ros__msg__DetectionResultMsg * data =
      (yolov8_ros__msg__DetectionResultMsg *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolov8_ros__msg__DetectionResultMsg__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          yolov8_ros__msg__DetectionResultMsg__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolov8_ros__msg__DetectionResultMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
