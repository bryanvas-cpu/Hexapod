// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hexapod_interfaces:msg/PointArray.idl
// generated code does not contain a copyright notice
#include "hexapod_interfaces/msg/detail/point_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `points`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
hexapod_interfaces__msg__PointArray__init(hexapod_interfaces__msg__PointArray * msg)
{
  if (!msg) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->points, 0)) {
    hexapod_interfaces__msg__PointArray__fini(msg);
    return false;
  }
  return true;
}

void
hexapod_interfaces__msg__PointArray__fini(hexapod_interfaces__msg__PointArray * msg)
{
  if (!msg) {
    return;
  }
  // points
  geometry_msgs__msg__Point__Sequence__fini(&msg->points);
}

bool
hexapod_interfaces__msg__PointArray__are_equal(const hexapod_interfaces__msg__PointArray * lhs, const hexapod_interfaces__msg__PointArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  return true;
}

bool
hexapod_interfaces__msg__PointArray__copy(
  const hexapod_interfaces__msg__PointArray * input,
  hexapod_interfaces__msg__PointArray * output)
{
  if (!input || !output) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  return true;
}

hexapod_interfaces__msg__PointArray *
hexapod_interfaces__msg__PointArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__PointArray * msg = (hexapod_interfaces__msg__PointArray *)allocator.allocate(sizeof(hexapod_interfaces__msg__PointArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hexapod_interfaces__msg__PointArray));
  bool success = hexapod_interfaces__msg__PointArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hexapod_interfaces__msg__PointArray__destroy(hexapod_interfaces__msg__PointArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hexapod_interfaces__msg__PointArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hexapod_interfaces__msg__PointArray__Sequence__init(hexapod_interfaces__msg__PointArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__PointArray * data = NULL;

  if (size) {
    data = (hexapod_interfaces__msg__PointArray *)allocator.zero_allocate(size, sizeof(hexapod_interfaces__msg__PointArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hexapod_interfaces__msg__PointArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hexapod_interfaces__msg__PointArray__fini(&data[i - 1]);
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
hexapod_interfaces__msg__PointArray__Sequence__fini(hexapod_interfaces__msg__PointArray__Sequence * array)
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
      hexapod_interfaces__msg__PointArray__fini(&array->data[i]);
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

hexapod_interfaces__msg__PointArray__Sequence *
hexapod_interfaces__msg__PointArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__PointArray__Sequence * array = (hexapod_interfaces__msg__PointArray__Sequence *)allocator.allocate(sizeof(hexapod_interfaces__msg__PointArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hexapod_interfaces__msg__PointArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hexapod_interfaces__msg__PointArray__Sequence__destroy(hexapod_interfaces__msg__PointArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hexapod_interfaces__msg__PointArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hexapod_interfaces__msg__PointArray__Sequence__are_equal(const hexapod_interfaces__msg__PointArray__Sequence * lhs, const hexapod_interfaces__msg__PointArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hexapod_interfaces__msg__PointArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hexapod_interfaces__msg__PointArray__Sequence__copy(
  const hexapod_interfaces__msg__PointArray__Sequence * input,
  hexapod_interfaces__msg__PointArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hexapod_interfaces__msg__PointArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hexapod_interfaces__msg__PointArray * data =
      (hexapod_interfaces__msg__PointArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hexapod_interfaces__msg__PointArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hexapod_interfaces__msg__PointArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hexapod_interfaces__msg__PointArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
