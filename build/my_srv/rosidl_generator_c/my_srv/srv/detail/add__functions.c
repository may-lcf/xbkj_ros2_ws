// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_srv:srv/Add.idl
// generated code does not contain a copyright notice
#include "my_srv/srv/detail/add__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `color`
#include "rosidl_runtime_c/string_functions.h"

bool
my_srv__srv__Add_Request__init(my_srv__srv__Add_Request * msg)
{
  if (!msg) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__init(&msg->color)) {
    my_srv__srv__Add_Request__fini(msg);
    return false;
  }
  // low_h
  // low_s
  // low_v
  // high_h
  // high_s
  // high_v
  return true;
}

void
my_srv__srv__Add_Request__fini(my_srv__srv__Add_Request * msg)
{
  if (!msg) {
    return;
  }
  // color
  rosidl_runtime_c__String__fini(&msg->color);
  // low_h
  // low_s
  // low_v
  // high_h
  // high_s
  // high_v
}

bool
my_srv__srv__Add_Request__are_equal(const my_srv__srv__Add_Request * lhs, const my_srv__srv__Add_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  // low_h
  if (lhs->low_h != rhs->low_h) {
    return false;
  }
  // low_s
  if (lhs->low_s != rhs->low_s) {
    return false;
  }
  // low_v
  if (lhs->low_v != rhs->low_v) {
    return false;
  }
  // high_h
  if (lhs->high_h != rhs->high_h) {
    return false;
  }
  // high_s
  if (lhs->high_s != rhs->high_s) {
    return false;
  }
  // high_v
  if (lhs->high_v != rhs->high_v) {
    return false;
  }
  return true;
}

bool
my_srv__srv__Add_Request__copy(
  const my_srv__srv__Add_Request * input,
  my_srv__srv__Add_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  // low_h
  output->low_h = input->low_h;
  // low_s
  output->low_s = input->low_s;
  // low_v
  output->low_v = input->low_v;
  // high_h
  output->high_h = input->high_h;
  // high_s
  output->high_s = input->high_s;
  // high_v
  output->high_v = input->high_v;
  return true;
}

my_srv__srv__Add_Request *
my_srv__srv__Add_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Request * msg = (my_srv__srv__Add_Request *)allocator.allocate(sizeof(my_srv__srv__Add_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_srv__srv__Add_Request));
  bool success = my_srv__srv__Add_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_srv__srv__Add_Request__destroy(my_srv__srv__Add_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_srv__srv__Add_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_srv__srv__Add_Request__Sequence__init(my_srv__srv__Add_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Request * data = NULL;

  if (size) {
    data = (my_srv__srv__Add_Request *)allocator.zero_allocate(size, sizeof(my_srv__srv__Add_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_srv__srv__Add_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_srv__srv__Add_Request__fini(&data[i - 1]);
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
my_srv__srv__Add_Request__Sequence__fini(my_srv__srv__Add_Request__Sequence * array)
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
      my_srv__srv__Add_Request__fini(&array->data[i]);
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

my_srv__srv__Add_Request__Sequence *
my_srv__srv__Add_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Request__Sequence * array = (my_srv__srv__Add_Request__Sequence *)allocator.allocate(sizeof(my_srv__srv__Add_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_srv__srv__Add_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_srv__srv__Add_Request__Sequence__destroy(my_srv__srv__Add_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_srv__srv__Add_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_srv__srv__Add_Request__Sequence__are_equal(const my_srv__srv__Add_Request__Sequence * lhs, const my_srv__srv__Add_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_srv__srv__Add_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_srv__srv__Add_Request__Sequence__copy(
  const my_srv__srv__Add_Request__Sequence * input,
  my_srv__srv__Add_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_srv__srv__Add_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_srv__srv__Add_Request * data =
      (my_srv__srv__Add_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_srv__srv__Add_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_srv__srv__Add_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_srv__srv__Add_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
my_srv__srv__Add_Response__init(my_srv__srv__Add_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    my_srv__srv__Add_Response__fini(msg);
    return false;
  }
  return true;
}

void
my_srv__srv__Add_Response__fini(my_srv__srv__Add_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
my_srv__srv__Add_Response__are_equal(const my_srv__srv__Add_Response * lhs, const my_srv__srv__Add_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
my_srv__srv__Add_Response__copy(
  const my_srv__srv__Add_Response * input,
  my_srv__srv__Add_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

my_srv__srv__Add_Response *
my_srv__srv__Add_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Response * msg = (my_srv__srv__Add_Response *)allocator.allocate(sizeof(my_srv__srv__Add_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_srv__srv__Add_Response));
  bool success = my_srv__srv__Add_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_srv__srv__Add_Response__destroy(my_srv__srv__Add_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_srv__srv__Add_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_srv__srv__Add_Response__Sequence__init(my_srv__srv__Add_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Response * data = NULL;

  if (size) {
    data = (my_srv__srv__Add_Response *)allocator.zero_allocate(size, sizeof(my_srv__srv__Add_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_srv__srv__Add_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_srv__srv__Add_Response__fini(&data[i - 1]);
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
my_srv__srv__Add_Response__Sequence__fini(my_srv__srv__Add_Response__Sequence * array)
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
      my_srv__srv__Add_Response__fini(&array->data[i]);
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

my_srv__srv__Add_Response__Sequence *
my_srv__srv__Add_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Response__Sequence * array = (my_srv__srv__Add_Response__Sequence *)allocator.allocate(sizeof(my_srv__srv__Add_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_srv__srv__Add_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_srv__srv__Add_Response__Sequence__destroy(my_srv__srv__Add_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_srv__srv__Add_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_srv__srv__Add_Response__Sequence__are_equal(const my_srv__srv__Add_Response__Sequence * lhs, const my_srv__srv__Add_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_srv__srv__Add_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_srv__srv__Add_Response__Sequence__copy(
  const my_srv__srv__Add_Response__Sequence * input,
  my_srv__srv__Add_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_srv__srv__Add_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_srv__srv__Add_Response * data =
      (my_srv__srv__Add_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_srv__srv__Add_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_srv__srv__Add_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_srv__srv__Add_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "my_srv/srv/detail/add__functions.h"

bool
my_srv__srv__Add_Event__init(my_srv__srv__Add_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    my_srv__srv__Add_Event__fini(msg);
    return false;
  }
  // request
  if (!my_srv__srv__Add_Request__Sequence__init(&msg->request, 0)) {
    my_srv__srv__Add_Event__fini(msg);
    return false;
  }
  // response
  if (!my_srv__srv__Add_Response__Sequence__init(&msg->response, 0)) {
    my_srv__srv__Add_Event__fini(msg);
    return false;
  }
  return true;
}

void
my_srv__srv__Add_Event__fini(my_srv__srv__Add_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  my_srv__srv__Add_Request__Sequence__fini(&msg->request);
  // response
  my_srv__srv__Add_Response__Sequence__fini(&msg->response);
}

bool
my_srv__srv__Add_Event__are_equal(const my_srv__srv__Add_Event * lhs, const my_srv__srv__Add_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!my_srv__srv__Add_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!my_srv__srv__Add_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
my_srv__srv__Add_Event__copy(
  const my_srv__srv__Add_Event * input,
  my_srv__srv__Add_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!my_srv__srv__Add_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!my_srv__srv__Add_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

my_srv__srv__Add_Event *
my_srv__srv__Add_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Event * msg = (my_srv__srv__Add_Event *)allocator.allocate(sizeof(my_srv__srv__Add_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_srv__srv__Add_Event));
  bool success = my_srv__srv__Add_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_srv__srv__Add_Event__destroy(my_srv__srv__Add_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_srv__srv__Add_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_srv__srv__Add_Event__Sequence__init(my_srv__srv__Add_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Event * data = NULL;

  if (size) {
    data = (my_srv__srv__Add_Event *)allocator.zero_allocate(size, sizeof(my_srv__srv__Add_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_srv__srv__Add_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_srv__srv__Add_Event__fini(&data[i - 1]);
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
my_srv__srv__Add_Event__Sequence__fini(my_srv__srv__Add_Event__Sequence * array)
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
      my_srv__srv__Add_Event__fini(&array->data[i]);
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

my_srv__srv__Add_Event__Sequence *
my_srv__srv__Add_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_srv__srv__Add_Event__Sequence * array = (my_srv__srv__Add_Event__Sequence *)allocator.allocate(sizeof(my_srv__srv__Add_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_srv__srv__Add_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_srv__srv__Add_Event__Sequence__destroy(my_srv__srv__Add_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_srv__srv__Add_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_srv__srv__Add_Event__Sequence__are_equal(const my_srv__srv__Add_Event__Sequence * lhs, const my_srv__srv__Add_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_srv__srv__Add_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_srv__srv__Add_Event__Sequence__copy(
  const my_srv__srv__Add_Event__Sequence * input,
  my_srv__srv__Add_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_srv__srv__Add_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_srv__srv__Add_Event * data =
      (my_srv__srv__Add_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_srv__srv__Add_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_srv__srv__Add_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_srv__srv__Add_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
