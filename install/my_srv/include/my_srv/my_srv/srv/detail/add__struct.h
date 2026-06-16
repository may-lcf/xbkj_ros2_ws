// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_srv:srv/Add.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_srv/srv/add.h"


#ifndef MY_SRV__SRV__DETAIL__ADD__STRUCT_H_
#define MY_SRV__SRV__DETAIL__ADD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Add in the package my_srv.
typedef struct my_srv__srv__Add_Request
{
  rosidl_runtime_c__String color;
  int32_t low_h;
  int32_t low_s;
  int32_t low_v;
  int32_t high_h;
  int32_t high_s;
  int32_t high_v;
} my_srv__srv__Add_Request;

// Struct for a sequence of my_srv__srv__Add_Request.
typedef struct my_srv__srv__Add_Request__Sequence
{
  my_srv__srv__Add_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_srv__srv__Add_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Add in the package my_srv.
typedef struct my_srv__srv__Add_Response
{
  bool success;
  rosidl_runtime_c__String message;
} my_srv__srv__Add_Response;

// Struct for a sequence of my_srv__srv__Add_Response.
typedef struct my_srv__srv__Add_Response__Sequence
{
  my_srv__srv__Add_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_srv__srv__Add_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  my_srv__srv__Add_Event__request__MAX_SIZE = 1
};
// response
enum
{
  my_srv__srv__Add_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Add in the package my_srv.
typedef struct my_srv__srv__Add_Event
{
  service_msgs__msg__ServiceEventInfo info;
  my_srv__srv__Add_Request__Sequence request;
  my_srv__srv__Add_Response__Sequence response;
} my_srv__srv__Add_Event;

// Struct for a sequence of my_srv__srv__Add_Event.
typedef struct my_srv__srv__Add_Event__Sequence
{
  my_srv__srv__Add_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_srv__srv__Add_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_SRV__SRV__DETAIL__ADD__STRUCT_H_
