// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_srv:srv/Add.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_srv/srv/detail/add__rosidl_typesupport_introspection_c.h"
#include "my_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_srv/srv/detail/add__functions.h"
#include "my_srv/srv/detail/add__struct.h"


// Include directives for member types
// Member `color`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_srv__srv__Add_Request__init(message_memory);
}

void my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_fini_function(void * message_memory)
{
  my_srv__srv__Add_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_member_array[7] = {
  {
    "color",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Request, color),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "low_h",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Request, low_h),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "low_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Request, low_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "low_v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Request, low_v),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "high_h",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Request, high_h),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "high_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Request, high_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "high_v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Request, high_v),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_members = {
  "my_srv__srv",  // message namespace
  "Add_Request",  // message name
  7,  // number of fields
  sizeof(my_srv__srv__Add_Request),
  false,  // has_any_key_member_
  my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_member_array,  // message members
  my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_type_support_handle = {
  0,
  &my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_members,
  get_message_typesupport_handle_function,
  &my_srv__srv__Add_Request__get_type_hash,
  &my_srv__srv__Add_Request__get_type_description,
  &my_srv__srv__Add_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Request)() {
  if (!my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_type_support_handle.typesupport_identifier) {
    my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "my_srv/srv/detail/add__rosidl_typesupport_introspection_c.h"
// already included above
// #include "my_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "my_srv/srv/detail/add__functions.h"
// already included above
// #include "my_srv/srv/detail/add__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_srv__srv__Add_Response__init(message_memory);
}

void my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_fini_function(void * message_memory)
{
  my_srv__srv__Add_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_members = {
  "my_srv__srv",  // message namespace
  "Add_Response",  // message name
  2,  // number of fields
  sizeof(my_srv__srv__Add_Response),
  false,  // has_any_key_member_
  my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_member_array,  // message members
  my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_type_support_handle = {
  0,
  &my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_members,
  get_message_typesupport_handle_function,
  &my_srv__srv__Add_Response__get_type_hash,
  &my_srv__srv__Add_Response__get_type_description,
  &my_srv__srv__Add_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Response)() {
  if (!my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_type_support_handle.typesupport_identifier) {
    my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "my_srv/srv/detail/add__rosidl_typesupport_introspection_c.h"
// already included above
// #include "my_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "my_srv/srv/detail/add__functions.h"
// already included above
// #include "my_srv/srv/detail/add__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "my_srv/srv/add.h"
// Member `request`
// Member `response`
// already included above
// #include "my_srv/srv/detail/add__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_srv__srv__Add_Event__init(message_memory);
}

void my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_fini_function(void * message_memory)
{
  my_srv__srv__Add_Event__fini(message_memory);
}

size_t my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__size_function__Add_Event__request(
  const void * untyped_member)
{
  const my_srv__srv__Add_Request__Sequence * member =
    (const my_srv__srv__Add_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_const_function__Add_Event__request(
  const void * untyped_member, size_t index)
{
  const my_srv__srv__Add_Request__Sequence * member =
    (const my_srv__srv__Add_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_function__Add_Event__request(
  void * untyped_member, size_t index)
{
  my_srv__srv__Add_Request__Sequence * member =
    (my_srv__srv__Add_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__fetch_function__Add_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const my_srv__srv__Add_Request * item =
    ((const my_srv__srv__Add_Request *)
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_const_function__Add_Event__request(untyped_member, index));
  my_srv__srv__Add_Request * value =
    (my_srv__srv__Add_Request *)(untyped_value);
  *value = *item;
}

void my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__assign_function__Add_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  my_srv__srv__Add_Request * item =
    ((my_srv__srv__Add_Request *)
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_function__Add_Event__request(untyped_member, index));
  const my_srv__srv__Add_Request * value =
    (const my_srv__srv__Add_Request *)(untyped_value);
  *item = *value;
}

bool my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__resize_function__Add_Event__request(
  void * untyped_member, size_t size)
{
  my_srv__srv__Add_Request__Sequence * member =
    (my_srv__srv__Add_Request__Sequence *)(untyped_member);
  my_srv__srv__Add_Request__Sequence__fini(member);
  return my_srv__srv__Add_Request__Sequence__init(member, size);
}

size_t my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__size_function__Add_Event__response(
  const void * untyped_member)
{
  const my_srv__srv__Add_Response__Sequence * member =
    (const my_srv__srv__Add_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_const_function__Add_Event__response(
  const void * untyped_member, size_t index)
{
  const my_srv__srv__Add_Response__Sequence * member =
    (const my_srv__srv__Add_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_function__Add_Event__response(
  void * untyped_member, size_t index)
{
  my_srv__srv__Add_Response__Sequence * member =
    (my_srv__srv__Add_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__fetch_function__Add_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const my_srv__srv__Add_Response * item =
    ((const my_srv__srv__Add_Response *)
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_const_function__Add_Event__response(untyped_member, index));
  my_srv__srv__Add_Response * value =
    (my_srv__srv__Add_Response *)(untyped_value);
  *value = *item;
}

void my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__assign_function__Add_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  my_srv__srv__Add_Response * item =
    ((my_srv__srv__Add_Response *)
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_function__Add_Event__response(untyped_member, index));
  const my_srv__srv__Add_Response * value =
    (const my_srv__srv__Add_Response *)(untyped_value);
  *item = *value;
}

bool my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__resize_function__Add_Event__response(
  void * untyped_member, size_t size)
{
  my_srv__srv__Add_Response__Sequence * member =
    (my_srv__srv__Add_Response__Sequence *)(untyped_member);
  my_srv__srv__Add_Response__Sequence__fini(member);
  return my_srv__srv__Add_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_srv__srv__Add_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(my_srv__srv__Add_Event, request),  // bytes offset in struct
    NULL,  // default value
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__size_function__Add_Event__request,  // size() function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_const_function__Add_Event__request,  // get_const(index) function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_function__Add_Event__request,  // get(index) function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__fetch_function__Add_Event__request,  // fetch(index, &value) function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__assign_function__Add_Event__request,  // assign(index, value) function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__resize_function__Add_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(my_srv__srv__Add_Event, response),  // bytes offset in struct
    NULL,  // default value
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__size_function__Add_Event__response,  // size() function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_const_function__Add_Event__response,  // get_const(index) function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__get_function__Add_Event__response,  // get(index) function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__fetch_function__Add_Event__response,  // fetch(index, &value) function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__assign_function__Add_Event__response,  // assign(index, value) function pointer
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__resize_function__Add_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_members = {
  "my_srv__srv",  // message namespace
  "Add_Event",  // message name
  3,  // number of fields
  sizeof(my_srv__srv__Add_Event),
  false,  // has_any_key_member_
  my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_member_array,  // message members
  my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_type_support_handle = {
  0,
  &my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_members,
  get_message_typesupport_handle_function,
  &my_srv__srv__Add_Event__get_type_hash,
  &my_srv__srv__Add_Event__get_type_description,
  &my_srv__srv__Add_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Event)() {
  my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Request)();
  my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Response)();
  if (!my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_type_support_handle.typesupport_identifier) {
    my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "my_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "my_srv/srv/detail/add__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_service_members = {
  "my_srv__srv",  // service namespace
  "Add",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_Request_message_type_support_handle,
  NULL,  // response message
  // my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_Response_message_type_support_handle
  NULL  // event_message
  // my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_Response_message_type_support_handle
};


static rosidl_service_type_support_t my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_service_type_support_handle = {
  0,
  &my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_service_members,
  get_service_typesupport_handle_function,
  &my_srv__srv__Add_Request__rosidl_typesupport_introspection_c__Add_Request_message_type_support_handle,
  &my_srv__srv__Add_Response__rosidl_typesupport_introspection_c__Add_Response_message_type_support_handle,
  &my_srv__srv__Add_Event__rosidl_typesupport_introspection_c__Add_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    my_srv,
    srv,
    Add
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    my_srv,
    srv,
    Add
  ),
  &my_srv__srv__Add__get_type_hash,
  &my_srv__srv__Add__get_type_description,
  &my_srv__srv__Add__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_srv
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add)(void) {
  if (!my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_service_type_support_handle.typesupport_identifier) {
    my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_srv, srv, Add_Event)()->data;
  }

  return &my_srv__srv__detail__add__rosidl_typesupport_introspection_c__Add_service_type_support_handle;
}
