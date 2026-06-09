// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_srv:srv/Add.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_srv/srv/add.hpp"


#ifndef MY_SRV__SRV__DETAIL__ADD__STRUCT_HPP_
#define MY_SRV__SRV__DETAIL__ADD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_srv__srv__Add_Request __attribute__((deprecated))
#else
# define DEPRECATED__my_srv__srv__Add_Request __declspec(deprecated)
#endif

namespace my_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Add_Request_
{
  using Type = Add_Request_<ContainerAllocator>;

  explicit Add_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color = "";
      this->low_h = 0l;
      this->low_s = 0l;
      this->low_v = 0l;
      this->high_h = 0l;
      this->high_s = 0l;
      this->high_v = 0l;
    }
  }

  explicit Add_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color = "";
      this->low_h = 0l;
      this->low_s = 0l;
      this->low_v = 0l;
      this->high_h = 0l;
      this->high_s = 0l;
      this->high_v = 0l;
    }
  }

  // field types and members
  using _color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _color_type color;
  using _low_h_type =
    int32_t;
  _low_h_type low_h;
  using _low_s_type =
    int32_t;
  _low_s_type low_s;
  using _low_v_type =
    int32_t;
  _low_v_type low_v;
  using _high_h_type =
    int32_t;
  _high_h_type high_h;
  using _high_s_type =
    int32_t;
  _high_s_type high_s;
  using _high_v_type =
    int32_t;
  _high_v_type high_v;

  // setters for named parameter idiom
  Type & set__color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__low_h(
    const int32_t & _arg)
  {
    this->low_h = _arg;
    return *this;
  }
  Type & set__low_s(
    const int32_t & _arg)
  {
    this->low_s = _arg;
    return *this;
  }
  Type & set__low_v(
    const int32_t & _arg)
  {
    this->low_v = _arg;
    return *this;
  }
  Type & set__high_h(
    const int32_t & _arg)
  {
    this->high_h = _arg;
    return *this;
  }
  Type & set__high_s(
    const int32_t & _arg)
  {
    this->high_s = _arg;
    return *this;
  }
  Type & set__high_v(
    const int32_t & _arg)
  {
    this->high_v = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_srv::srv::Add_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_srv::srv::Add_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_srv::srv::Add_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_srv::srv::Add_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_srv::srv::Add_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_srv::srv::Add_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_srv::srv::Add_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_srv::srv::Add_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_srv::srv::Add_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_srv::srv::Add_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_srv__srv__Add_Request
    std::shared_ptr<my_srv::srv::Add_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_srv__srv__Add_Request
    std::shared_ptr<my_srv::srv::Add_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Add_Request_ & other) const
  {
    if (this->color != other.color) {
      return false;
    }
    if (this->low_h != other.low_h) {
      return false;
    }
    if (this->low_s != other.low_s) {
      return false;
    }
    if (this->low_v != other.low_v) {
      return false;
    }
    if (this->high_h != other.high_h) {
      return false;
    }
    if (this->high_s != other.high_s) {
      return false;
    }
    if (this->high_v != other.high_v) {
      return false;
    }
    return true;
  }
  bool operator!=(const Add_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Add_Request_

// alias to use template instance with default allocator
using Add_Request =
  my_srv::srv::Add_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace my_srv


#ifndef _WIN32
# define DEPRECATED__my_srv__srv__Add_Response __attribute__((deprecated))
#else
# define DEPRECATED__my_srv__srv__Add_Response __declspec(deprecated)
#endif

namespace my_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Add_Response_
{
  using Type = Add_Response_<ContainerAllocator>;

  explicit Add_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Add_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_srv::srv::Add_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_srv::srv::Add_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_srv::srv::Add_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_srv::srv::Add_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_srv::srv::Add_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_srv::srv::Add_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_srv::srv::Add_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_srv::srv::Add_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_srv::srv::Add_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_srv::srv::Add_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_srv__srv__Add_Response
    std::shared_ptr<my_srv::srv::Add_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_srv__srv__Add_Response
    std::shared_ptr<my_srv::srv::Add_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Add_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Add_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Add_Response_

// alias to use template instance with default allocator
using Add_Response =
  my_srv::srv::Add_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace my_srv


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__my_srv__srv__Add_Event __attribute__((deprecated))
#else
# define DEPRECATED__my_srv__srv__Add_Event __declspec(deprecated)
#endif

namespace my_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Add_Event_
{
  using Type = Add_Event_<ContainerAllocator>;

  explicit Add_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Add_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<my_srv::srv::Add_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_srv::srv::Add_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<my_srv::srv::Add_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_srv::srv::Add_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<my_srv::srv::Add_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_srv::srv::Add_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<my_srv::srv::Add_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_srv::srv::Add_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_srv::srv::Add_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_srv::srv::Add_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_srv::srv::Add_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_srv::srv::Add_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_srv::srv::Add_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_srv::srv::Add_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_srv::srv::Add_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_srv::srv::Add_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_srv::srv::Add_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_srv::srv::Add_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_srv__srv__Add_Event
    std::shared_ptr<my_srv::srv::Add_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_srv__srv__Add_Event
    std::shared_ptr<my_srv::srv::Add_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Add_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Add_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Add_Event_

// alias to use template instance with default allocator
using Add_Event =
  my_srv::srv::Add_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace my_srv

namespace my_srv
{

namespace srv
{

struct Add
{
  using Request = my_srv::srv::Add_Request;
  using Response = my_srv::srv::Add_Response;
  using Event = my_srv::srv::Add_Event;
};

}  // namespace srv

}  // namespace my_srv

#endif  // MY_SRV__SRV__DETAIL__ADD__STRUCT_HPP_
