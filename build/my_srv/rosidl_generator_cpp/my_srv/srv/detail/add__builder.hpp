// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_srv:srv/Add.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_srv/srv/add.hpp"


#ifndef MY_SRV__SRV__DETAIL__ADD__BUILDER_HPP_
#define MY_SRV__SRV__DETAIL__ADD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_srv/srv/detail/add__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_srv
{

namespace srv
{

namespace builder
{

class Init_Add_Request_high_v
{
public:
  explicit Init_Add_Request_high_v(::my_srv::srv::Add_Request & msg)
  : msg_(msg)
  {}
  ::my_srv::srv::Add_Request high_v(::my_srv::srv::Add_Request::_high_v_type arg)
  {
    msg_.high_v = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_srv::srv::Add_Request msg_;
};

class Init_Add_Request_high_s
{
public:
  explicit Init_Add_Request_high_s(::my_srv::srv::Add_Request & msg)
  : msg_(msg)
  {}
  Init_Add_Request_high_v high_s(::my_srv::srv::Add_Request::_high_s_type arg)
  {
    msg_.high_s = std::move(arg);
    return Init_Add_Request_high_v(msg_);
  }

private:
  ::my_srv::srv::Add_Request msg_;
};

class Init_Add_Request_high_h
{
public:
  explicit Init_Add_Request_high_h(::my_srv::srv::Add_Request & msg)
  : msg_(msg)
  {}
  Init_Add_Request_high_s high_h(::my_srv::srv::Add_Request::_high_h_type arg)
  {
    msg_.high_h = std::move(arg);
    return Init_Add_Request_high_s(msg_);
  }

private:
  ::my_srv::srv::Add_Request msg_;
};

class Init_Add_Request_low_v
{
public:
  explicit Init_Add_Request_low_v(::my_srv::srv::Add_Request & msg)
  : msg_(msg)
  {}
  Init_Add_Request_high_h low_v(::my_srv::srv::Add_Request::_low_v_type arg)
  {
    msg_.low_v = std::move(arg);
    return Init_Add_Request_high_h(msg_);
  }

private:
  ::my_srv::srv::Add_Request msg_;
};

class Init_Add_Request_low_s
{
public:
  explicit Init_Add_Request_low_s(::my_srv::srv::Add_Request & msg)
  : msg_(msg)
  {}
  Init_Add_Request_low_v low_s(::my_srv::srv::Add_Request::_low_s_type arg)
  {
    msg_.low_s = std::move(arg);
    return Init_Add_Request_low_v(msg_);
  }

private:
  ::my_srv::srv::Add_Request msg_;
};

class Init_Add_Request_low_h
{
public:
  explicit Init_Add_Request_low_h(::my_srv::srv::Add_Request & msg)
  : msg_(msg)
  {}
  Init_Add_Request_low_s low_h(::my_srv::srv::Add_Request::_low_h_type arg)
  {
    msg_.low_h = std::move(arg);
    return Init_Add_Request_low_s(msg_);
  }

private:
  ::my_srv::srv::Add_Request msg_;
};

class Init_Add_Request_color
{
public:
  Init_Add_Request_color()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Add_Request_low_h color(::my_srv::srv::Add_Request::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_Add_Request_low_h(msg_);
  }

private:
  ::my_srv::srv::Add_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_srv::srv::Add_Request>()
{
  return my_srv::srv::builder::Init_Add_Request_color();
}

}  // namespace my_srv


namespace my_srv
{

namespace srv
{

namespace builder
{

class Init_Add_Response_message
{
public:
  explicit Init_Add_Response_message(::my_srv::srv::Add_Response & msg)
  : msg_(msg)
  {}
  ::my_srv::srv::Add_Response message(::my_srv::srv::Add_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_srv::srv::Add_Response msg_;
};

class Init_Add_Response_success
{
public:
  Init_Add_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Add_Response_message success(::my_srv::srv::Add_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Add_Response_message(msg_);
  }

private:
  ::my_srv::srv::Add_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_srv::srv::Add_Response>()
{
  return my_srv::srv::builder::Init_Add_Response_success();
}

}  // namespace my_srv


namespace my_srv
{

namespace srv
{

namespace builder
{

class Init_Add_Event_response
{
public:
  explicit Init_Add_Event_response(::my_srv::srv::Add_Event & msg)
  : msg_(msg)
  {}
  ::my_srv::srv::Add_Event response(::my_srv::srv::Add_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_srv::srv::Add_Event msg_;
};

class Init_Add_Event_request
{
public:
  explicit Init_Add_Event_request(::my_srv::srv::Add_Event & msg)
  : msg_(msg)
  {}
  Init_Add_Event_response request(::my_srv::srv::Add_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Add_Event_response(msg_);
  }

private:
  ::my_srv::srv::Add_Event msg_;
};

class Init_Add_Event_info
{
public:
  Init_Add_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Add_Event_request info(::my_srv::srv::Add_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Add_Event_request(msg_);
  }

private:
  ::my_srv::srv::Add_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_srv::srv::Add_Event>()
{
  return my_srv::srv::builder::Init_Add_Event_info();
}

}  // namespace my_srv

#endif  // MY_SRV__SRV__DETAIL__ADD__BUILDER_HPP_
