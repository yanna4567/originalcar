// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from origincar_base:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef ORIGINCAR_BASE__MSG__DETAIL__POSITION__BUILDER_HPP_
#define ORIGINCAR_BASE__MSG__DETAIL__POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "origincar_base/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace origincar_base
{

namespace msg
{

namespace builder
{

class Init_Position_distance
{
public:
  explicit Init_Position_distance(::origincar_base::msg::Position & msg)
  : msg_(msg)
  {}
  ::origincar_base::msg::Position distance(::origincar_base::msg::Position::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::origincar_base::msg::Position msg_;
};

class Init_Position_angle_y
{
public:
  explicit Init_Position_angle_y(::origincar_base::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_distance angle_y(::origincar_base::msg::Position::_angle_y_type arg)
  {
    msg_.angle_y = std::move(arg);
    return Init_Position_distance(msg_);
  }

private:
  ::origincar_base::msg::Position msg_;
};

class Init_Position_angle_x
{
public:
  Init_Position_angle_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_angle_y angle_x(::origincar_base::msg::Position::_angle_x_type arg)
  {
    msg_.angle_x = std::move(arg);
    return Init_Position_angle_y(msg_);
  }

private:
  ::origincar_base::msg::Position msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::origincar_base::msg::Position>()
{
  return origincar_base::msg::builder::Init_Position_angle_x();
}

}  // namespace origincar_base

#endif  // ORIGINCAR_BASE__MSG__DETAIL__POSITION__BUILDER_HPP_
