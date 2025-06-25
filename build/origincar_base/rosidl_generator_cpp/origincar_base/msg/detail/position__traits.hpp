// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from origincar_base:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef ORIGINCAR_BASE__MSG__DETAIL__POSITION__TRAITS_HPP_
#define ORIGINCAR_BASE__MSG__DETAIL__POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "origincar_base/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace origincar_base
{

namespace msg
{

inline void to_flow_style_yaml(
  const Position & msg,
  std::ostream & out)
{
  out << "{";
  // member: angle_x
  {
    out << "angle_x: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_x, out);
    out << ", ";
  }

  // member: angle_y
  {
    out << "angle_y: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_y, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_x: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_x, out);
    out << "\n";
  }

  // member: angle_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_y: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_y, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace origincar_base

namespace rosidl_generator_traits
{

[[deprecated("use origincar_base::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const origincar_base::msg::Position & msg,
  std::ostream & out, size_t indentation = 0)
{
  origincar_base::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use origincar_base::msg::to_yaml() instead")]]
inline std::string to_yaml(const origincar_base::msg::Position & msg)
{
  return origincar_base::msg::to_yaml(msg);
}

template<>
inline const char * data_type<origincar_base::msg::Position>()
{
  return "origincar_base::msg::Position";
}

template<>
inline const char * name<origincar_base::msg::Position>()
{
  return "origincar_base/msg/Position";
}

template<>
struct has_fixed_size<origincar_base::msg::Position>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<origincar_base::msg::Position>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<origincar_base::msg::Position>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ORIGINCAR_BASE__MSG__DETAIL__POSITION__TRAITS_HPP_
