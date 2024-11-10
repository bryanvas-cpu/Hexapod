// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hexapod_interfaces:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
#define HEXAPOD_INTERFACES__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hexapod_interfaces/msg/detail/point_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace hexapod_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PointArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PointArray & msg, bool use_flow_style = false)
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

}  // namespace hexapod_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hexapod_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hexapod_interfaces::msg::PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  hexapod_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hexapod_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hexapod_interfaces::msg::PointArray & msg)
{
  return hexapod_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hexapod_interfaces::msg::PointArray>()
{
  return "hexapod_interfaces::msg::PointArray";
}

template<>
inline const char * name<hexapod_interfaces::msg::PointArray>()
{
  return "hexapod_interfaces/msg/PointArray";
}

template<>
struct has_fixed_size<hexapod_interfaces::msg::PointArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hexapod_interfaces::msg::PointArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hexapod_interfaces::msg::PointArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
