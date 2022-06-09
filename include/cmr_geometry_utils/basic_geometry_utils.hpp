#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include <angles/angles.h>

namespace cmr_geometry_utils {

double compute_distance(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal)
{
  return std::hypot(goal.x - start.x, 
                    goal.y - start.y);
}

double compute_angle_distance(geometry_msgs::msg::Quaternion start, geometry_msgs::msg::Quaternion goal)
{
  double start_yaw = angles::normalize_angle(tf2::getYaw(start));
  double goal_yaw = angles::normalize_angle(tf2::getYaw(goal));
  return std::abs(angles::normalize_angle(goal_yaw - start_yaw));
}

}
