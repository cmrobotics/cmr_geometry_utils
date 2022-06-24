#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include <angles/angles.h>

namespace cmr_geometry_utils {

double compute_distance_2d(const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & goal)
{
  return std::hypot(goal.x - start.x, 
                    goal.y - start.y);
}

double compute_distance_2d(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal)
{
  return compute_distance_2d(start.position, goal.position);
}

double compute_distance_2d(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  return compute_distance_2d(start.pose, goal.pose);
}


double compute_yaw_distance(const geometry_msgs::msg::Quaternion & start, const geometry_msgs::msg::Quaternion & goal)
{
  double start_yaw = angles::normalize_angle(tf2::getYaw(start));
  double goal_yaw = angles::normalize_angle(tf2::getYaw(goal));
  return std::abs(angles::normalize_angle(goal_yaw - start_yaw));
}

double compute_yaw_distance(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal)
{
  return compute_yaw_distance(start.orientation, goal.orientation);
}

double compute_yaw_distance(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  return compute_yaw_distance(start.pose, goal.pose);
}


}
