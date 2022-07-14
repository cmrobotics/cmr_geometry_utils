#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include <angles/angles.h>

namespace cmr_geometry_utils {

inline double compute_distance_2d(const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & goal)
{
  return std::hypot(goal.x - start.x, 
                    goal.y - start.y);
}

inline double compute_distance_2d(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal)
{
  return compute_distance_2d(start.position, goal.position);
}

inline double compute_distance_2d(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  return compute_distance_2d(start.pose, goal.pose);
}


inline double compute_yaw_distance(const geometry_msgs::msg::Quaternion & start, const geometry_msgs::msg::Quaternion & goal)
{
  double start_yaw = tf2::getYaw(start);
  double goal_yaw = tf2::getYaw(goal);
  return std::abs(angles::normalize_angle(goal_yaw - start_yaw));
}

inline double compute_yaw_distance(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal)
{
  return compute_yaw_distance(start.orientation, goal.orientation);
}

inline double compute_yaw_distance(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  return compute_yaw_distance(start.pose, goal.pose);
}


}
