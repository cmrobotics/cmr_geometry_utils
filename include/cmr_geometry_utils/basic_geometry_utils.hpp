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
  return std::hypot(goal.position.x - start.position.x, 
                    goal.position.y - start.position.y);
}

double compute_distance_2d(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  return std::hypot(goal.pose.position.x - start.pose.position.x, 
                    goal.pose.position.y - start.pose.position.y);
}


double compute_yaw_distance(const geometry_msgs::msg::Quaternion & start, const geometry_msgs::msg::Quaternion & goal)
{
  double start_yaw = angles::normalize_angle(tf2::getYaw(start));
  double goal_yaw = angles::normalize_angle(tf2::getYaw(goal));
  return std::abs(angles::normalize_angle(goal_yaw - start_yaw));
}

double compute_yaw_distance(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal)
{
  double start_yaw = angles::normalize_angle(tf2::getYaw(start.orientation));
  double goal_yaw = angles::normalize_angle(tf2::getYaw(goal.orientation));
  return std::abs(angles::normalize_angle(goal_yaw - start_yaw));
}

double compute_yaw_distance(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  double start_yaw = angles::normalize_angle(tf2::getYaw(start.pose.orientation));
  double goal_yaw = angles::normalize_angle(tf2::getYaw(goal.pose.orientation));
  return std::abs(angles::normalize_angle(goal_yaw - start_yaw));
}


}
