#include "cmr_geometry_utils/basic_geometry_utils.hpp"

#include <gtest/gtest.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmr_tests_utils/utils.hpp>
#include <math.h>
#include <angles/angles.h>

TEST(BasicGeometryTest, calculate_linear_distance)
{
  double dx, dy, dist;
  geometry_msgs::msg::PoseStamped p1, p2;
  for (std::size_t k = 0; k < 50; ++k)
  {
    p1.pose.position.x = cmr_tests_utils::randMToN(-30.0, 30.0);
    p1.pose.position.y = cmr_tests_utils::randMToN(-30.0, 30.0);
    p2.pose.position.x = cmr_tests_utils::randMToN(-30.0, 30.0);
    p2.pose.position.y = cmr_tests_utils::randMToN(-30.0, 30.0);
    dx = p2.pose.position.x - p1.pose.position.x;
    dy = p2.pose.position.y - p1.pose.position.y;
    dist = sqrt(dx*dx+dy*dy);
    ASSERT_NEAR(cmr_geometry_utils::compute_distance_2d(p1, p2), dist, 1e-4);
    ASSERT_NEAR(cmr_geometry_utils::compute_distance_2d(p1.pose, p2.pose), dist, 1e-4);
    ASSERT_NEAR(cmr_geometry_utils::compute_distance_2d(p1.pose.position, p2.pose.position), dist, 1e-4);
  }
}

TEST(BasicGeometryTest, calculate_angle_distance)
{
  double angle1, angle2, dist;
  geometry_msgs::msg::PoseStamped p1, p2;
  tf2::Quaternion quat;

  for (std::size_t k = 0; k < 50; ++k)
  {
    angle1 = cmr_tests_utils::randMToN(-50, 50);
    quat.setRPY(0, 0, angle1);
    p1.pose.orientation = tf2::toMsg(quat);

    angle2 = cmr_tests_utils::randMToN(-50, 50);
    quat.setRPY(0, 0, angle2);
    p2.pose.orientation = tf2::toMsg(quat);

    dist = std::abs(angles::normalize_angle(angle1) - angles::normalize_angle(angle2));
    ASSERT_NEAR(cmr_geometry_utils::compute_yaw_distance(p1, p2), dist, 1e-4);
    ASSERT_NEAR(cmr_geometry_utils::compute_yaw_distance(p1.pose, p2.pose), dist, 1e-4);
    ASSERT_NEAR(cmr_geometry_utils::compute_yaw_distance(p1.pose.orientation, p2.pose.orientation), dist, 1e-4);

    ASSERT_GE(dist, 0);
    ASSERT_LE(dist, M_PI);
    ASSERT_GE(dist, -M_PI);
  }
}
