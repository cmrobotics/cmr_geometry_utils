#include "cmr_geometry_utils/basic_geometry_utils.hpp"

#include <gtest/gtest.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmr_tests_utils/utils.hpp>
#include <math.h>

TEST(BasicGeometryTest, calculate_linear_distance)
{
  double dx, dy, dist;
  geometry_msgs::msg::Point p1, p2;
  for (std::size_t k = 0; k < 50; ++k)
  {
    p1.x = cmr_tests_utils::randMToN(-30.0, 30.0);
    p1.y = cmr_tests_utils::randMToN(-30.0, 30.0);
    p2.x = cmr_tests_utils::randMToN(-30.0, 30.0);
    p2.y = cmr_tests_utils::randMToN(-30.0, 30.0);
    dx = p2.x - p1.x;
    dy = p2.y - p1.y;
    dist = sqrt(dx*dx+dy*dy);
    ASSERT_NEAR(cmr_geometry_utils::compute_distance(p1, p2), dist, 1e-4);
  }
}

TEST(BasicGeometryTest, calculate_angle_distance)
{
  double angle1, angle2, dist;
  geometry_msgs::msg::Quaternion quat_msg, quat_msg2;
  tf2::Quaternion quat;

  for (std::size_t k = 0; k < 50; ++k)
  {
    angle1 = cmr_tests_utils::randMToN(-50, 50);
    quat.setRPY(0, 0, angle1);
    quat_msg = tf2::toMsg(quat);

    angle2 = cmr_tests_utils::randMToN(-50, 50);
    quat.setRPY(0, 0, angle2);
    quat_msg2 = tf2::toMsg(quat);

    dist = cmr_geometry_utils::compute_angle_distance(quat_msg, quat_msg2);
    ASSERT_GE(dist, 0);
    ASSERT_LE(dist, M_PI);
    ASSERT_GE(dist, -M_PI);
  }
}
