#include "cmr_geometry_utils/pose.hpp"

#include <gtest/gtest.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <deque>

TEST(PoseGeometryTest, calculate_average_pose)
{
    geometry_msgs::msg::Pose p1;
    p1.position.x = 0.0;
    p1.position.y = 0.0;
    p1.position.z = 0.0;

    p1.orientation.x = 0.0;
    p1.orientation.y = 0.0;
    p1.orientation.z = 0.0;
    p1.orientation.w = 1.0;

    geometry_msgs::msg::Pose p2;
    p2.position.x = 20.0;
    p2.position.y = 10.0;
    p2.position.z = 0.0;

    p2.orientation.x = 0.8788171;
    p2.orientation.y = 0.0;
    p2.orientation.z = 0.0;
    p2.orientation.w = 0.4771588;

    // test using std::vector
    {
      geometry_msgs::msg::Pose p;
      ASSERT_EQ(p.position.x, 0);
      ASSERT_EQ(p.position.y, 0);
      ASSERT_EQ(p.position.z, 0);

      std::vector<geometry_msgs::msg::Pose> v = {p1, p2};
      cmr_geometry_utils::pose::average(v, p);

      ASSERT_EQ(p.position.x, 10.);
      ASSERT_EQ(p.position.y, 5.);
      ASSERT_EQ(p.position.z, 0.0);

      ASSERT_NEAR(p.orientation.x, 0.5112, 1e-4);
      ASSERT_EQ(p.orientation.y, 0.0);
      ASSERT_EQ(p.orientation.z, 0.0);
      ASSERT_NEAR(p.orientation.w, 0.8594, 1e-4);
    }

    // same test using std::deque
    {
      geometry_msgs::msg::Pose p;
      ASSERT_EQ(p.position.x, 0);
      ASSERT_EQ(p.position.y, 0);
      ASSERT_EQ(p.position.z, 0);

      std::deque<geometry_msgs::msg::Pose> q = {p1, p2};
      cmr_geometry_utils::pose::average(q, p);

      ASSERT_EQ(p.position.x, 10.);
      ASSERT_EQ(p.position.y, 5.);
      ASSERT_EQ(p.position.z, 0.0);

      ASSERT_NEAR(p.orientation.x, 0.5112, 1e-4);
      ASSERT_EQ(p.orientation.y, 0.0);
      ASSERT_EQ(p.orientation.z, 0.0);
      ASSERT_NEAR(p.orientation.w, 0.8594, 1e-4);
    }
}
