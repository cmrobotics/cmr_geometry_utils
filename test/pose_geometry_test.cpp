#include "cmr_geometry_utils/pose.hpp"

#include <gtest/gtest.h>
#include <geometry_msgs/msg/pose.hpp>

TEST(PoseGeometryTest, calculate_average_pose)
{
    geometry_msgs::msg::Pose p;
    std::vector<geometry_msgs::msg::Pose> v = {};
    cmr_geometry_utils::pose::average(v, p);

    ASSERT_EQ(p.position.x, 1234);
}