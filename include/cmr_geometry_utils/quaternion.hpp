#pragma once

#include <geometry_msgs/msg/quaternion.hpp>

namespace cmr_geometry_utils {
namespace quaternion {
    void average(std::vector<geometry_msgs::msg::Quaternion> quats, geometry_msgs::msg::Quaternion& average_quaternion, std::vector<double> weights = {}){

    }
} // quaternion
} // cmr_geometry_utils