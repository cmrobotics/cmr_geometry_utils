#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "quaternion.hpp"

namespace cmr_geometry_utils {
namespace pose {
    void average(std::vector<geometry_msgs::msg::Pose> poses, geometry_msgs::msg::Pose& average_pose, std::vector<double> weights = {}){
        if(weights.size() != 0 && pose.size() != weights.size()){
            throw std::invalid_argument("poses and weights arrays should be of the same size OR weights should be empty");
            return;
        }

        std::vector<geometry_msgs::msg::Quaternion> quats;
        double average_x = 0.0;
        double average_y = 0.0;
        for(auto& pose : poses){
            average_pose.position.x += pose.position.x;
            average_pose.position.y += pose.position.y;

            quats.push_back(pose.orientation);
        }

        average_pose.position.x /= poses.size();
        average_pose.position.y /= poses.size();

        
        cmr_geometry_utils::quaternion::average(quats, average_pose.orientation, weights);
    }
} // pose
} // cmr_geometry_utils