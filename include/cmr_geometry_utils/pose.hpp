// #pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "quaternion.hpp"

namespace cmr_geometry_utils {
namespace pose {
    template <template<typename> class Container>
    inline void average(const Container<geometry_msgs::msg::Pose> & poses, geometry_msgs::msg::Pose& average_pose){
        std::vector<geometry_msgs::msg::Quaternion> quats;
        for(const auto& pose : poses){
            average_pose.position.x += pose.position.x;
            average_pose.position.y += pose.position.y;
            average_pose.position.z += pose.position.z;

            quats.push_back(pose.orientation);
        }

        average_pose.position.x /= poses.size();
        average_pose.position.y /= poses.size();
        average_pose.position.z /= poses.size();

        
        cmr_geometry_utils::quaternion::average(quats, average_pose.orientation);
    }

    template <template<typename> class Container>
    inline void average(const Container<geometry_msgs::msg::PoseWithCovariance> & poses_with_covariance, geometry_msgs::msg::Pose& average_pose){
        std::vector<geometry_msgs::msg::Pose> poses;
        for(const auto& pose : poses_with_covariance){
            poses.push_back(pose.pose);
        }
        average(poses, average_pose);
    }
} // pose
} // cmr_geometry_utils
