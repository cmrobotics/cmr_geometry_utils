// #pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "quaternion.hpp"

namespace cmr_geometry_utils {

namespace pose {
    template <template<typename> class Container>
    inline void average(const Container<geometry_msgs::msg::Pose>& poses,
                        const std::vector<double>& weights,
                        geometry_msgs::msg::Pose& average_pose)
    {
        average_pose.position.x = 0.0;
        average_pose.position.y = 0.0;
        average_pose.position.z = 0.0;

        std::vector<geometry_msgs::msg::Quaternion> quats;
        quats.reserve(poses.size());

        // Normalize weights here too (robust if caller didn't)
        double wsum = 0.0;
        for (size_t i = 0; i < poses.size(); ++i)
            wsum += (i < weights.size() ? weights[i] : 1.0);
        const double inv_wsum = (wsum > 0.0) ? (1.0 / wsum) : 1.0;

        for (size_t i = 0; i < poses.size(); ++i) {
            const auto& p = poses[i];
            const double w = (i < weights.size() ? weights[i] : 1.0) * inv_wsum;

            average_pose.position.x += w * p.position.x;
            average_pose.position.y += w * p.position.y;
            average_pose.position.z += w * p.position.z;

            quats.push_back(p.orientation);
        }

        cmr_geometry_utils::quaternion::average(quats, /*weights=*/[&]{
            std::vector<double> wnorm(poses.size(), 1.0);
            for (size_t i = 0; i < wnorm.size(); ++i)
                wnorm[i] = (i < weights.size() ? weights[i] : 1.0) * inv_wsum;
            return wnorm;
        }(), average_pose.orientation);
    }

    template <template<typename> class Container>
    inline void average(const Container<geometry_msgs::msg::PoseWithCovariance>& poses_with_covariance,
                        const std::vector<double>& weights,
                        geometry_msgs::msg::Pose& average_pose)
    {
        std::vector<geometry_msgs::msg::Pose> poses;
        poses.reserve(poses_with_covariance.size());
        for (const auto& pwc : poses_with_covariance){
            poses.push_back(pwc.pose);
        }
        average(poses, weights, average_pose);
    }

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
