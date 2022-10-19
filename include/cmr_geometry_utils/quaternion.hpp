#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <Eigen/SVD>

namespace cmr_geometry_utils {
namespace quaternion {
    inline void msg2eigen(geometry_msgs::msg::Quaternion q_msg, Eigen::Vector4f& q_eig){
        q_eig[0] = q_msg.x;
        q_eig[1] = q_msg.y;
        q_eig[2] = q_msg.z;
        q_eig[3] = q_msg.w;
    }

    inline void average(
        std::vector<geometry_msgs::msg::Quaternion> quats
        , geometry_msgs::msg::Quaternion& average_quaternion){

        Eigen::Matrix4f A = Eigen::Matrix4f::Zero();

        for (auto& q_msg : quats){
            Eigen::Vector4f q;
            msg2eigen(q_msg, q);

            A += q * q.transpose();
        }

        // normalise with the number of quaternions
        A /= quats.size();

        // Compute the SVD of this 4x4 matrix
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXf U = svd.matrixU();

        // find the eigen vector corresponding to the largest eigen value.
        // according to Eigen doc, singular values sorted in descending order 
        // i.e. the first singular vector corresponds to the largest singular value
        size_t largestEigenValueIndex = 0;

        Eigen::Vector4f average;
        average_quaternion.x = U(0, largestEigenValueIndex);
        average_quaternion.y = U(1, largestEigenValueIndex);
        average_quaternion.z = U(2, largestEigenValueIndex);
        average_quaternion.w = U(3, largestEigenValueIndex);
    }
} // quaternion
} // cmr_geometry_utils