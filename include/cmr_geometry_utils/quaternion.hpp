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


    // @ai_generated Weighted Markley quaternion average (unit quats, right-handed, w last).
    inline void average(const std::vector<geometry_msgs::msg::Quaternion>& quats,
                        const std::vector<double>& weights,
                        geometry_msgs::msg::Quaternion& out_q)
    {
        if (quats.empty()) {
            out_q = geometry_msgs::msg::Quaternion{};
            out_q.w = 1.0;
            return;
        }

        // Normalize weights (and guard against zeros)
        double wsum = 0.0;
        for (double w : weights) wsum += w;
        const double inv_wsum = (wsum > 0.0) ? (1.0 / wsum) : 1.0;

        // Build 4x4 accumulator A = sum_i (w_i * q_i * q_i^T)
        double A[4][4] = {{0}};
        auto add_outer = [&](const geometry_msgs::msg::Quaternion& q, double wnorm){
            // qvec = [x, y, z, w]
            const double qq[4] = {q.x, q.y, q.z, q.w};
            for (int r=0; r<4; ++r)
                for (int c=0; c<4; ++c)
                    A[r][c] += wnorm * qq[r] * qq[c];
        };

        for (size_t i = 0; i < quats.size(); ++i)
            add_outer(quats[i], (i < weights.size() ? weights[i] : 1.0) * inv_wsum);

        // Power iteration to get the principal eigenvector of A
        double v[4] = {0,0,0,1};                       // init near identity
        for (int it=0; it<12; ++it) {                  // few iters suffice
            double Av[4] = {0,0,0,0};
            for (int r=0; r<4; ++r)
                for (int c=0; c<4; ++c)
                    Av[r] += A[r][c] * v[c];
            // normalize
            double n = std::sqrt(Av[0]*Av[0]+Av[1]*Av[1]+Av[2]*Av[2]+Av[3]*Av[3]);
            if (n <= 1e-12) break;
            for (int k=0; k<4; ++k) v[k] = Av[k] / n;
        }

        out_q.x = v[0]; out_q.y = v[1]; out_q.z = v[2]; out_q.w = v[3];
        // Ensure canonical hemisphere: flip if needed to keep w >= 0
        if (out_q.w < 0) { out_q.x*=-1; out_q.y*=-1; out_q.z*=-1; out_q.w*=-1; }
    }


    template <template<typename> class Container>
    inline void average(
        const Container<geometry_msgs::msg::Quaternion> & quats
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
