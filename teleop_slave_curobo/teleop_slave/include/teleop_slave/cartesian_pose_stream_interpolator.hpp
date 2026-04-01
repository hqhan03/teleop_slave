#pragma once

#include <algorithm>
#include <chrono>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace teleop_slave {

inline tf2::Quaternion NormalizeStreamQuaternion(tf2::Quaternion quat) {
    if (quat.length2() < 1e-12) {
        return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }
    quat.normalize();
    if (quat.w() < 0.0) {
        quat = tf2::Quaternion(-quat.x(), -quat.y(), -quat.z(), -quat.w());
    }
    return quat;
}

inline geometry_msgs::msg::Pose NormalizePoseQuaternion(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::Pose normalized = pose;
    tf2::Quaternion quat;
    tf2::fromMsg(pose.orientation, quat);
    normalized.orientation = tf2::toMsg(NormalizeStreamQuaternion(quat));
    return normalized;
}

inline geometry_msgs::msg::Pose MakeIdentityPose() {
    geometry_msgs::msg::Pose identity;
    identity.orientation.w = 1.0;
    return identity;
}

inline geometry_msgs::msg::Pose InterpolatePoseLinearSlerp(
    const geometry_msgs::msg::Pose& from,
    const geometry_msgs::msg::Pose& to,
    double t) {
    const double clamped_t = std::clamp(t, 0.0, 1.0);

    geometry_msgs::msg::Pose interpolated;
    interpolated.position.x = from.position.x + clamped_t * (to.position.x - from.position.x);
    interpolated.position.y = from.position.y + clamped_t * (to.position.y - from.position.y);
    interpolated.position.z = from.position.z + clamped_t * (to.position.z - from.position.z);

    tf2::Quaternion from_q;
    tf2::Quaternion to_q;
    tf2::fromMsg(from.orientation, from_q);
    tf2::fromMsg(to.orientation, to_q);
    interpolated.orientation = tf2::toMsg(
        NormalizeStreamQuaternion(from_q).slerp(NormalizeStreamQuaternion(to_q), clamped_t).normalized());
    return interpolated;
}

inline geometry_msgs::msg::Pose ComputeIncrementalPoseDeltaBaseFrame(
    const geometry_msgs::msg::Pose& from,
    const geometry_msgs::msg::Pose& to) {
    const geometry_msgs::msg::Pose normalized_from = NormalizePoseQuaternion(from);
    const geometry_msgs::msg::Pose normalized_to = NormalizePoseQuaternion(to);

    geometry_msgs::msg::Pose delta = MakeIdentityPose();
    delta.position.x = normalized_to.position.x - normalized_from.position.x;
    delta.position.y = normalized_to.position.y - normalized_from.position.y;
    delta.position.z = normalized_to.position.z - normalized_from.position.z;

    tf2::Quaternion from_q;
    tf2::Quaternion to_q;
    tf2::fromMsg(normalized_from.orientation, from_q);
    tf2::fromMsg(normalized_to.orientation, to_q);
    delta.orientation = tf2::toMsg(
        NormalizeStreamQuaternion(to_q * NormalizeStreamQuaternion(from_q).inverse()));
    return delta;
}

class CartesianPoseStreamInterpolator {
public:
    void reset(const geometry_msgs::msg::Pose& seed_pose,
               std::chrono::steady_clock::time_point now) {
        start_pose_ = NormalizePoseQuaternion(seed_pose);
        target_pose_ = start_pose_;
        segment_start_time_ = now;
        segment_duration_sec_ = 0.0;
        initialized_ = true;
    }

    void setTarget(const geometry_msgs::msg::Pose& target_pose,
                   std::chrono::steady_clock::time_point now,
                   double segment_duration_sec,
                   bool interpolate) {
        const geometry_msgs::msg::Pose normalized_target = NormalizePoseQuaternion(target_pose);
        if (!initialized_) {
            reset(normalized_target, now);
            return;
        }

        start_pose_ = sample(now);
        target_pose_ = normalized_target;
        segment_start_time_ = now;
        segment_duration_sec_ = interpolate ? std::max(0.0, segment_duration_sec) : 0.0;
    }

    geometry_msgs::msg::Pose sample(std::chrono::steady_clock::time_point now) const {
        if (!initialized_) {
            return MakeIdentityPose();
        }

        if (segment_duration_sec_ <= 1e-6) {
            return target_pose_;
        }

        const double elapsed_sec =
            std::chrono::duration<double>(now - segment_start_time_).count();
        const double t = std::clamp(elapsed_sec / segment_duration_sec_, 0.0, 1.0);
        return InterpolatePoseLinearSlerp(start_pose_, target_pose_, t);
    }

    const geometry_msgs::msg::Pose& target_pose() const {
        return target_pose_;
    }

    bool initialized() const {
        return initialized_;
    }

private:
    geometry_msgs::msg::Pose start_pose_{};
    geometry_msgs::msg::Pose target_pose_{};
    std::chrono::steady_clock::time_point segment_start_time_{};
    double segment_duration_sec_{0.0};
    bool initialized_{false};
};

}  // namespace teleop_slave
