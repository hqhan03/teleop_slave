#include "teleop_slave/fr5_teleop_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>

#include <tf2/LinearMath/Matrix3x3.h>

namespace teleop_slave {

namespace {

constexpr double kMmPerMeter = 1000.0;
constexpr double kDegPerRad = 180.0 / M_PI;
constexpr double kRadPerDeg = M_PI / 180.0;

tf2::Quaternion NormalizeQuaternion(tf2::Quaternion quat) {
    quat.normalize();
    if (quat.w() < 0.0) {
        quat = tf2::Quaternion(-quat.x(), -quat.y(), -quat.z(), -quat.w());
    }
    return quat;
}

tf2::Vector3 ClampVectorPerAxis(const tf2::Vector3& input,
                                const tf2::Vector3& min_xyz,
                                const tf2::Vector3& max_xyz) {
    return tf2::Vector3(
        std::clamp(input.x(), min_xyz.x(), max_xyz.x()),
        std::clamp(input.y(), min_xyz.y(), max_xyz.y()),
        std::clamp(input.z(), min_xyz.z(), max_xyz.z()));
}

}  // namespace

std::string ExpandUserPath(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    const char* home = std::getenv("HOME");
    if (home == nullptr) {
        return path;
    }

    if (path.size() == 1) {
        return std::string(home);
    }

    if (path[1] == '/') {
        return std::string(home) + path.substr(1);
    }

    return path;
}

OrientationMode ParseOrientationMode(const std::string& mode) {
    if (mode == "position_only") {
        return OrientationMode::kPositionOnly;
    }
    if (mode == "yaw_only") {
        return OrientationMode::kYawOnly;
    }
    if (mode == "full_6dof") {
        return OrientationMode::kFull6Dof;
    }

    throw std::invalid_argument("Unsupported orientation_mode: " + mode);
}

std::string OrientationModeToString(OrientationMode mode) {
    switch (mode) {
        case OrientationMode::kPositionOnly:
            return "position_only";
        case OrientationMode::kYawOnly:
            return "yaw_only";
        case OrientationMode::kFull6Dof:
            return "full_6dof";
    }
    return "position_only";
}

tf2::Vector3 ApplyAxisMapping(const tf2::Vector3& input,
                              const std::array<int, 3>& axes,
                              const std::array<double, 3>& signs) {
    const double values[3] = {input.x(), input.y(), input.z()};
    return tf2::Vector3(
        signs[0] * values[axes[0]],
        signs[1] * values[axes[1]],
        signs[2] * values[axes[2]]);
}

tf2::Vector3 ScaleVector(const tf2::Vector3& input, const tf2::Vector3& scale) {
    return tf2::Vector3(input.x() * scale.x(), input.y() * scale.y(), input.z() * scale.z());
}

tf2::Quaternion QuaternionFromRPYDegrees(const tf2::Vector3& rpy_deg) {
    tf2::Quaternion quat;
    quat.setRPY(rpy_deg.x() * kRadPerDeg, rpy_deg.y() * kRadPerDeg, rpy_deg.z() * kRadPerDeg);
    return NormalizeQuaternion(quat);
}

tf2::Quaternion ComputeMappedOrientation(const tf2::Quaternion& base_robot_orientation,
                                         const tf2::Quaternion& tracker_zero_orientation,
                                         const tf2::Quaternion& tracker_current_orientation,
                                         const tf2::Quaternion& tracker_to_robot_basis,
                                         OrientationMode mode) {
    if (mode == OrientationMode::kPositionOnly) {
        return NormalizeQuaternion(base_robot_orientation);
    }

    tf2::Quaternion tracker_delta = tracker_current_orientation * tracker_zero_orientation.inverse();
    tracker_delta = NormalizeQuaternion(tracker_delta);

    tf2::Quaternion basis = NormalizeQuaternion(tracker_to_robot_basis);
    tf2::Quaternion mapped_delta = basis * tracker_delta * basis.inverse();
    mapped_delta = NormalizeQuaternion(mapped_delta);

    if (mode == OrientationMode::kYawOnly) {
        tf2::Matrix3x3 yaw_matrix(mapped_delta);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        yaw_matrix.getRPY(roll, pitch, yaw);

        tf2::Quaternion yaw_only;
        yaw_only.setRPY(0.0, 0.0, yaw);
        mapped_delta = NormalizeQuaternion(yaw_only);
    }

    return NormalizeQuaternion(base_robot_orientation * mapped_delta);
}

double QuaternionAngularDistanceDegrees(const tf2::Quaternion& from, const tf2::Quaternion& to) {
    tf2::Quaternion delta = NormalizeQuaternion(from.inverse() * to);
    const double angle = delta.getAngleShortestPath();
    return angle * kDegPerRad;
}

geometry_msgs::msg::Pose ClampPoseTarget(const geometry_msgs::msg::Pose& candidate,
                                         const geometry_msgs::msg::Pose* previous,
                                         const tf2::Vector3& min_xyz,
                                         const tf2::Vector3& max_xyz,
                                         double workspace_radius,
                                         double max_linear_step_m,
                                         double max_angular_step_deg) {
    tf2::Vector3 position(candidate.position.x, candidate.position.y, candidate.position.z);
    tf2::Quaternion orientation(candidate.orientation.x, candidate.orientation.y,
                                candidate.orientation.z, candidate.orientation.w);
    orientation = NormalizeQuaternion(orientation);

    if (previous != nullptr) {
        tf2::Vector3 prev_position(previous->position.x, previous->position.y, previous->position.z);
        tf2::Vector3 delta = position - prev_position;
        const double length = delta.length();
        if (max_linear_step_m > 0.0 && length > max_linear_step_m) {
            position = prev_position + delta.normalized() * max_linear_step_m;
        }

        tf2::Quaternion prev_orientation(previous->orientation.x, previous->orientation.y,
                                         previous->orientation.z, previous->orientation.w);
        prev_orientation = NormalizeQuaternion(prev_orientation);

        const double angle_deg = QuaternionAngularDistanceDegrees(prev_orientation, orientation);
        if (max_angular_step_deg > 0.0 && angle_deg > max_angular_step_deg) {
            const double ratio = max_angular_step_deg / angle_deg;
            orientation = NormalizeQuaternion(prev_orientation.slerp(orientation, ratio));
        }
    }

    position = ClampVectorPerAxis(position, min_xyz, max_xyz);
    if (workspace_radius > 0.0 && position.length() > workspace_radius) {
        position = position.normalized() * workspace_radius;
    }

    geometry_msgs::msg::Pose clamped = candidate;
    clamped.position.x = position.x();
    clamped.position.y = position.y();
    clamped.position.z = position.z();
    clamped.orientation.x = orientation.x();
    clamped.orientation.y = orientation.y();
    clamped.orientation.z = orientation.z();
    clamped.orientation.w = orientation.w();
    return clamped;
}

DescPose PoseToDescPose(const geometry_msgs::msg::Pose& pose) {
    DescPose desc_pose;
    desc_pose.tran.x = pose.position.x * kMmPerMeter;
    desc_pose.tran.y = pose.position.y * kMmPerMeter;
    desc_pose.tran.z = pose.position.z * kMmPerMeter;

    tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    quat = NormalizeQuaternion(quat);
    tf2::Matrix3x3 rot(quat);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    rot.getRPY(roll, pitch, yaw);

    desc_pose.rpy.rx = roll * kDegPerRad;
    desc_pose.rpy.ry = pitch * kDegPerRad;
    desc_pose.rpy.rz = yaw * kDegPerRad;
    return desc_pose;
}

geometry_msgs::msg::Pose DescPoseToPose(const DescPose& pose) {
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = pose.tran.x / kMmPerMeter;
    ros_pose.position.y = pose.tran.y / kMmPerMeter;
    ros_pose.position.z = pose.tran.z / kMmPerMeter;

    tf2::Quaternion quat;
    quat.setRPY(pose.rpy.rx * kRadPerDeg, pose.rpy.ry * kRadPerDeg, pose.rpy.rz * kRadPerDeg);
    quat = NormalizeQuaternion(quat);

    ros_pose.orientation.x = quat.x();
    ros_pose.orientation.y = quat.y();
    ros_pose.orientation.z = quat.z();
    ros_pose.orientation.w = quat.w();
    return ros_pose;
}

ControllerValidationResult ValidateControllerState(IFairinoRobot& robot,
                                                   int expected_tcp_id,
                                                   int expected_wobj_id) {
    ControllerValidationResult result;

    int tcp_id = -1;
    if (robot.GetActualTCPNum(0, &tcp_id) != 0) {
        result.message = "Failed to query active TCP id";
        return result;
    }

    int wobj_id = -1;
    if (robot.GetActualWObjNum(0, &wobj_id) != 0) {
        result.message = "Failed to query active workobject id";
        return result;
    }

    DescPose tcp_pose;
    if (robot.GetActualTCPPose(0, &tcp_pose) != 0) {
        result.message = "Failed to query active TCP pose";
        return result;
    }

    result.actual_tcp_id = tcp_id;
    result.actual_wobj_id = wobj_id;
    result.actual_tcp_pose = tcp_pose;

    if (expected_tcp_id >= 0 && tcp_id != expected_tcp_id) {
        std::ostringstream stream;
        stream << "Active TCP id mismatch: expected " << expected_tcp_id << ", got " << tcp_id;
        result.message = stream.str();
        return result;
    }

    if (expected_wobj_id >= 0 && wobj_id != expected_wobj_id) {
        std::ostringstream stream;
        stream << "Active workobject id mismatch: expected " << expected_wobj_id
               << ", got " << wobj_id;
        result.message = stream.str();
        return result;
    }

    result.ok = true;
    std::ostringstream stream;
    stream << "TCP=" << tcp_id << " WObj=" << wobj_id
           << " Pose(mm,deg)=[" << tcp_pose.tran.x << ", " << tcp_pose.tran.y << ", "
           << tcp_pose.tran.z << "; " << tcp_pose.rpy.rx << ", " << tcp_pose.rpy.ry
           << ", " << tcp_pose.rpy.rz << "]";
    result.message = stream.str();
    return result;
}

PoseIkSolveResult SolvePoseTargetIK(IFairinoRobot& robot,
                                    const geometry_msgs::msg::PoseStamped& pose_target,
                                    const PoseIkOptions& options) {
    PoseIkSolveResult result;

    ControllerValidationResult validation = ValidateControllerState(
        robot, options.expected_tcp_id, options.expected_wobj_id);
    result.actual_tcp_id = validation.actual_tcp_id;
    result.actual_wobj_id = validation.actual_wobj_id;
    if (!validation.ok) {
        result.message = validation.message;
        return result;
    }

    JointPos reference_joints;
    if (robot.GetActualJointPosDegree(0, &reference_joints) != 0) {
        result.message = "Failed to query current joint positions for IK seed";
        return result;
    }

    const geometry_msgs::msg::Pose current_tcp_pose = DescPoseToPose(validation.actual_tcp_pose);
    geometry_msgs::msg::Pose candidate_pose = pose_target.pose;
    DescPose desc_pose{};
    errno_t ret = 0;
    bool used_orientation_fallback = false;
    int used_position_backoff_step = -1;

    auto try_candidate = [&](const geometry_msgs::msg::Pose& pose,
                             bool orientation_fallback,
                             uint8_t* has_solution) -> bool {
        desc_pose = PoseToDescPose(pose);
        ret = robot.GetInverseKinHasSolution(0, &desc_pose, &reference_joints, has_solution);
        if (ret != 0) {
            return false;
        }
        if (*has_solution == 0) {
            return false;
        }

        candidate_pose = pose;
        used_orientation_fallback = orientation_fallback;
        return true;
    };

    bool solved = false;
    uint8_t has_solution = 0;
    const int max_backoff_steps = std::max(0, options.position_backoff_steps);

    for (int step = 0; step <= max_backoff_steps && !solved; ++step) {
        const double alpha = (max_backoff_steps == 0)
                                 ? 1.0
                                 : 1.0 - (static_cast<double>(step) / max_backoff_steps);

        geometry_msgs::msg::Pose blended_pose = pose_target.pose;
        blended_pose.position.x =
            current_tcp_pose.position.x +
            alpha * (pose_target.pose.position.x - current_tcp_pose.position.x);
        blended_pose.position.y =
            current_tcp_pose.position.y +
            alpha * (pose_target.pose.position.y - current_tcp_pose.position.y);
        blended_pose.position.z =
            current_tcp_pose.position.z +
            alpha * (pose_target.pose.position.z - current_tcp_pose.position.z);

        if (try_candidate(blended_pose, false, &has_solution)) {
            used_position_backoff_step = step;
            solved = true;
            break;
        }

        if (ret != 0) {
            break;
        }

        if (!options.allow_orientation_fallback) {
            continue;
        }

        blended_pose.orientation = current_tcp_pose.orientation;
        if (try_candidate(blended_pose, true, &has_solution)) {
            used_position_backoff_step = step;
            solved = true;
            break;
        }

        if (ret != 0) {
            break;
        }
    }

    if (ret != 0) {
        int maincode = 0;
        int subcode = 0;
        robot.GetRobotErrorCode(&maincode, &subcode);
        std::ostringstream stream;
        stream << "GetInverseKinHasSolution failed: ret=" << ret
               << " main=" << maincode << " sub=" << subcode;
        result.message = stream.str();
        return result;
    }

    if (!solved || has_solution == 0) {
        result.message = "No IK solution for requested pose";
        return result;
    }

    JointPos solved_joints;
    ret = robot.GetInverseKinRef(0, &desc_pose, &reference_joints, &solved_joints);
    if (ret != 0) {
        int maincode = 0;
        int subcode = 0;
        robot.GetRobotErrorCode(&maincode, &subcode);
        std::ostringstream stream;
        stream << "GetInverseKinRef failed: ret=" << ret
               << " main=" << maincode << " sub=" << subcode;
        result.message = stream.str();
        return result;
    }

    for (size_t i = 0; i < result.joint_target_deg.size(); ++i) {
        result.joint_target_deg[i] = solved_joints.jPos[i];
    }

    result.success = true;
    std::ostringstream stream;
    stream << "IK solution ready";
    if (used_position_backoff_step > 0) {
        stream << " with position backoff step " << used_position_backoff_step;
    }
    if (used_orientation_fallback) {
        stream << " and orientation fallback";
    }
    result.message = stream.str();
    return result;
}

}  // namespace teleop_slave
