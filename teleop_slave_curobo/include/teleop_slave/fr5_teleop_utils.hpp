#pragma once

#include <array>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "teleop_slave/fairino_robot_interface.hpp"

namespace teleop_slave {

enum class OrientationMode {
    kPositionOnly,
    kYawOnly,
    kFull6Dof,
};

enum class ServoCartMode {
    kIncrementalBase,
    kAbsoluteBase,
};

struct ControllerValidationResult {
    bool ok{false};
    std::string message;
    int actual_tcp_id{-1};
    int actual_wobj_id{-1};
    DescPose actual_tcp_pose{};
};

struct PoseIkOptions {
    int expected_tcp_id{-1};
    int expected_wobj_id{-1};
    bool allow_orientation_fallback{true};
    int position_backoff_steps{4};
    bool has_reference_joints{false};
    std::array<double, 6> reference_joint_deg{};
};

struct PoseIkSolveResult {
    bool success{false};
    std::string message;
    int actual_tcp_id{-1};
    int actual_wobj_id{-1};
    std::array<double, 6> joint_target_deg{};
};

std::string ExpandUserPath(const std::string& path);
OrientationMode ParseOrientationMode(const std::string& mode);
std::string OrientationModeToString(OrientationMode mode);
ServoCartMode ParseServoCartMode(const std::string& mode);
std::string ServoCartModeToString(ServoCartMode mode);
int ServoCartModeToSdkMode(ServoCartMode mode);
geometry_msgs::msg::Pose BuildServoCartCommandPose(
    ServoCartMode mode,
    const geometry_msgs::msg::Pose& sampled_target_pose,
    const geometry_msgs::msg::Pose& previous_command_pose,
    bool previous_command_pose_valid);

tf2::Vector3 ApplyAxisMapping(const tf2::Vector3& input,
                              const std::array<int, 3>& axes,
                              const std::array<double, 3>& signs);
tf2::Vector3 ScaleVector(const tf2::Vector3& input, const tf2::Vector3& scale);
tf2::Quaternion QuaternionFromRPYDegrees(const tf2::Vector3& rpy_deg);
tf2::Quaternion ComputeMappedOrientation(const tf2::Quaternion& base_robot_orientation,
                                         const tf2::Quaternion& tracker_zero_orientation,
                                         const tf2::Quaternion& tracker_current_orientation,
                                         const tf2::Quaternion& tracker_to_robot_basis,
                                         OrientationMode mode);
double QuaternionAngularDistanceDegrees(const tf2::Quaternion& from, const tf2::Quaternion& to);
double PoseTranslationDistanceMillimeters(const geometry_msgs::msg::Pose& from,
                                          const geometry_msgs::msg::Pose& to);
bool IsLikelyJointBranchJump(const geometry_msgs::msg::Pose& previous_pose,
                             const geometry_msgs::msg::Pose& current_pose,
                             const std::array<double, 6>& previous_joint_target_deg,
                             const std::array<double, 6>& candidate_joint_target_deg,
                             double max_translation_step_mm,
                             double max_orientation_step_deg,
                             const std::array<double, 6>& min_joint_jump_deg);

geometry_msgs::msg::Pose ClampPoseTarget(const geometry_msgs::msg::Pose& candidate,
                                         const geometry_msgs::msg::Pose* previous,
                                         const tf2::Vector3& min_xyz,
                                         const tf2::Vector3& max_xyz,
                                         double workspace_radius,
                                         double max_linear_step_m,
                                         double max_angular_step_deg);

DescPose PoseToDescPose(const geometry_msgs::msg::Pose& pose);
geometry_msgs::msg::Pose DescPoseToPose(const DescPose& pose);

ControllerValidationResult ValidateControllerState(IFairinoRobot& robot,
                                                   int expected_tcp_id,
                                                   int expected_wobj_id);
PoseIkSolveResult SolvePoseTargetIK(IFairinoRobot& robot,
                                    const geometry_msgs::msg::PoseStamped& pose_target,
                                    const PoseIkOptions& options);

}  // namespace teleop_slave
