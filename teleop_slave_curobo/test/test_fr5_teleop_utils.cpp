#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <stdexcept>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "teleop_slave/cartesian_pose_stream_interpolator.hpp"
#include "teleop_slave/fairino_robot_interface.hpp"
#include "teleop_slave/fr5_teleop_utils.hpp"
#include "teleop_slave/joint_target_smoother.hpp"

namespace {

class FakeFairinoRobot : public IFairinoRobot {
public:
    errno_t RPC(const char*) override { return 0; }
    errno_t CloseRPC() override { return 0; }
    errno_t SetReConnectParam(bool, int, int) override { return 0; }
    errno_t ResetAllError() override { return 0; }
    errno_t Mode(int) override { return 0; }
    errno_t RobotEnable(uint8_t) override { return 0; }
    errno_t SetSpeed(int) override { return 0; }
    errno_t ServoMoveStart() override { return 0; }
    errno_t ServoMoveEnd() override { return 0; }
    errno_t MoveJ(JointPos* joint_pos, int, int, float, float, float, ExaxisPos*, float, uint8_t,
                  DescPose*) override {
        last_movej_joint_pos = *joint_pos;
        movej_calls++;
        return movej_ret;
    }
    errno_t ServoJ(JointPos*, ExaxisPos*, float, float, float, float, float, int) override { return 0; }
    errno_t ServoCart(int mode, DescPose* desc_pos, ExaxisPos, float[6], float, float, float, float, float) override {
        last_servo_cart_mode = mode;
        last_servo_cart_pose = *desc_pos;
        servo_cart_calls++;
        return servo_cart_ret;
    }

    errno_t GetActualJointPosDegree(uint8_t, JointPos* jpos) override {
        *jpos = actual_joint_pos;
        return get_joint_ret;
    }

    errno_t GetActualTCPPose(uint8_t, DescPose* desc_pos) override {
        *desc_pos = actual_tcp_pose;
        return get_tcp_pose_ret;
    }

    errno_t GetActualTCPNum(uint8_t, int* id) override {
        *id = actual_tcp_id;
        return get_tcp_num_ret;
    }

    errno_t GetActualWObjNum(uint8_t, int* id) override {
        *id = actual_wobj_id;
        return get_wobj_num_ret;
    }

    errno_t GetInverseKinHasSolution(int, DescPose* desc_pos, JointPos* ref_joints, uint8_t* result) override {
        last_desc_pose = *desc_pos;
        last_reference_joint_pos = *ref_joints;
        if (has_solution_fn) {
            *result = has_solution_fn(*desc_pos);
            return ik_has_solution_ret;
        }
        *result = has_solution;
        return ik_has_solution_ret;
    }

    errno_t GetInverseKinRef(int, DescPose*, JointPos* ref_joints, JointPos* joint_pos) override {
        last_reference_joint_pos = *ref_joints;
        *joint_pos = solved_joint_pos;
        return ik_ref_ret;
    }

    errno_t GetRobotErrorCode(int* maincode, int* subcode) override {
        *maincode = error_main;
        *subcode = error_sub;
        return 0;
    }

    JointPos actual_joint_pos{};
    JointPos solved_joint_pos{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    DescPose actual_tcp_pose{100.0, 200.0, 300.0, 10.0, 20.0, 30.0};
    DescPose last_desc_pose{};
    DescPose last_servo_cart_pose{};
    JointPos last_reference_joint_pos{};
    JointPos last_movej_joint_pos{};
    int last_servo_cart_mode{-1};
    int actual_tcp_id{3};
    int actual_wobj_id{1};
    uint8_t has_solution{1};
    int servo_cart_calls{0};
    int movej_calls{0};
    errno_t get_joint_ret{0};
    errno_t get_tcp_pose_ret{0};
    errno_t get_tcp_num_ret{0};
    errno_t get_wobj_num_ret{0};
    errno_t servo_cart_ret{0};
    errno_t movej_ret{0};
    errno_t ik_has_solution_ret{0};
    errno_t ik_ref_ret{0};
    int error_main{11};
    int error_sub{22};
    std::function<uint8_t(const DescPose&)> has_solution_fn;
};

geometry_msgs::msg::Pose MakePose(double x, double y, double z, double roll, double pitch, double yaw) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return pose;
}

}  // namespace

TEST(Fr5TeleopUtilsTest, AxisMappingAppliesSignedPermutation) {
    const tf2::Vector3 mapped = teleop_slave::ApplyAxisMapping(
        tf2::Vector3(1.0, 2.0, 3.0), {2, 0, 1}, {-1.0, 1.0, 1.0});

    EXPECT_DOUBLE_EQ(mapped.x(), -3.0);
    EXPECT_DOUBLE_EQ(mapped.y(), 1.0);
    EXPECT_DOUBLE_EQ(mapped.z(), 2.0);
}

TEST(Fr5TeleopUtilsTest, ComputeMappedOrientationSupportsPhasedModes) {
    tf2::Quaternion base;
    base.setRPY(0.0, 0.0, 0.0);

    tf2::Quaternion zero;
    zero.setRPY(0.0, 0.0, 0.0);

    tf2::Quaternion current;
    current.setRPY(0.3, -0.2, 0.5);

    tf2::Quaternion basis;
    basis.setRPY(0.0, 0.0, 0.0);

    const tf2::Quaternion position_only = teleop_slave::ComputeMappedOrientation(
        base, zero, current, basis, teleop_slave::OrientationMode::kPositionOnly);
    const tf2::Quaternion yaw_only = teleop_slave::ComputeMappedOrientation(
        base, zero, current, basis, teleop_slave::OrientationMode::kYawOnly);
    const tf2::Quaternion full = teleop_slave::ComputeMappedOrientation(
        base, zero, current, basis, teleop_slave::OrientationMode::kFull6Dof);

    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(base, position_only), 0.0, 1e-6);
    EXPECT_GT(teleop_slave::QuaternionAngularDistanceDegrees(base, yaw_only), 0.0);
    EXPECT_GT(teleop_slave::QuaternionAngularDistanceDegrees(yaw_only, full), 1.0);
}

TEST(Fr5TeleopUtilsTest, ComputeMappedOrientationKeepsAxesAlignedWithIdentityBasis) {
    const tf2::Quaternion base(0.0, 0.0, 0.0, 1.0);
    const tf2::Quaternion zero(0.0, 0.0, 0.0, 1.0);
    const tf2::Quaternion basis = teleop_slave::QuaternionFromRPYDegrees(
        tf2::Vector3(0.0, 0.0, 0.0));

    tf2::Quaternion tracker_x;
    tracker_x.setRPY(0.25, 0.0, 0.0);
    tf2::Quaternion expected_x;
    expected_x.setRPY(0.25, 0.0, 0.0);
    const tf2::Quaternion mapped_x = teleop_slave::ComputeMappedOrientation(
        base, zero, tracker_x, basis, teleop_slave::OrientationMode::kFull6Dof);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(mapped_x, expected_x), 0.0, 1e-6);

    tf2::Quaternion tracker_y;
    tracker_y.setRPY(0.0, 0.25, 0.0);
    tf2::Quaternion expected_y;
    expected_y.setRPY(0.0, 0.25, 0.0);
    const tf2::Quaternion mapped_y = teleop_slave::ComputeMappedOrientation(
        base, zero, tracker_y, basis, teleop_slave::OrientationMode::kFull6Dof);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(mapped_y, expected_y), 0.0, 1e-6);

    tf2::Quaternion tracker_z;
    tracker_z.setRPY(0.0, 0.0, 0.25);
    tf2::Quaternion expected_z;
    expected_z.setRPY(0.0, 0.0, 0.25);
    const tf2::Quaternion mapped_z = teleop_slave::ComputeMappedOrientation(
        base, zero, tracker_z, basis, teleop_slave::OrientationMode::kFull6Dof);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(mapped_z, expected_z), 0.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, ComputeMappedOrientationRemapsTrackerAxesToCandidateRobotBasis) {
    const tf2::Quaternion base(0.0, 0.0, 0.0, 1.0);
    const tf2::Quaternion zero(0.0, 0.0, 0.0, 1.0);
    const tf2::Quaternion basis = teleop_slave::QuaternionFromRPYDegrees(
        tf2::Vector3(0.0, -90.0, 180.0));

    tf2::Quaternion tracker_roll;
    tracker_roll.setRPY(0.25, 0.0, 0.0);
    tf2::Quaternion expected_yaw;
    expected_yaw.setRPY(0.0, 0.0, 0.25);
    const tf2::Quaternion mapped_roll = teleop_slave::ComputeMappedOrientation(
        base, zero, tracker_roll, basis, teleop_slave::OrientationMode::kFull6Dof);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(mapped_roll, expected_yaw), 0.0, 1e-6);

    tf2::Quaternion tracker_pitch;
    tracker_pitch.setRPY(0.0, 0.25, 0.0);
    tf2::Quaternion expected_neg_pitch;
    expected_neg_pitch.setRPY(0.0, -0.25, 0.0);
    const tf2::Quaternion mapped_pitch = teleop_slave::ComputeMappedOrientation(
        base, zero, tracker_pitch, basis, teleop_slave::OrientationMode::kFull6Dof);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(mapped_pitch, expected_neg_pitch), 0.0, 1e-6);

    tf2::Quaternion tracker_yaw;
    tracker_yaw.setRPY(0.0, 0.0, 0.25);
    tf2::Quaternion expected_roll;
    expected_roll.setRPY(0.25, 0.0, 0.0);
    const tf2::Quaternion mapped_yaw = teleop_slave::ComputeMappedOrientation(
        base, zero, tracker_yaw, basis, teleop_slave::OrientationMode::kFull6Dof);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(mapped_yaw, expected_roll), 0.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, ComputeMappedOrientationWorksWithNonIdentityZero) {
    const tf2::Quaternion base(0.0, 0.0, 0.0, 1.0);
    const tf2::Quaternion basis = teleop_slave::QuaternionFromRPYDegrees(
        tf2::Vector3(0.0, 0.0, 0.0));

    // Clutch (zero) at 45 deg yaw — non-identity reference orientation
    tf2::Quaternion zero;
    zero.setRPY(0.0, 0.0, M_PI / 4.0);

    // Rotate tracker by 0.25 rad roll in tracker LOCAL frame:
    // q_current = q_zero * Rx_local(0.25)
    tf2::Quaternion local_roll;
    local_roll.setRPY(0.25, 0.0, 0.0);
    tf2::Quaternion current = zero * local_roll;

    // With identity basis, body-frame roll should map to robot roll
    tf2::Quaternion expected;
    expected.setRPY(0.25, 0.0, 0.0);

    const tf2::Quaternion result = teleop_slave::ComputeMappedOrientation(
        base, zero, current, basis, teleop_slave::OrientationMode::kFull6Dof);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(result, expected), 0.0, 0.5);
}

TEST(Fr5TeleopUtilsTest, IsLikelyJointBranchJumpDetectsLargeJ123JumpOnSmallPoseStep) {
    const geometry_msgs::msg::Pose previous_pose = MakePose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0);
    const geometry_msgs::msg::Pose current_pose = MakePose(0.104, 0.2, 0.3, 0.0, 0.0, 0.02);
    const std::array<double, 6> previous_joint_target_deg{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const std::array<double, 6> candidate_joint_target_deg{6.5, 1.0, 2.0, 3.0, 4.0, 5.0};
    const std::array<double, 6> min_joint_jump_deg{5.0, 5.0, 6.0, 0.0, 0.0, 0.0};

    EXPECT_TRUE(teleop_slave::IsLikelyJointBranchJump(
        previous_pose,
        current_pose,
        previous_joint_target_deg,
        candidate_joint_target_deg,
        8.0,
        10.0,
        min_joint_jump_deg));
}

TEST(Fr5TeleopUtilsTest, IsLikelyJointBranchJumpIgnoresLargePoseSteps) {
    const geometry_msgs::msg::Pose previous_pose = MakePose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0);
    const geometry_msgs::msg::Pose current_pose = MakePose(0.12, 0.2, 0.3, 0.0, 0.0, 0.02);
    const std::array<double, 6> previous_joint_target_deg{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const std::array<double, 6> candidate_joint_target_deg{6.5, 1.0, 2.0, 3.0, 4.0, 5.0};
    const std::array<double, 6> min_joint_jump_deg{5.0, 5.0, 6.0, 0.0, 0.0, 0.0};

    EXPECT_FALSE(teleop_slave::IsLikelyJointBranchJump(
        previous_pose,
        current_pose,
        previous_joint_target_deg,
        candidate_joint_target_deg,
        8.0,
        10.0,
        min_joint_jump_deg));
}

TEST(Fr5TeleopUtilsTest, ClampPoseTargetLimitsStepAndWorkspace) {
    geometry_msgs::msg::Pose previous = MakePose(0.1, 0.0, 0.4, 0.0, 0.0, 0.0);
    geometry_msgs::msg::Pose candidate = MakePose(0.5, 0.5, 1.5, 0.0, 0.0, 1.5);

    const geometry_msgs::msg::Pose clamped = teleop_slave::ClampPoseTarget(
        candidate,
        &previous,
        tf2::Vector3(-0.2, -0.2, 0.1),
        tf2::Vector3(0.4, 0.4, 0.8),
        0.7,
        0.05,
        10.0);

    const tf2::Vector3 clamped_pos(clamped.position.x, clamped.position.y, clamped.position.z);
    EXPECT_LE((clamped_pos - tf2::Vector3(previous.position.x, previous.position.y, previous.position.z)).length(),
              0.050001);
    EXPECT_LE(clamped_pos.length(), 0.700001);
    EXPECT_GE(clamped.position.z, 0.1);
}

TEST(Fr5TeleopUtilsTest, ServoCartModeParsesAndMapsToSdkMode) {
    const auto incremental = teleop_slave::ParseServoCartMode("incremental_base");
    const auto absolute = teleop_slave::ParseServoCartMode("absolute_base");

    EXPECT_EQ(incremental, teleop_slave::ServoCartMode::kIncrementalBase);
    EXPECT_EQ(absolute, teleop_slave::ServoCartMode::kAbsoluteBase);
    EXPECT_EQ(teleop_slave::ServoCartModeToString(incremental), "incremental_base");
    EXPECT_EQ(teleop_slave::ServoCartModeToString(absolute), "absolute_base");
    EXPECT_EQ(teleop_slave::ServoCartModeToSdkMode(incremental), 1);
    EXPECT_EQ(teleop_slave::ServoCartModeToSdkMode(absolute), 0);
    EXPECT_THROW(teleop_slave::ParseServoCartMode("tool_relative"), std::invalid_argument);
}

TEST(Fr5TeleopUtilsTest, CartesianPoseStreamInterpolatorSeedsFromActualPose) {
    teleop_slave::CartesianPoseStreamInterpolator interpolator;
    const auto now = std::chrono::steady_clock::now();
    const geometry_msgs::msg::Pose actual_pose = MakePose(0.2, -0.1, 0.45, 0.1, -0.2, 0.3);

    interpolator.reset(actual_pose, now);
    const auto sampled = interpolator.sample(now);

    EXPECT_NEAR(sampled.position.x, actual_pose.position.x, 1e-9);
    EXPECT_NEAR(sampled.position.y, actual_pose.position.y, 1e-9);
    EXPECT_NEAR(sampled.position.z, actual_pose.position.z, 1e-9);

    tf2::Quaternion sampled_q;
    tf2::Quaternion actual_q;
    tf2::fromMsg(sampled.orientation, sampled_q);
    tf2::fromMsg(actual_pose.orientation, actual_q);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(sampled_q, actual_q), 0.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, BuildServoCartCommandPoseReturnsIdentityForFirstIncrementalCommand) {
    const geometry_msgs::msg::Pose target_pose = MakePose(0.2, -0.1, 0.45, 0.1, -0.2, 0.3);
    const geometry_msgs::msg::Pose previous_pose = MakePose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0);

    const geometry_msgs::msg::Pose command_pose = teleop_slave::BuildServoCartCommandPose(
        teleop_slave::ServoCartMode::kIncrementalBase,
        target_pose,
        previous_pose,
        false);

    EXPECT_NEAR(command_pose.position.x, 0.0, 1e-9);
    EXPECT_NEAR(command_pose.position.y, 0.0, 1e-9);
    EXPECT_NEAR(command_pose.position.z, 0.0, 1e-9);

    tf2::Quaternion command_q;
    tf2::Quaternion identity_q;
    tf2::fromMsg(command_pose.orientation, command_q);
    identity_q.setRPY(0.0, 0.0, 0.0);
    EXPECT_NEAR(
        teleop_slave::QuaternionAngularDistanceDegrees(identity_q, command_q), 0.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, BuildServoCartCommandPoseReturnsIncrementalDeltaForFollowupCommand) {
    const geometry_msgs::msg::Pose from_pose = MakePose(0.1, -0.2, 0.3, 0.0, 0.0, 0.0);
    const geometry_msgs::msg::Pose to_pose = MakePose(0.12, -0.18, 0.29, 0.0, 0.0, 0.1);

    const geometry_msgs::msg::Pose command_pose = teleop_slave::BuildServoCartCommandPose(
        teleop_slave::ServoCartMode::kIncrementalBase,
        to_pose,
        from_pose,
        true);

    EXPECT_NEAR(command_pose.position.x, 0.02, 1e-9);
    EXPECT_NEAR(command_pose.position.y, 0.02, 1e-9);
    EXPECT_NEAR(command_pose.position.z, -0.01, 1e-9);

    tf2::Quaternion identity_q;
    tf2::Quaternion delta_q;
    identity_q.setRPY(0.0, 0.0, 0.0);
    tf2::fromMsg(command_pose.orientation, delta_q);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(identity_q, delta_q),
                0.1 * 180.0 / M_PI,
                1e-6);
}

TEST(Fr5TeleopUtilsTest, BuildServoCartCommandPoseReturnsAbsoluteTargetForAbsoluteMode) {
    const geometry_msgs::msg::Pose target_pose = MakePose(0.2, -0.1, 0.45, 0.1, -0.2, 0.3);
    const geometry_msgs::msg::Pose previous_pose = MakePose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0);

    const geometry_msgs::msg::Pose command_pose = teleop_slave::BuildServoCartCommandPose(
        teleop_slave::ServoCartMode::kAbsoluteBase,
        target_pose,
        previous_pose,
        true);

    EXPECT_NEAR(command_pose.position.x, target_pose.position.x, 1e-9);
    EXPECT_NEAR(command_pose.position.y, target_pose.position.y, 1e-9);
    EXPECT_NEAR(command_pose.position.z, target_pose.position.z, 1e-9);

    tf2::Quaternion command_q;
    tf2::Quaternion target_q;
    tf2::fromMsg(command_pose.orientation, command_q);
    tf2::fromMsg(target_pose.orientation, target_q);
    EXPECT_NEAR(
        teleop_slave::QuaternionAngularDistanceDegrees(command_q, target_q), 0.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, CartesianPoseStreamInterpolatorBlendsFirstTargetOverNominalPeriod) {
    teleop_slave::CartesianPoseStreamInterpolator interpolator;
    const auto now = std::chrono::steady_clock::now();
    const geometry_msgs::msg::Pose actual_pose = MakePose(0.0, 0.0, 0.5, 0.0, 0.0, 0.0);
    const geometry_msgs::msg::Pose target_pose = MakePose(0.1, 0.0, 0.5, 0.0, 0.0, M_PI / 2.0);

    interpolator.reset(actual_pose, now);
    interpolator.setTarget(target_pose, now, 0.02, true);

    const auto immediate = interpolator.sample(now);
    const auto final = interpolator.sample(now + std::chrono::milliseconds(20));

    EXPECT_NEAR(immediate.position.x, actual_pose.position.x, 1e-9);
    EXPECT_NEAR(immediate.position.y, actual_pose.position.y, 1e-9);
    EXPECT_NEAR(final.position.x, target_pose.position.x, 1e-9);
    EXPECT_NEAR(final.position.y, target_pose.position.y, 1e-9);

    tf2::Quaternion immediate_q;
    tf2::Quaternion actual_q;
    tf2::Quaternion final_q;
    tf2::Quaternion target_q;
    tf2::fromMsg(immediate.orientation, immediate_q);
    tf2::fromMsg(actual_pose.orientation, actual_q);
    tf2::fromMsg(final.orientation, final_q);
    tf2::fromMsg(target_pose.orientation, target_q);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(immediate_q, actual_q), 0.0, 1e-6);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(final_q, target_q), 0.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, CartesianPoseStreamInterpolatorUsesLinearPositionAndSlerpOrientation) {
    teleop_slave::CartesianPoseStreamInterpolator interpolator;
    const auto now = std::chrono::steady_clock::now();
    const geometry_msgs::msg::Pose from_pose = MakePose(0.0, 0.0, 0.4, 0.0, 0.0, 0.0);
    const geometry_msgs::msg::Pose to_pose = MakePose(0.2, -0.1, 0.6, 0.0, 0.0, M_PI / 2.0);

    interpolator.reset(from_pose, now);
    interpolator.setTarget(to_pose, now, 0.02, true);
    const auto halfway = interpolator.sample(now + std::chrono::milliseconds(10));

    EXPECT_NEAR(halfway.position.x, 0.1, 1e-9);
    EXPECT_NEAR(halfway.position.y, -0.05, 1e-9);
    EXPECT_NEAR(halfway.position.z, 0.5, 1e-9);

    tf2::Quaternion halfway_q;
    tf2::Quaternion from_q;
    tf2::fromMsg(halfway.orientation, halfway_q);
    tf2::fromMsg(from_pose.orientation, from_q);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(from_q, halfway_q), 45.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, ComputeIncrementalPoseDeltaBaseFrameReturnsExpectedTranslationAndRotation) {
    const geometry_msgs::msg::Pose from_pose = MakePose(0.1, -0.2, 0.3, 0.0, 0.0, 0.0);
    const geometry_msgs::msg::Pose to_pose = MakePose(0.12, -0.18, 0.29, 0.0, 0.0, 0.1);

    const geometry_msgs::msg::Pose delta =
        teleop_slave::ComputeIncrementalPoseDeltaBaseFrame(from_pose, to_pose);

    EXPECT_NEAR(delta.position.x, 0.02, 1e-9);
    EXPECT_NEAR(delta.position.y, 0.02, 1e-9);
    EXPECT_NEAR(delta.position.z, -0.01, 1e-9);

    tf2::Quaternion identity_q;
    identity_q.setRPY(0.0, 0.0, 0.0);
    tf2::Quaternion delta_q;
    tf2::fromMsg(delta.orientation, delta_q);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(identity_q, delta_q),
                0.1 * 180.0 / M_PI,
                1e-6);
}

TEST(Fr5TeleopUtilsTest, ComputeIncrementalPoseDeltaBaseFrameReturnsIdentityForEqualPoses) {
    const geometry_msgs::msg::Pose pose = MakePose(0.2, 0.1, 0.4, 0.2, -0.1, 0.3);

    const geometry_msgs::msg::Pose delta =
        teleop_slave::ComputeIncrementalPoseDeltaBaseFrame(pose, pose);

    EXPECT_NEAR(delta.position.x, 0.0, 1e-9);
    EXPECT_NEAR(delta.position.y, 0.0, 1e-9);
    EXPECT_NEAR(delta.position.z, 0.0, 1e-9);

    tf2::Quaternion delta_q;
    tf2::Quaternion identity_q;
    tf2::fromMsg(delta.orientation, delta_q);
    identity_q.setRPY(0.0, 0.0, 0.0);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(identity_q, delta_q), 0.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, JointTargetSmootherDeadbandHoldsSmallJ123Jitter) {
    teleop_slave::JointTargetSmoother smoother;
    smoother.configure(
        {0.08, 0.08, 0.08, 0.12, 0.12, 0.12},
        {35.0, 35.0, 40.0, 120.0, 120.0, 150.0},
        {1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6});
    smoother.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    const auto result = smoother.update({0.05, -0.06, 0.07, 0.0, 0.0, 0.0}, 0.02);

    EXPECT_FALSE(result.held_due_to_invalid_dt);
    EXPECT_DOUBLE_EQ(result.filtered_target_deg[0], 0.0);
    EXPECT_DOUBLE_EQ(result.filtered_target_deg[1], 0.0);
    EXPECT_DOUBLE_EQ(result.filtered_target_deg[2], 0.0);
}

TEST(Fr5TeleopUtilsTest, JointTargetSmootherRespectsPerJointVelocityLimits) {
    teleop_slave::JointTargetSmoother smoother;
    smoother.configure(
        {0.08, 0.08, 0.08, 0.12, 0.12, 0.12},
        {35.0, 35.0, 40.0, 120.0, 120.0, 150.0},
        {1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6});
    smoother.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    const auto result = smoother.update({10.0, -10.0, 10.0, 10.0, -10.0, 10.0}, 0.02);

    EXPECT_NEAR(result.filtered_target_deg[0], 0.7, 1e-9);
    EXPECT_NEAR(result.filtered_target_deg[1], -0.7, 1e-9);
    EXPECT_NEAR(result.filtered_target_deg[2], 0.8, 1e-9);
    EXPECT_NEAR(result.filtered_target_deg[3], 2.4, 1e-9);
    EXPECT_NEAR(result.filtered_target_deg[4], -2.4, 1e-9);
    EXPECT_NEAR(result.filtered_target_deg[5], 3.0, 1e-9);
}

TEST(Fr5TeleopUtilsTest, JointTargetSmootherHoldsLastTargetForInvalidDt) {
    teleop_slave::JointTargetSmoother smoother;
    smoother.configure(
        {0.08, 0.08, 0.08, 0.12, 0.12, 0.12},
        {35.0, 35.0, 40.0, 120.0, 120.0, 150.0},
        {1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6});
    smoother.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    const auto seeded = smoother.update({2.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.02);
    const auto held = smoother.update({10.0, 10.0, 10.0, 10.0, 10.0, 10.0}, 0.0);

    EXPECT_FALSE(seeded.held_due_to_invalid_dt);
    EXPECT_TRUE(held.held_due_to_invalid_dt);
    EXPECT_NEAR(held.filtered_target_deg[0], seeded.filtered_target_deg[0], 1e-9);
    EXPECT_NEAR(held.filtered_target_deg[1], seeded.filtered_target_deg[1], 1e-9);
    EXPECT_NEAR(held.filtered_target_deg[2], seeded.filtered_target_deg[2], 1e-9);
}

TEST(Fr5TeleopUtilsTest, JointTargetSmootherRespectsPerJointAccelerationLimits) {
    teleop_slave::JointTargetSmoother smoother;
    smoother.configure(
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0},
        {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
    smoother.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    const auto first = smoother.update({10.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.02);
    const auto second = smoother.update({10.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.02);

    EXPECT_NEAR(first.filtered_target_deg[0], 0.04, 1e-9);
    EXPECT_NEAR(second.filtered_target_deg[0], 0.12, 1e-9);
}

TEST(Fr5TeleopUtilsTest, PoseConversionRoundTripsMetersAndEulerPose) {
    const geometry_msgs::msg::Pose input = MakePose(0.123, -0.456, 0.789, 0.2, -0.1, 0.3);

    const DescPose desc = teleop_slave::PoseToDescPose(input);
    EXPECT_NEAR(desc.tran.x, 123.0, 1e-6);
    EXPECT_NEAR(desc.tran.y, -456.0, 1e-6);
    EXPECT_NEAR(desc.tran.z, 789.0, 1e-6);

    const geometry_msgs::msg::Pose round_trip = teleop_slave::DescPoseToPose(desc);
    EXPECT_NEAR(round_trip.position.x, input.position.x, 1e-6);
    EXPECT_NEAR(round_trip.position.y, input.position.y, 1e-6);
    EXPECT_NEAR(round_trip.position.z, input.position.z, 1e-6);
    EXPECT_NEAR(teleop_slave::QuaternionAngularDistanceDegrees(
                    tf2::Quaternion(input.orientation.x, input.orientation.y,
                                    input.orientation.z, input.orientation.w),
                    tf2::Quaternion(round_trip.orientation.x, round_trip.orientation.y,
                                    round_trip.orientation.z, round_trip.orientation.w)),
                0.0,
                1e-4);
}

TEST(Fr5TeleopUtilsTest, SolvePoseTargetIKUsesControllerFramesAndReturnsJointTargets) {
    FakeFairinoRobot robot;
    geometry_msgs::msg::PoseStamped target;
    target.pose = MakePose(0.1, -0.2, 0.3, 0.0, 0.0, 0.5);

    teleop_slave::PoseIkOptions options;
    options.expected_tcp_id = 3;
    options.expected_wobj_id = 1;

    const teleop_slave::PoseIkSolveResult result =
        teleop_slave::SolvePoseTargetIK(robot, target, options);

    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.actual_tcp_id, 3);
    EXPECT_EQ(result.actual_wobj_id, 1);
    EXPECT_DOUBLE_EQ(result.joint_target_deg[0], 1.0);
    EXPECT_NEAR(robot.last_desc_pose.tran.x, 100.0, 1e-6);
    EXPECT_NEAR(robot.last_desc_pose.tran.y, -200.0, 1e-6);
    EXPECT_NEAR(robot.last_desc_pose.tran.z, 300.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, SolvePoseTargetIKFailsOnFrameMismatchOrNoSolution) {
    FakeFairinoRobot robot;
    geometry_msgs::msg::PoseStamped target;
    target.pose = MakePose(0.1, 0.0, 0.2, 0.0, 0.0, 0.0);

    teleop_slave::PoseIkOptions mismatch_options;
    mismatch_options.expected_tcp_id = 9;
    const teleop_slave::PoseIkSolveResult mismatch =
        teleop_slave::SolvePoseTargetIK(robot, target, mismatch_options);
    EXPECT_FALSE(mismatch.success);
    EXPECT_NE(mismatch.message.find("mismatch"), std::string::npos);

    robot.has_solution = 0;
    teleop_slave::PoseIkOptions options;
    const teleop_slave::PoseIkSolveResult no_solution =
        teleop_slave::SolvePoseTargetIK(robot, target, options);
    EXPECT_FALSE(no_solution.success);
    EXPECT_NE(no_solution.message.find("No IK solution"), std::string::npos);
}

TEST(Fr5TeleopUtilsTest, SolvePoseTargetIKUsesOrientationFallbackWhenNeeded) {
    FakeFairinoRobot robot;
    geometry_msgs::msg::PoseStamped target;
    target.pose = MakePose(0.1, -0.2, 0.3, 0.5, -0.4, 0.8);

    robot.has_solution_fn = [&](const DescPose& desc_pose) {
        const bool same_orientation =
            std::abs(desc_pose.rpy.rx - robot.actual_tcp_pose.rpy.rx) < 1e-6 &&
            std::abs(desc_pose.rpy.ry - robot.actual_tcp_pose.rpy.ry) < 1e-6 &&
            std::abs(desc_pose.rpy.rz - robot.actual_tcp_pose.rpy.rz) < 1e-6;
        return same_orientation ? 1 : 0;
    };

    const teleop_slave::PoseIkSolveResult result =
        teleop_slave::SolvePoseTargetIK(robot, target, teleop_slave::PoseIkOptions{});

    ASSERT_TRUE(result.success);
    EXPECT_NE(result.message.find("orientation fallback"), std::string::npos);
}

TEST(Fr5TeleopUtilsTest, SolvePoseTargetIKBacksOffPositionTowardCurrentTcp) {
    FakeFairinoRobot robot;
    geometry_msgs::msg::PoseStamped target;
    target.pose = MakePose(1.0, -0.2, 0.3, 0.0, 0.0, 0.0);

    robot.has_solution_fn = [](const DescPose& desc_pose) {
        return desc_pose.tran.x <= 400.0 ? 1 : 0;
    };

    teleop_slave::PoseIkOptions options;
    options.allow_orientation_fallback = false;
    options.position_backoff_steps = 4;

    const teleop_slave::PoseIkSolveResult result =
        teleop_slave::SolvePoseTargetIK(robot, target, options);

    ASSERT_TRUE(result.success);
    EXPECT_NE(result.message.find("position backoff"), std::string::npos);
    EXPECT_NEAR(robot.last_desc_pose.tran.x, 325.0, 1e-6);
}

TEST(Fr5TeleopUtilsTest, SolvePoseTargetIKCanUsePreferredReferenceJointSeed) {
    FakeFairinoRobot robot;
    geometry_msgs::msg::PoseStamped target;
    target.pose = MakePose(0.1, -0.1, 0.2, 0.0, 0.0, 0.0);

    teleop_slave::PoseIkOptions options;
    options.has_reference_joints = true;
    options.reference_joint_deg = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};

    const teleop_slave::PoseIkSolveResult result =
        teleop_slave::SolvePoseTargetIK(robot, target, options);

    ASSERT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(robot.last_reference_joint_pos.jPos[0], 10.0);
    EXPECT_DOUBLE_EQ(robot.last_reference_joint_pos.jPos[1], 20.0);
    EXPECT_DOUBLE_EQ(robot.last_reference_joint_pos.jPos[2], 30.0);
}
