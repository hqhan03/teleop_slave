#include <gtest/gtest.h>

#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "teleop_slave/fairino_robot_interface.hpp"
#include "teleop_slave/fr5_teleop_utils.hpp"

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
    errno_t ServoJ(JointPos*, ExaxisPos*, float, float, float, float, float, int) override { return 0; }

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

    errno_t GetInverseKinHasSolution(int, DescPose* desc_pos, JointPos*, uint8_t* result) override {
        last_desc_pose = *desc_pos;
        if (has_solution_fn) {
            *result = has_solution_fn(*desc_pos);
            return ik_has_solution_ret;
        }
        *result = has_solution;
        return ik_has_solution_ret;
    }

    errno_t GetInverseKinRef(int, DescPose*, JointPos*, JointPos* joint_pos) override {
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
    int actual_tcp_id{3};
    int actual_wobj_id{1};
    uint8_t has_solution{1};
    errno_t get_joint_ret{0};
    errno_t get_tcp_pose_ret{0};
    errno_t get_tcp_num_ret{0};
    errno_t get_wobj_num_ret{0};
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
