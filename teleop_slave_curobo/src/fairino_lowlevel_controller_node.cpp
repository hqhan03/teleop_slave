#include "teleop_slave/fairino_lowlevel_controller_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "teleop_slave/fr5_teleop_utils.hpp"

using namespace std::chrono_literals;

namespace {

using JointArray = teleop_slave::JointTargetSmoother::JointArray;
constexpr double kStartupTranslationWarnThresholdMm = 20.0;
constexpr double kStartupOrientationWarnThresholdDeg = 5.0;

JointArray JointPosToArray(const JointPos& joint_pos) {
    JointArray joints_deg{};
    for (std::size_t i = 0; i < joints_deg.size(); ++i) {
        joints_deg[i] = joint_pos.jPos[i];
    }
    return joints_deg;
}

std::vector<double> JointArrayToVector(const JointArray& joints_deg) {
    return std::vector<double>(joints_deg.begin(), joints_deg.end());
}

JointArray VectorToJointArray(const std::vector<double>& values, const char* param_name) {
    if (values.size() != JointArray{}.size()) {
        std::ostringstream stream;
        stream << param_name << " must have exactly " << JointArray{}.size() << " entries";
        throw std::runtime_error(stream.str());
    }

    JointArray output{};
    std::copy(values.begin(), values.end(), output.begin());
    return output;
}

std::string BuildFairinoErrorDetail(IFairinoRobot* robot, errno_t ret) {
    int maincode = 0;
    int subcode = 0;
    if (robot != nullptr) {
        robot->GetRobotErrorCode(&maincode, &subcode);
    }

    std::ostringstream stream;
    stream << "ret=" << ret << ", main=" << maincode << ", sub=" << subcode;
    return stream.str();
}

std::string DescribeDescPose(const DescPose& pose) {
    std::ostringstream stream;
    stream << "[" << pose.tran.x << ", " << pose.tran.y << ", " << pose.tran.z
           << "; " << pose.rpy.rx << ", " << pose.rpy.ry << ", " << pose.rpy.rz << "]";
    return stream.str();
}

std::string DescribeMaybeDescPose(const DescPose& pose, bool valid) {
    if (!valid) {
        return "unavailable";
    }
    return DescribeDescPose(pose);
}

}  // namespace

FairinoControllerNode::FairinoControllerNode()
    : Node("fairino_lowlevel_controller_node") {
    this->declare_parameter("robot_ip", "192.168.58.2");
    this->declare_parameter("dummy_mode", false);
    this->declare_parameter("stream_command_mode", "servo_cart");
    this->declare_parameter("global_speed_percent", 30);
    this->declare_parameter("expected_tcp_id", -1);
    this->declare_parameter("expected_wobj_id", -1);
    this->declare_parameter("ik_failure_limit", 5);
    this->declare_parameter("ik_failure_timeout_sec", 0.75);
    this->declare_parameter("allow_orientation_fallback", true);
    this->declare_parameter("ik_position_backoff_steps", 4);
    this->declare_parameter("pose_target_timeout_sec", 0.30);
    this->declare_parameter("servo_cmd_period_sec", 0.008);
    this->declare_parameter("pose_target_nominal_period_sec", 0.02);
    this->declare_parameter("servocart_mode", "incremental_base");
    this->declare_parameter("servocart_filter_t", 0.0);
    this->declare_parameter("servocart_gain", 0.0);
    this->declare_parameter("servoj_max_step_deg", 0.8);
    this->declare_parameter("servoj_filter_t", 0.0);
    this->declare_parameter("servoj_gain", 0.0);
    this->declare_parameter("movej_cmd_period_sec", 0.08);
    this->declare_parameter("movej_vel_percent", 100.0);
    this->declare_parameter("movej_acc_percent", 100.0);
    this->declare_parameter("movej_ovl_percent", 100.0);
    this->declare_parameter("movej_blend_t_ms", 80.0);
    this->declare_parameter("movej_command_deadband_deg",
                            std::vector<double>{0.5, 0.7, 0.7, 0.5, 0.5, 0.5});
    this->declare_parameter("servoj_output_deadband_deg",
                            std::vector<double>{0.25, 0.35, 0.35, 0.0, 0.0, 0.0});
    this->declare_parameter("enable_servoj_tracking_guard", true);
    this->declare_parameter("servoj_tracking_error_hold_deg",
                            std::vector<double>{0.6, 1.2, 1.2, 0.0, 0.0, 0.0});
    this->declare_parameter("prefer_stream_target_for_ik_seed", true);
    this->declare_parameter("enable_joint_debug_csv", false);
    this->declare_parameter("joint_debug_csv_path", "/tmp/fairino_joint_debug.csv");
    this->declare_parameter("joint_debug_csv_decimation", 1);
    this->declare_parameter("enable_joint_continuity_guard", true);
    this->declare_parameter("joint_continuity_guard_pose_step_mm", 3.0);
    this->declare_parameter("joint_continuity_guard_orientation_step_deg", 5.0);
    this->declare_parameter("enable_joint_reversal_guard", true);
    this->declare_parameter("joint_reversal_hold_cycles", 2);
    this->declare_parameter("servoj_failure_limit", 3);
    this->declare_parameter("interpolate_stream", true);
    this->declare_parameter("joint_deadband_deg",
                            std::vector<double>{0.08, 0.08, 0.08, 0.12, 0.12, 0.12});
    this->declare_parameter("joint_velocity_limit_deg_s",
                            std::vector<double>{35.0, 35.0, 40.0, 120.0, 120.0, 150.0});
    this->declare_parameter("joint_acceleration_limit_deg_s2",
                            std::vector<double>{120.0, 120.0, 160.0, 400.0, 400.0, 600.0});
    this->declare_parameter("joint_continuity_guard_jump_deg",
                            std::vector<double>{6.0, 6.0, 8.0, 0.0, 0.0, 0.0});
    this->declare_parameter("joint_reversal_guard_step_deg",
                            std::vector<double>{0.9, 0.9, 1.2, 0.0, 0.0, 0.0});

    robot_ip_ = this->get_parameter("robot_ip").as_string();
    dummy_mode_ = this->get_parameter("dummy_mode").as_bool();
    const std::string stream_command_mode =
        this->get_parameter("stream_command_mode").as_string();
    if (stream_command_mode == "servo_cart") {
        stream_command_mode_ = StreamCommandMode::kServoCart;
    } else if (stream_command_mode == "servo_j") {
        stream_command_mode_ = StreamCommandMode::kServoJ;
    } else if (stream_command_mode == "move_j") {
        stream_command_mode_ = StreamCommandMode::kMoveJ;
    } else {
        throw std::runtime_error(
            "stream_command_mode must be one of 'servo_cart', 'servo_j', or 'move_j'");
    }
    global_speed_percent_ = this->get_parameter("global_speed_percent").as_int();
    servo_cmd_period_sec_ = this->get_parameter("servo_cmd_period_sec").as_double();
    pose_target_nominal_period_sec_ =
        this->get_parameter("pose_target_nominal_period_sec").as_double();
    servocart_mode_ = teleop_slave::ParseServoCartMode(
        this->get_parameter("servocart_mode").as_string());
    servocart_filter_t_ = this->get_parameter("servocart_filter_t").as_double();
    servocart_gain_ = this->get_parameter("servocart_gain").as_double();
    servoj_max_step_deg_ = this->get_parameter("servoj_max_step_deg").as_double();
    servoj_filter_t_ = this->get_parameter("servoj_filter_t").as_double();
    servoj_gain_ = this->get_parameter("servoj_gain").as_double();
    movej_cmd_period_sec_ = this->get_parameter("movej_cmd_period_sec").as_double();
    movej_vel_percent_ = this->get_parameter("movej_vel_percent").as_double();
    movej_acc_percent_ = this->get_parameter("movej_acc_percent").as_double();
    movej_ovl_percent_ = this->get_parameter("movej_ovl_percent").as_double();
    movej_blend_t_ms_ = this->get_parameter("movej_blend_t_ms").as_double();
    movej_command_deadband_deg_ = VectorToJointArray(
        this->get_parameter("movej_command_deadband_deg").as_double_array(),
        "movej_command_deadband_deg");
    servoj_output_deadband_deg_ = VectorToJointArray(
        this->get_parameter("servoj_output_deadband_deg").as_double_array(),
        "servoj_output_deadband_deg");
    enable_servoj_tracking_guard_ =
        this->get_parameter("enable_servoj_tracking_guard").as_bool();
    servoj_tracking_error_hold_deg_ = VectorToJointArray(
        this->get_parameter("servoj_tracking_error_hold_deg").as_double_array(),
        "servoj_tracking_error_hold_deg");
    prefer_stream_target_for_ik_seed_ =
        this->get_parameter("prefer_stream_target_for_ik_seed").as_bool();
    enable_joint_debug_csv_ =
        this->get_parameter("enable_joint_debug_csv").as_bool();
    joint_debug_csv_path_ = teleop_slave::ExpandUserPath(
        this->get_parameter("joint_debug_csv_path").as_string());
    joint_debug_csv_decimation_ =
        std::max<int>(1, static_cast<int>(
            this->get_parameter("joint_debug_csv_decimation").as_int()));
    enable_joint_continuity_guard_ =
        this->get_parameter("enable_joint_continuity_guard").as_bool();
    joint_continuity_guard_pose_step_mm_ =
        this->get_parameter("joint_continuity_guard_pose_step_mm").as_double();
    joint_continuity_guard_orientation_step_deg_ =
        this->get_parameter("joint_continuity_guard_orientation_step_deg").as_double();
    enable_joint_reversal_guard_ =
        this->get_parameter("enable_joint_reversal_guard").as_bool();
    joint_reversal_hold_cycles_ =
        this->get_parameter("joint_reversal_hold_cycles").as_int();
    servoj_failure_limit_ = this->get_parameter("servoj_failure_limit").as_int();
    joint_deadband_deg_ = VectorToJointArray(
        this->get_parameter("joint_deadband_deg").as_double_array(), "joint_deadband_deg");
    joint_velocity_limit_deg_s_ = VectorToJointArray(
        this->get_parameter("joint_velocity_limit_deg_s").as_double_array(),
        "joint_velocity_limit_deg_s");
    joint_acceleration_limit_deg_s2_ = VectorToJointArray(
        this->get_parameter("joint_acceleration_limit_deg_s2").as_double_array(),
        "joint_acceleration_limit_deg_s2");
    joint_continuity_guard_jump_deg_ = VectorToJointArray(
        this->get_parameter("joint_continuity_guard_jump_deg").as_double_array(),
        "joint_continuity_guard_jump_deg");
    joint_reversal_guard_step_deg_ = VectorToJointArray(
        this->get_parameter("joint_reversal_guard_step_deg").as_double_array(),
        "joint_reversal_guard_step_deg");
    joint_target_smoother_.configure(
        joint_deadband_deg_, joint_velocity_limit_deg_s_, joint_acceleration_limit_deg_s2_);

    if (enable_joint_debug_csv_) {
        try {
            const std::filesystem::path csv_path(joint_debug_csv_path_);
            if (csv_path.has_parent_path()) {
                std::filesystem::create_directories(csv_path.parent_path());
            }
            joint_debug_csv_stream_.open(joint_debug_csv_path_, std::ios::out | std::ios::trunc);
            if (!joint_debug_csv_stream_.is_open()) {
                throw std::runtime_error("open failed");
            }
            joint_debug_csv_stream_
                << "stamp_sec,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw,"
                << "raw_j1,raw_j2,raw_j3,raw_j4,raw_j5,raw_j6,"
                << "guarded_j1,guarded_j2,guarded_j3,guarded_j4,guarded_j5,guarded_j6,"
                << "filtered_j1,filtered_j2,filtered_j3,filtered_j4,filtered_j5,filtered_j6,"
                << "actual_j1,actual_j2,actual_j3,actual_j4,actual_j5,actual_j6,"
                << "actual_age_sec,continuity_j1,continuity_j2,continuity_j3,"
                << "reversal_j1,reversal_j2,reversal_j3,reversal_detected,"
                << "continuity_pose_step_mm,continuity_pose_step_deg,continuity_raw_jump_deg\n";
            joint_debug_csv_stream_.flush();
        } catch (const std::exception& ex) {
            enable_joint_debug_csv_ = false;
            RCLCPP_WARN(this->get_logger(),
                        "Failed to enable joint debug CSV at %s: %s",
                        joint_debug_csv_path_.c_str(), ex.what());
        }
    }

    RCLCPP_INFO(this->get_logger(), "Fairino Controller Node");
    RCLCPP_INFO(this->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Dummy Mode: %s", dummy_mode_ ? "TRUE (Simulation)" : "FALSE (Hardware)");
    const char* stream_mode_label = "servo_j";
    if (stream_command_mode_ == StreamCommandMode::kServoCart) {
        stream_mode_label = "servo_cart";
    } else if (stream_command_mode_ == StreamCommandMode::kMoveJ) {
        stream_mode_label = "move_j";
    }
    RCLCPP_INFO(this->get_logger(), "  Stream command mode: %s", stream_mode_label);
    RCLCPP_INFO(this->get_logger(), "  Global speed percent: %d", global_speed_percent_);
    RCLCPP_INFO(this->get_logger(), "  Servo stream cmd period: %.4f s", servo_cmd_period_sec_);
    RCLCPP_INFO(this->get_logger(), "  Pose target nominal period: %.4f s",
                pose_target_nominal_period_sec_);
    RCLCPP_INFO(this->get_logger(), "  ServoCart mode: %s (sdk mode %d)",
                teleop_slave::ServoCartModeToString(servocart_mode_).c_str(),
                teleop_slave::ServoCartModeToSdkMode(servocart_mode_));
    RCLCPP_INFO(this->get_logger(), "  ServoCart filter_t: %.4f s", servocart_filter_t_);
    RCLCPP_INFO(this->get_logger(), "  ServoCart gain: %.2f", servocart_gain_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ max step: %.2f deg", servoj_max_step_deg_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ filter_t: %.4f s", servoj_filter_t_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ gain: %.2f", servoj_gain_);
    RCLCPP_INFO(this->get_logger(),
                "  MoveJ cmd period: %.4f s, vel %.1f%%, ovl %.1f%%, blend %.1f ms",
                movej_cmd_period_sec_, movej_vel_percent_, movej_ovl_percent_, movej_blend_t_ms_);
    RCLCPP_INFO(this->get_logger(),
                "  MoveJ command deadband deg: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                movej_command_deadband_deg_[0], movej_command_deadband_deg_[1],
                movej_command_deadband_deg_[2], movej_command_deadband_deg_[3],
                movej_command_deadband_deg_[4], movej_command_deadband_deg_[5]);
    RCLCPP_INFO(this->get_logger(),
                "  ServoJ output deadband deg: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                servoj_output_deadband_deg_[0], servoj_output_deadband_deg_[1],
                servoj_output_deadband_deg_[2], servoj_output_deadband_deg_[3],
                servoj_output_deadband_deg_[4], servoj_output_deadband_deg_[5]);
    RCLCPP_INFO(this->get_logger(),
                "  ServoJ tracking guard: %s [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                enable_servoj_tracking_guard_ ? "enabled" : "disabled",
                servoj_tracking_error_hold_deg_[0], servoj_tracking_error_hold_deg_[1],
                servoj_tracking_error_hold_deg_[2], servoj_tracking_error_hold_deg_[3],
                servoj_tracking_error_hold_deg_[4], servoj_tracking_error_hold_deg_[5]);
    RCLCPP_INFO(this->get_logger(), "  Prefer stream target for IK seed: %s",
                prefer_stream_target_for_ik_seed_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Joint debug CSV: %s",
                enable_joint_debug_csv_ ? joint_debug_csv_path_.c_str() : "disabled");
    RCLCPP_INFO(this->get_logger(),
                "  Joint continuity guard: %s (pose step %.1f mm, %.1f deg)",
                enable_joint_continuity_guard_ ? "enabled" : "disabled",
                joint_continuity_guard_pose_step_mm_,
                joint_continuity_guard_orientation_step_deg_);
    RCLCPP_INFO(this->get_logger(), "  Joint reversal guard: %s (hold cycles=%d)",
                enable_joint_reversal_guard_ ? "enabled" : "disabled",
                joint_reversal_hold_cycles_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ failure limit: %d", servoj_failure_limit_);
    RCLCPP_INFO(this->get_logger(),
                "  Joint deadband deg: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                joint_deadband_deg_[0], joint_deadband_deg_[1], joint_deadband_deg_[2],
                joint_deadband_deg_[3], joint_deadband_deg_[4], joint_deadband_deg_[5]);
    RCLCPP_INFO(this->get_logger(),
                "  Joint velocity limit deg/s: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                joint_velocity_limit_deg_s_[0], joint_velocity_limit_deg_s_[1],
                joint_velocity_limit_deg_s_[2], joint_velocity_limit_deg_s_[3],
                joint_velocity_limit_deg_s_[4], joint_velocity_limit_deg_s_[5]);
    RCLCPP_INFO(this->get_logger(),
                "  Joint acceleration limit deg/s^2: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                joint_acceleration_limit_deg_s2_[0], joint_acceleration_limit_deg_s2_[1],
                joint_acceleration_limit_deg_s2_[2], joint_acceleration_limit_deg_s2_[3],
                joint_acceleration_limit_deg_s2_[4], joint_acceleration_limit_deg_s2_[5]);
    RCLCPP_INFO(this->get_logger(),
                "  Joint continuity guard jump deg: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                joint_continuity_guard_jump_deg_[0], joint_continuity_guard_jump_deg_[1],
                joint_continuity_guard_jump_deg_[2], joint_continuity_guard_jump_deg_[3],
                joint_continuity_guard_jump_deg_[4], joint_continuity_guard_jump_deg_[5]);
    RCLCPP_INFO(this->get_logger(),
                "  Joint reversal guard step deg: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                joint_reversal_guard_step_deg_[0], joint_reversal_guard_step_deg_[1],
                joint_reversal_guard_step_deg_[2], joint_reversal_guard_step_deg_[3],
                joint_reversal_guard_step_deg_[4], joint_reversal_guard_step_deg_[5]);

    traj_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/trajectory_points", 10,
        std::bind(&FairinoControllerNode::trajectoryCallback, this, std::placeholders::_1));

    pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/fr5/pose_target", 10,
        std::bind(&FairinoControllerNode::poseTargetCallback, this, std::placeholders::_1));

    // legacy_pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //     "/curobo/pose_target", 10,
    //     std::bind(&FairinoControllerNode::poseTargetCallback, this, std::placeholders::_1));

    const std::string joint_topic = dummy_mode_ ? "/joint_states" : "/robot_joint_states";
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_topic, 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

    execute_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/execute_trajectory",
        std::bind(&FairinoControllerNode::executeTrajectoryService, this,
                  std::placeholders::_1, std::placeholders::_2));

    stream_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo_target", 10,
        std::bind(&FairinoControllerNode::streamCallback, this, std::placeholders::_1));

    stream_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/enable_streaming",
        std::bind(&FairinoControllerNode::streamService, this,
                  std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = this->create_wall_timer(
        20ms, std::bind(&FairinoControllerNode::publishJointStates, this));

    dummy_robot_pose_.header.frame_id = "base_link";
    dummy_robot_pose_.pose.orientation.w = 1.0;
    latest_actual_pose_.orientation.w = 1.0;
    last_successful_ik_time_ = this->now();
}

FairinoControllerNode::~FairinoControllerNode() {
    shutdown();
}

bool FairinoControllerNode::initialize() {
    if (global_speed_percent_ < 0 || global_speed_percent_ > 100) {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid global_speed_percent=%d. Expected an integer in [0, 100].",
                     global_speed_percent_);
        return false;
    }
    if (pose_target_nominal_period_sec_ < 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid pose_target_nominal_period_sec=%.4f. Expected >= 0.",
                     pose_target_nominal_period_sec_);
        return false;
    }
    return connectRobot();
}

void FairinoControllerNode::shutdown() {
    if (shutdown_done_) {
        return;
    }
    shutdown_done_ = true;

    RCLCPP_INFO(this->get_logger(), "Starting safe shutdown...");

    executing_ = false;
    stream_mode_ = false;
    resetPoseTargetSolverState();
    std::this_thread::sleep_for(100ms);

    disconnectRobot();

    RCLCPP_INFO(this->get_logger(), "Safe shutdown complete");
}

bool FairinoControllerNode::connectRobot() {
    if (dummy_mode_) {
        dummy_joint_positions_.resize(NUM_JOINTS, 0.0);
        connected_ = true;
        RCLCPP_INFO(this->get_logger(), "Dummy mode enabled. Hardware connection skipped.");
        return true;
    }

    robot_ = CreateFairinoRobotSdkAdapter();
    RCLCPP_INFO(this->get_logger(), "Connecting to Fairino controller at %s", robot_ip_.c_str());

    const errno_t ret = robot_->RPC(robot_ip_.c_str());
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot controller (ret=%d)", ret);
        robot_.reset();
        return false;
    }

    robot_->SetReConnectParam(true, 30000, 2000);
    robot_->ResetAllError();

    const errno_t mode_ret = robot_->Mode(0);
    if (mode_ret != 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to switch controller to automatic mode while connecting (%s)",
                     BuildFairinoErrorDetail(robot_.get(), mode_ret).c_str());
        disconnectRobot();
        return false;
    }

    const errno_t enable_ret = robot_->RobotEnable(1);
    if (enable_ret != 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to enable robot while connecting (%s)",
                     BuildFairinoErrorDetail(robot_.get(), enable_ret).c_str());
        disconnectRobot();
        return false;
    }

    const errno_t speed_ret = robot_->SetSpeed(global_speed_percent_);
    if (speed_ret != 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set global_speed_percent=%d on controller (ret=%d)",
                     global_speed_percent_, speed_ret);
        disconnectRobot();
        return false;
    }

    connected_ = validateControllerState();
    if (!connected_) {
        disconnectRobot();
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Connected to %s. Verify alarms, payload, TCP, and workobject in the web UI at http://192.168.58.2",
                robot_ip_.c_str());
    return true;
}

bool FairinoControllerNode::validateControllerState() {
    teleop_slave::ControllerValidationResult validation = teleop_slave::ValidateControllerState(
        *robot_,
        this->get_parameter("expected_tcp_id").as_int(),
        this->get_parameter("expected_wobj_id").as_int());

    int maincode = 0;
    int subcode = 0;
    robot_->GetRobotErrorCode(&maincode, &subcode);

    if (!validation.ok) {
        RCLCPP_ERROR(this->get_logger(), "Controller validation failed: %s",
                     validation.message.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Controller state validated: %s | error(main=%d, sub=%d) | mode set to automatic in node",
                validation.message.c_str(), maincode, subcode);
    return true;
}

bool FairinoControllerNode::startServoMotionSession(const char* context,
                                                    std::string* failure_message) {
    if (dummy_mode_) {
        return true;
    }

    std::lock_guard<std::mutex> lock(servo_motion_session_mutex_);
    if (!robot_) {
        const std::string message = "Robot interface is unavailable before servo session start.";
        if (failure_message != nullptr) {
            *failure_message = message;
        }
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }

    if (servo_motion_session_active_) {
        std::ostringstream stream;
        stream << "Servo motion session is already active while starting " << context << ".";
        if (failure_message != nullptr) {
            *failure_message = stream.str();
        }
        RCLCPP_ERROR(this->get_logger(), "%s", stream.str().c_str());
        return false;
    }

    const errno_t reset_ret = robot_->ResetAllError();
    if (reset_ret != 0) {
        RCLCPP_WARN(this->get_logger(),
                    "ResetAllError before %s returned non-zero (%s)",
                    context, BuildFairinoErrorDetail(robot_.get(), reset_ret).c_str());
    }
    std::this_thread::sleep_for(20ms);

    const errno_t mode_ret = robot_->Mode(0);
    if (mode_ret != 0) {
        std::ostringstream stream;
        stream << "Controller-side prerequisite failure while starting " << context
               << ": Mode(0) failed (" << BuildFairinoErrorDetail(robot_.get(), mode_ret)
               << ").";
        if (failure_message != nullptr) {
            *failure_message = stream.str();
        }
        RCLCPP_ERROR(this->get_logger(), "%s", stream.str().c_str());
        return false;
    }
    std::this_thread::sleep_for(20ms);

    const errno_t enable_ret = robot_->RobotEnable(1);
    if (enable_ret != 0) {
        std::ostringstream stream;
        stream << "Controller-side prerequisite failure while starting " << context
               << ": RobotEnable(1) failed ("
               << BuildFairinoErrorDetail(robot_.get(), enable_ret) << ").";
        if (failure_message != nullptr) {
            *failure_message = stream.str();
        }
        RCLCPP_ERROR(this->get_logger(), "%s", stream.str().c_str());
        return false;
    }
    std::this_thread::sleep_for(20ms);

    const errno_t start_ret = robot_->ServoMoveStart();
    if (start_ret != 0) {
        std::ostringstream stream;
        stream << "Servo motion session start failed for " << context << " ("
               << BuildFairinoErrorDetail(robot_.get(), start_ret)
               << "). This usually means the controller rejected servo-mode entry.";
        if (failure_message != nullptr) {
            *failure_message = stream.str();
        }
        RCLCPP_ERROR(this->get_logger(), "%s", stream.str().c_str());
        return false;
    }
    std::this_thread::sleep_for(50ms);

    servo_motion_session_active_ = true;
    return true;
}

void FairinoControllerNode::stopServoMotionSession(const char* context) {
    std::lock_guard<std::mutex> lock(servo_motion_session_mutex_);
    if (dummy_mode_) {
        return;
    }

    servo_cart_first_command_pending_.store(false);
    has_prev_servoj_cmd_ = false;
    has_prev_movej_cmd_ = false;
    if (!servo_motion_session_active_) {
        return;
    }

    if (!robot_) {
        servo_motion_session_active_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Servo motion session marked inactive during %s because robot interface is unavailable.",
                    context);
        return;
    }

    const errno_t stop_ret = robot_->ServoMoveEnd();
    if (stop_ret != 0) {
        RCLCPP_WARN(this->get_logger(),
                    "ServoMoveEnd during %s returned non-zero (%s)",
                    context, BuildFairinoErrorDetail(robot_.get(), stop_ret).c_str());
    }

    servo_motion_session_active_ = false;
}

void FairinoControllerNode::disconnectRobot() {
    if (!connected_ && !robot_) {
        return;
    }

    if (dummy_mode_) {
        connected_ = false;
        RCLCPP_INFO(this->get_logger(), "Dummy mode disconnected");
        return;
    }

    if (!robot_) {
        return;
    }

    stopServoMotionSession("disconnect");
    robot_->RobotEnable(0);
    robot_->CloseRPC();

    connected_ = false;
    robot_.reset();
    RCLCPP_INFO(this->get_logger(), "Disconnected from Fairino controller");
}

void FairinoControllerNode::trajectoryCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (executing_ || stream_mode_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring trajectory input while executing or streaming.");
        return;
    }

    std::lock_guard<std::mutex> lock(traj_mutex_);
    trajectory_queue_.clear();

    const size_t n_points = msg->data.size() / NUM_JOINTS;
    if (msg->data.size() % NUM_JOINTS != 0 || n_points == 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid trajectory payload size: %zu", msg->data.size());
        return;
    }

    for (size_t i = 0; i < n_points; ++i) {
        std::vector<double> point(NUM_JOINTS);
        for (int j = 0; j < NUM_JOINTS; ++j) {
            point[j] = msg->data[i * NUM_JOINTS + j];
        }
        trajectory_queue_.push_back(point);
    }

    trajectory_loaded_ = true;
    current_traj_idx_ = 0;

    RCLCPP_INFO(this->get_logger(), "Loaded trajectory with %zu points", n_points);
}

void FairinoControllerNode::poseTargetCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(pose_target_mutex_);
        latest_pose_target_ = *msg;
        latest_pose_target_arrival_ = this->now();
        pose_target_received_ = true;
        pose_target_stream_active_ = true;
    }

    if (dummy_mode_ && !stream_mode_) {
        dummy_robot_pose_ = *msg;
        dummy_robot_pose_.header.stamp = this->now();
    }

    if (!stream_mode_ || !connected_) {
        return;
    }

    if (stream_command_mode_ == StreamCommandMode::kServoCart) {
        const geometry_msgs::msg::Pose normalized_target =
            teleop_slave::NormalizePoseQuaternion(msg->pose);
        bool log_startup_jump = false;
        double startup_translation_mm = 0.0;
        double startup_orientation_deg = 0.0;
        {
            std::lock_guard<std::mutex> lock(stream_mutex_);
            const auto now_tp = std::chrono::steady_clock::now();
            const geometry_msgs::msg::Pose current_stream_pose =
                cartesian_stream_interpolator_.sample(now_tp);

            if (!cartesian_stream_has_tracker_target_) {
                const tf2::Vector3 delta(
                    normalized_target.position.x - current_stream_pose.position.x,
                    normalized_target.position.y - current_stream_pose.position.y,
                    normalized_target.position.z - current_stream_pose.position.z);
                startup_translation_mm = delta.length() * 1000.0;
                tf2::Quaternion current_q;
                tf2::Quaternion target_q;
                tf2::fromMsg(current_stream_pose.orientation, current_q);
                tf2::fromMsg(normalized_target.orientation, target_q);
                startup_orientation_deg = teleop_slave::QuaternionAngularDistanceDegrees(
                    current_q, target_q);
                log_startup_jump =
                    startup_translation_mm > kStartupTranslationWarnThresholdMm ||
                    startup_orientation_deg > kStartupOrientationWarnThresholdDeg;
            }

            cartesian_stream_interpolator_.setTarget(
                normalized_target,
                now_tp,
                pose_target_nominal_period_sec_,
                this->get_parameter("interpolate_stream").as_bool());
            cartesian_stream_has_tracker_target_ = true;
        }

        if (log_startup_jump) {
            RCLCPP_WARN(this->get_logger(),
                        "First ServoCart target after clutch is large: %.1f mm translation, %.1f deg orientation. Blending over %.3f s.",
                        startup_translation_mm, startup_orientation_deg,
                        pose_target_nominal_period_sec_);
        }
        return;
    }

    auto should_stop_for_ik_failures = [this]() {
        const double ik_failure_timeout_sec =
            this->get_parameter("ik_failure_timeout_sec").as_double();
        const bool exceeded_count =
            consecutive_ik_failures_ >= this->get_parameter("ik_failure_limit").as_int();
        const bool exceeded_time =
            (this->now() - last_successful_ik_time_).seconds() >= ik_failure_timeout_sec;
        return exceeded_count && exceeded_time;
    };

    if (dummy_mode_) {
        return;
    }

    teleop_slave::PoseIkOptions options;
    options.expected_tcp_id = this->get_parameter("expected_tcp_id").as_int();
    options.expected_wobj_id = this->get_parameter("expected_wobj_id").as_int();
    options.allow_orientation_fallback =
        this->get_parameter("allow_orientation_fallback").as_bool();
    options.position_backoff_steps =
        this->get_parameter("ik_position_backoff_steps").as_int();
    if (prefer_stream_target_for_ik_seed_) {
        std::lock_guard<std::mutex> lock(stream_mutex_);
        if (stream_target_deg_.size() == NUM_JOINTS) {
            options.has_reference_joints = true;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                options.reference_joint_deg[i] = stream_target_deg_[i];
            }
        }
    }

    const auto result = teleop_slave::SolvePoseTargetIK(*robot_, *msg, options);
    if (!result.success) {
        consecutive_ik_failures_++;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Pose target IK rejected: %s", result.message.c_str());
        if (should_stop_for_ik_failures()) {
            RCLCPP_ERROR(this->get_logger(),
                         "IK failure limit reached for %.2f seconds. Holding last valid target stopped the arm as a safety fallback.",
                         (this->now() - last_successful_ik_time_).seconds());
            stream_mode_ = false;
        }
        return;
    }

    consecutive_ik_failures_ = 0;
    last_successful_ik_time_ = this->now();
    JointArray raw_target_deg{};
    std::copy(result.joint_target_deg.begin(), result.joint_target_deg.end(), raw_target_deg.begin());
    const geometry_msgs::msg::Pose normalized_pose_target =
        teleop_slave::NormalizePoseQuaternion(msg->pose);

    JointArray filtered_target_deg{};
    JointArray guarded_target_deg = raw_target_deg;
    std::array<bool, 3> continuity_guarded_j123{false, false, false};
    std::array<bool, 3> guarded_j123{false, false, false};
    bool j123_direction_reversal_detected = false;
    double continuity_guard_pose_step_mm = 0.0;
    double continuity_guard_pose_step_deg = 0.0;
    double continuity_guard_max_jump_deg = 0.0;
    {
        std::lock_guard<std::mutex> lock(stream_mutex_);
        const auto now_tp = std::chrono::steady_clock::now();
        const double dt_sec = joint_filter_time_initialized_
            ? std::chrono::duration<double>(now_tp - last_joint_filter_update_time_).count()
            : 0.0;
        const auto& current_filtered_deg = joint_target_smoother_.filtered_target_deg();

        if (previous_pose_target_pose_valid_) {
            continuity_guard_pose_step_mm = teleop_slave::PoseTranslationDistanceMillimeters(
                previous_pose_target_pose_, normalized_pose_target);
            tf2::Quaternion previous_pose_q;
            tf2::Quaternion current_pose_q;
            tf2::fromMsg(previous_pose_target_pose_.orientation, previous_pose_q);
            tf2::fromMsg(normalized_pose_target.orientation, current_pose_q);
            continuity_guard_pose_step_deg = teleop_slave::QuaternionAngularDistanceDegrees(
                previous_pose_q, current_pose_q);

            if (enable_joint_continuity_guard_ &&
                teleop_slave::IsLikelyJointBranchJump(
                    previous_pose_target_pose_,
                    normalized_pose_target,
                    current_filtered_deg,
                    raw_target_deg,
                    joint_continuity_guard_pose_step_mm_,
                    joint_continuity_guard_orientation_step_deg_,
                    joint_continuity_guard_jump_deg_)) {
                for (int i = 0; i < 3; ++i) {
                    if (joint_continuity_guard_jump_deg_[i] <= 0.0) {
                        continue;
                    }
                    continuity_guard_max_jump_deg = std::max(
                        continuity_guard_max_jump_deg,
                        std::abs(raw_target_deg[i] - current_filtered_deg[i]));
                    guarded_target_deg[i] = current_filtered_deg[i];
                    continuity_guarded_j123[i] = true;
                }
            }
        }
        previous_pose_target_pose_ = normalized_pose_target;
        previous_pose_target_pose_valid_ = true;

        if (raw_has_prev_ik_target_) {
            JointArray current_raw_step_deg{};
            for (int i = 0; i < NUM_JOINTS; ++i) {
                current_raw_step_deg[i] = raw_target_deg[i] - raw_prev_ik_target_deg_[i];
            }
            if (raw_has_prev_ik_step_) {
                for (int i = 0; i < 3; ++i) {
                    if (std::abs(raw_prev_ik_step_deg_[i]) >= joint_deadband_deg_[i] &&
                        std::abs(current_raw_step_deg[i]) >= joint_deadband_deg_[i] &&
                        raw_prev_ik_step_deg_[i] * current_raw_step_deg[i] < 0.0) {
                        j123_direction_reversal_detected = true;
                        if (enable_joint_reversal_guard_ &&
                            joint_reversal_guard_step_deg_[i] > 0.0 &&
                            std::abs(current_raw_step_deg[i]) <= joint_reversal_guard_step_deg_[i] &&
                            std::abs(raw_target_deg[i] - current_filtered_deg[i]) <=
                                joint_reversal_guard_step_deg_[i]) {
                            joint_reversal_hold_remaining_[i] =
                                std::max(joint_reversal_hold_remaining_[i], joint_reversal_hold_cycles_);
                        }
                    }
                }
            }
            raw_prev_ik_step_deg_ = current_raw_step_deg;
            raw_has_prev_ik_step_ = true;
        }
        raw_prev_ik_target_deg_ = raw_target_deg;
        raw_has_prev_ik_target_ = true;

        if (enable_joint_reversal_guard_) {
            for (int i = 0; i < 3; ++i) {
                if (joint_reversal_hold_remaining_[i] <= 0) {
                    continue;
                }
                if (std::abs(raw_target_deg[i] - current_filtered_deg[i]) <=
                    joint_reversal_guard_step_deg_[i]) {
                    guarded_target_deg[i] = current_filtered_deg[i];
                    guarded_j123[i] = true;
                    --joint_reversal_hold_remaining_[i];
                } else {
                    joint_reversal_hold_remaining_[i] = 0;
                }
            }
        }

        const auto smooth_result = joint_target_smoother_.update(guarded_target_deg, dt_sec);
        filtered_target_deg = smooth_result.filtered_target_deg;
        last_joint_filter_update_time_ = now_tp;
        joint_filter_time_initialized_ = true;

        if (!stream_target_deg_.empty()) {
            if (stream_has_prev_) {
                stream_prev_prev_deg_ = stream_prev_deg_;
                stream_prev_prev_time_ = stream_prev_time_;
                stream_has_prev_prev_ = true;
            }
            stream_prev_deg_ = stream_target_deg_;
            stream_prev_time_ = stream_target_time_;
            stream_has_prev_ = true;
        }
        stream_target_deg_ = JointArrayToVector(filtered_target_deg);
        stream_target_time_ = now_tp;
    }

    if (j123_direction_reversal_detected) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Detected rapid J1-J3 raw IK direction reversal. This is a likely chatter signature from tracker translation jitter.");
    }
    if (continuity_guarded_j123[0] || continuity_guarded_j123[1] || continuity_guarded_j123[2]) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Joint continuity guard held J1-J3 = [%s, %s, %s]; pose step %.2f mm / %.2f deg, raw J1-J3 jump up to %.3f deg.",
            continuity_guarded_j123[0] ? "Y" : "N",
            continuity_guarded_j123[1] ? "Y" : "N",
            continuity_guarded_j123[2] ? "Y" : "N",
            continuity_guard_pose_step_mm,
            continuity_guard_pose_step_deg,
            continuity_guard_max_jump_deg);
    }
    if (guarded_j123[0] || guarded_j123[1] || guarded_j123[2]) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Joint reversal guard holding J1-J3 = [%s, %s, %s] to suppress IK chatter.",
            guarded_j123[0] ? "Y" : "N",
            guarded_j123[1] ? "Y" : "N",
            guarded_j123[2] ? "Y" : "N");
    }
    logJointSmoothingDiagnostics(raw_target_deg, filtered_target_deg);
    maybeWriteJointDebugCsv(normalized_pose_target,
                            raw_target_deg,
                            guarded_target_deg,
                            filtered_target_deg,
                            continuity_guarded_j123,
                            guarded_j123,
                            j123_direction_reversal_detected,
                            continuity_guard_pose_step_mm,
                            continuity_guard_pose_step_deg,
                            continuity_guard_max_jump_deg);
}

void FairinoControllerNode::executeTrajectoryService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!connected_) {
        response->success = false;
        response->message = "Robot is not connected.";
        return;
    }

    if (!trajectory_loaded_ || trajectory_queue_.empty()) {
        response->success = false;
        response->message = "No trajectory loaded.";
        return;
    }

    if (executing_ || stream_mode_) {
        response->success = false;
        response->message = "Trajectory execution or streaming is already active.";
        return;
    }

    executing_ = true;
    current_traj_idx_ = 0;
    servo_error_count_.store(0);
    consecutive_servoj_failures_ = 0;
    servo_cart_first_command_pending_.store(false);

    if (!dummy_mode_) {
        std::string failure_message;
        if (!startServoMotionSession("trajectory execution", &failure_message)) {
            executing_ = false;
            response->success = false;
            response->message = failure_message;
            return;
        }
    }

    std::thread([this]() {
        controlLoop();
        stopServoMotionSession("trajectory execution completion");
        executing_ = false;
        RCLCPP_INFO(this->get_logger(), "Trajectory execution complete");
    }).detach();

    response->success = true;
    response->message = "Trajectory execution started";
}

void FairinoControllerNode::controlLoop() {
    const auto period = std::chrono::duration<double>(servo_cmd_period_sec_);

    while (executing_ && current_traj_idx_ < trajectory_queue_.size()) {
        const auto start_time = std::chrono::steady_clock::now();

        std::vector<double> point;
        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            point = trajectory_queue_[current_traj_idx_];
        }

        std::vector<double> joints_deg(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joints_deg[i] = point[i] * RAD_TO_DEG;
        }

        executeServoJ(joints_deg);
        current_traj_idx_++;

        const auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

bool FairinoControllerNode::executeServoJ(const std::vector<double>& target_deg) {
    if (target_deg.size() != NUM_JOINTS) {
        return false;
    }

    if (dummy_mode_) {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            dummy_joint_positions_[i] = target_deg[i];
        }
        return true;
    }

    if (!robot_) {
        return false;
    }

    if (!has_prev_servoj_cmd_) {
        robot_->GetActualJointPosDegree(0, &prev_servoj_cmd_);
        has_prev_servoj_cmd_ = true;
    }

    JointArray actual_joint_deg{};
    bool has_actual_joint = false;
    {
        std::lock_guard<std::mutex> lock(actual_joint_mutex_);
        if (actual_joint_received_) {
            actual_joint_deg = latest_actual_joint_deg_;
            has_actual_joint = true;
        }
    }

    JointPos cmd;
    std::array<bool, NUM_JOINTS> output_deadband_hold{};
    std::array<bool, NUM_JOINTS> tracking_guard_hold{};
    double max_tracking_error_deg = 0.0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        const double step = target_deg[i] - prev_servoj_cmd_.jPos[i];
        const double clamped = std::clamp(step, -servoj_max_step_deg_, servoj_max_step_deg_);
        double next_cmd = prev_servoj_cmd_.jPos[i] + clamped;

        if (servoj_output_deadband_deg_[i] > 0.0 &&
            std::abs(next_cmd - prev_servoj_cmd_.jPos[i]) < servoj_output_deadband_deg_[i]) {
            next_cmd = prev_servoj_cmd_.jPos[i];
            output_deadband_hold[i] = true;
        }

        if (enable_servoj_tracking_guard_ && has_actual_joint &&
            servoj_tracking_error_hold_deg_[i] > 0.0) {
            const double tracking_error_deg = prev_servoj_cmd_.jPos[i] - actual_joint_deg[i];
            max_tracking_error_deg =
                std::max(max_tracking_error_deg, std::abs(tracking_error_deg));
            const bool pushing_further_away =
                std::abs(tracking_error_deg) >= servoj_tracking_error_hold_deg_[i] &&
                (next_cmd - prev_servoj_cmd_.jPos[i]) * tracking_error_deg > 0.0;
            if (pushing_further_away) {
                next_cmd = prev_servoj_cmd_.jPos[i];
                tracking_guard_hold[i] = true;
            }
        }

        cmd.jPos[i] = next_cmd;
    }

    if (tracking_guard_hold[0] || tracking_guard_hold[1] || tracking_guard_hold[2]) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "ServoJ tracking guard holding J1-J3 = [%s, %s, %s]; last-command tracking error up to %.3f deg.",
            tracking_guard_hold[0] ? "Y" : "N",
            tracking_guard_hold[1] ? "Y" : "N",
            tracking_guard_hold[2] ? "Y" : "N",
            max_tracking_error_deg);
    }
    if (output_deadband_hold[0] || output_deadband_hold[1] || output_deadband_hold[2]) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "ServoJ output deadband holding J1-J3 = [%s, %s, %s].",
            output_deadband_hold[0] ? "Y" : "N",
            output_deadband_hold[1] ? "Y" : "N",
            output_deadband_hold[2] ? "Y" : "N");
    }

    ExaxisPos epos(0, 0, 0, 0);
    const float t = static_cast<float>(servo_cmd_period_sec_);

    const float ft = static_cast<float>(servoj_filter_t_);
    const float g = static_cast<float>(servoj_gain_);
    const errno_t ret = robot_->ServoJ(&cmd, &epos, 0.0f, 0.0f, t, ft, g, 0);
    if (ret != 0) {
        const int err_cnt = servo_error_count_.fetch_add(1) + 1;
        if (err_cnt <= 3 || err_cnt % 100 == 0) {
            int maincode = 0;
            int subcode = 0;
            robot_->GetRobotErrorCode(&maincode, &subcode);
            RCLCPP_WARN(this->get_logger(),
                        "ServoJ failed (count=%d, ret=%d, main=%d, sub=%d)",
                        err_cnt, ret, maincode, subcode);
            if (ret == 14) {
                RCLCPP_WARN(this->get_logger(),
                            "ret=14 is a Fairino interface execution failure. Common causes are controller mode/enable mismatch, active alarms, or an unsupported ServoJ cycle time.");
            }
        }
        robot_->ResetAllError();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return false;
    }

    prev_servoj_cmd_ = cmd;
    return true;
}

bool FairinoControllerNode::executeMoveJ(const std::vector<double>& target_deg) {
    if (target_deg.size() != NUM_JOINTS) {
        return false;
    }

    if (dummy_mode_) {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            dummy_joint_positions_[i] = target_deg[i];
        }
        return true;
    }

    if (!robot_) {
        return false;
    }

    if (!has_prev_movej_cmd_) {
        if (robot_->GetActualJointPosDegree(0, &prev_movej_cmd_) != 0) {
            return false;
        }
        has_prev_movej_cmd_ = true;
    }

    bool should_send = false;
    JointPos cmd = prev_movej_cmd_;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        cmd.jPos[i] = target_deg[i];
        if (std::abs(cmd.jPos[i] - prev_movej_cmd_.jPos[i]) >= movej_command_deadband_deg_[i]) {
            should_send = true;
        }
    }

    if (!should_send) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "MoveJ command deadband holding J1-J3 = [%s, %s, %s].",
            std::abs(cmd.jPos[0] - prev_movej_cmd_.jPos[0]) < movej_command_deadband_deg_[0] ? "Y" : "N",
            std::abs(cmd.jPos[1] - prev_movej_cmd_.jPos[1]) < movej_command_deadband_deg_[1] ? "Y" : "N",
            std::abs(cmd.jPos[2] - prev_movej_cmd_.jPos[2]) < movej_command_deadband_deg_[2] ? "Y" : "N");
        return true;
    }

    const int tool = this->get_parameter("expected_tcp_id").as_int() >= 0
        ? this->get_parameter("expected_tcp_id").as_int()
        : 0;
    const int user = this->get_parameter("expected_wobj_id").as_int() >= 0
        ? this->get_parameter("expected_wobj_id").as_int()
        : 0;
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos{};
    const errno_t ret = robot_->MoveJ(
        &cmd,
        tool,
        user,
        static_cast<float>(movej_vel_percent_),
        static_cast<float>(movej_acc_percent_),
        static_cast<float>(movej_ovl_percent_),
        &epos,
        static_cast<float>(movej_blend_t_ms_),
        0,
        &offset_pos);
    if (ret != 0) {
        const int err_cnt = servo_error_count_.fetch_add(1) + 1;
        if (err_cnt <= 3 || err_cnt % 100 == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "MoveJ failed (count=%d, %s)",
                        err_cnt, BuildFairinoErrorDetail(robot_.get(), ret).c_str());
        }
        robot_->ResetAllError();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return false;
    }

    prev_movej_cmd_ = cmd;
    return true;
}

bool FairinoControllerNode::executeServoCart(const geometry_msgs::msg::Pose& target_pose) {
    if (dummy_mode_) {
        const geometry_msgs::msg::Pose command_pose =
            teleop_slave::NormalizePoseQuaternion(target_pose);
        if (servocart_mode_ == teleop_slave::ServoCartMode::kAbsoluteBase) {
            dummy_robot_pose_.pose = command_pose;
        } else {
            dummy_robot_pose_.pose.position.x += command_pose.position.x;
            dummy_robot_pose_.pose.position.y += command_pose.position.y;
            dummy_robot_pose_.pose.position.z += command_pose.position.z;

            tf2::Quaternion current_q;
            tf2::Quaternion delta_q;
            tf2::fromMsg(dummy_robot_pose_.pose.orientation, current_q);
            tf2::fromMsg(command_pose.orientation, delta_q);
            dummy_robot_pose_.pose.orientation =
                tf2::toMsg(teleop_slave::NormalizeStreamQuaternion(delta_q * current_q));
        }
        dummy_robot_pose_.header.stamp = this->now();
        dummy_robot_pose_.header.frame_id = "base_link";
        return true;
    }

    if (!robot_) {
        return false;
    }

    DescPose desc_pose = teleop_slave::PoseToDescPose(target_pose);
    ExaxisPos exaxis(0, 0, 0, 0);
    float pos_gain[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    const float t = static_cast<float>(servo_cmd_period_sec_);
    const float ft = static_cast<float>(servocart_filter_t_);
    const float g = static_cast<float>(servocart_gain_);
    const int sdk_mode = teleop_slave::ServoCartModeToSdkMode(servocart_mode_);
    const bool first_command_after_session_start =
        servo_cart_first_command_pending_.exchange(false);
    DescPose actual_tcp_pose{};
    bool actual_tcp_pose_valid = false;
    if (first_command_after_session_start) {
        actual_tcp_pose_valid = robot_->GetActualTCPPose(0, &actual_tcp_pose) == 0;
        RCLCPP_INFO(
            this->get_logger(),
            "First ServoCart command after session start: mode=%s (sdk mode %d), command Pose(mm,deg)=%s, actual TCP Pose(mm,deg)=%s, cmdT=%.4f s, filterT=%.4f s, gain=%.2f",
            teleop_slave::ServoCartModeToString(servocart_mode_).c_str(),
            sdk_mode,
            DescribeDescPose(desc_pose).c_str(),
            DescribeMaybeDescPose(actual_tcp_pose, actual_tcp_pose_valid).c_str(),
            t,
            ft,
            g);
    }
    const errno_t ret =
        robot_->ServoCart(sdk_mode, &desc_pose, exaxis, pos_gain, 0.0f, 0.0f, t, ft, g);
    if (ret != 0) {
        const int err_cnt = servo_error_count_.fetch_add(1) + 1;
        if (err_cnt <= 3 || err_cnt % 100 == 0) {
            int maincode = 0;
            int subcode = 0;
            robot_->GetRobotErrorCode(&maincode, &subcode);
            RCLCPP_WARN(this->get_logger(),
                        "ServoCart failed (count=%d, %s)",
                        err_cnt, BuildFairinoErrorDetail(robot_.get(), ret).c_str());
            if (first_command_after_session_start) {
                RCLCPP_ERROR(this->get_logger(),
                             "First ServoCart command after servo session start failed (%s). ServoMoveStart succeeded, so the controller is rejecting the Cartesian stream itself or a runtime prerequisite is still unmet.",
                             BuildFairinoErrorDetail(robot_.get(), ret).c_str());
                RCLCPP_ERROR(
                    this->get_logger(),
                    "First ServoCart failure diagnostics: mode=%s (sdk mode %d), command Pose(mm,deg)=%s, actual TCP Pose(mm,deg)=%s, cmdT=%.4f s, filterT=%.4f s, gain=%.2f, controller error(main=%d, sub=%d)",
                    teleop_slave::ServoCartModeToString(servocart_mode_).c_str(),
                    sdk_mode,
                    DescribeDescPose(desc_pose).c_str(),
                    DescribeMaybeDescPose(actual_tcp_pose, actual_tcp_pose_valid).c_str(),
                    t,
                    ft,
                    g,
                    maincode,
                    subcode);
            }
            if (ret == -4) {
                RCLCPP_WARN(this->get_logger(),
                            "ret=-4 is a Fairino XML-RPC execution failure. This node is using ServoCart mode=%s (sdk mode %d), pos_gain=1, and cmdT=%.3f s.",
                            teleop_slave::ServoCartModeToString(servocart_mode_).c_str(),
                            sdk_mode,
                            servo_cmd_period_sec_);
            }
        }
        robot_->ResetAllError();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return false;
    }

    return true;
}

void FairinoControllerNode::resetPoseTargetSolverState() {
    raw_has_prev_ik_target_ = false;
    raw_has_prev_ik_step_ = false;
    joint_reversal_hold_remaining_.fill(0);
    previous_pose_target_pose_valid_ = false;
}

void FairinoControllerNode::streamCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!stream_mode_ || stream_command_mode_ == StreamCommandMode::kServoCart ||
        msg->position.size() != NUM_JOINTS) {
        return;
    }

    std::lock_guard<std::mutex> lock(stream_mutex_);
    pose_target_stream_active_ = false;

    JointArray raw_cu_target{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
        raw_cu_target[i] = msg->position[i] * RAD_TO_DEG;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    const double dt_sec = joint_filter_time_initialized_
        ? std::chrono::duration<double>(now_tp - last_joint_filter_update_time_).count()
        : 0.0;

    const auto smooth_result = joint_target_smoother_.update(raw_cu_target, dt_sec);

    last_joint_filter_update_time_ = now_tp;
    joint_filter_time_initialized_ = true;

    stream_target_deg_ = JointArrayToVector(smooth_result.filtered_target_deg);
}

void FairinoControllerNode::streamService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (!connected_) {
        response->success = false;
        response->message = "Robot is not connected.";
        return;
    }

    if (request->data) {
        if (executing_) {
            response->success = false;
            response->message = "Trajectory execution is active.";
            return;
        }

        if (!stream_mode_) {
            if (!dummy_mode_ &&
                (stream_command_mode_ == StreamCommandMode::kServoJ ||
                 stream_command_mode_ == StreamCommandMode::kServoCart)) {
                std::string failure_message;
                if (!startServoMotionSession("streaming", &failure_message)) {
                    response->success = false;
                    response->message = failure_message;
                    return;
                }
            }

            geometry_msgs::msg::Pose actual_pose;
            if (!dummy_mode_) {
                DescPose actual_tcp_pose{};
                const errno_t tcp_ret = robot_->GetActualTCPPose(0, &actual_tcp_pose);
                if (tcp_ret != 0) {
                    stopServoMotionSession("streaming preflight TCP pose failure");
                    std::ostringstream stream;
                    stream << "Controller-side prerequisite failure: failed to query actual TCP pose before streaming ("
                           << BuildFairinoErrorDetail(robot_.get(), tcp_ret) << ").";
                    response->success = false;
                    response->message = stream.str();
                    RCLCPP_ERROR(this->get_logger(), "%s", stream.str().c_str());
                    return;
                }
                actual_pose = teleop_slave::DescPoseToPose(actual_tcp_pose);
            } else {
                actual_pose = dummy_robot_pose_.pose;
            }
            {
                std::lock_guard<std::mutex> lock(actual_pose_mutex_);
                latest_actual_pose_ = actual_pose;
                actual_pose_received_ = true;
            }

            {
                std::lock_guard<std::mutex> lock(stream_mutex_);
                if (stream_command_mode_ == StreamCommandMode::kServoCart) {
                    const geometry_msgs::msg::Pose normalized_actual_pose =
                        teleop_slave::NormalizePoseQuaternion(actual_pose);
                    cartesian_stream_interpolator_.reset(
                        normalized_actual_pose,
                        std::chrono::steady_clock::now());
                    cartesian_stream_has_tracker_target_ = false;
                    cartesian_previous_command_pose_ = normalized_actual_pose;
                    cartesian_previous_command_pose_valid_ = true;
                } else {
                    JointPos actual;
                    if (!dummy_mode_) {
                        const errno_t joint_ret = robot_->GetActualJointPosDegree(0, &actual);
                        if (joint_ret != 0) {
                            if (stream_command_mode_ != StreamCommandMode::kMoveJ) {
                                stopServoMotionSession("streaming preflight joint-state failure");
                            }
                            std::ostringstream stream;
                            stream << "Controller-side prerequisite failure: failed to query actual joint positions before joint streaming ("
                                   << BuildFairinoErrorDetail(robot_.get(), joint_ret) << ").";
                            response->success = false;
                            response->message = stream.str();
                            RCLCPP_ERROR(this->get_logger(), "%s", stream.str().c_str());
                            return;
                        }
                    } else {
                        for (int i = 0; i < NUM_JOINTS; ++i) {
                            actual.jPos[i] = dummy_joint_positions_[i];
                        }
                    }
                    const JointArray actual_joints_deg = JointPosToArray(actual);
                    resetPoseTargetSolverState();
                    joint_filter_time_initialized_ = false;
                    joint_target_smoother_.reset(actual_joints_deg);
                    stream_target_deg_ = JointArrayToVector(actual_joints_deg);
                    prev_movej_cmd_ = actual;
                    has_prev_movej_cmd_ = true;
                    {
                        std::lock_guard<std::mutex> actual_joint_lock(actual_joint_mutex_);
                        latest_actual_joint_deg_ = actual_joints_deg;
                        latest_actual_joint_arrival_ = this->now();
                        actual_joint_received_ = true;
                    }
                }
            }

            consecutive_ik_failures_ = 0;
            last_successful_ik_time_ = this->now();
            pose_target_stream_active_ = false;
            stream_has_prev_ = false;
            stream_has_prev_prev_ = false;
            stream_prev_deg_.clear();
            stream_prev_prev_deg_.clear();
            stream_mode_ = true;
            servo_error_count_.store(0);
            consecutive_servoj_failures_ = 0;
            servo_cart_first_command_pending_.store(
                stream_command_mode_ == StreamCommandMode::kServoCart);
            std::thread(&FairinoControllerNode::streamLoop, this).detach();
            if (stream_command_mode_ == StreamCommandMode::kServoCart) {
                RCLCPP_INFO(this->get_logger(),
                            "Streaming mode enabled (servo_cart, servocart_mode=%s, sdk mode %d)",
                            teleop_slave::ServoCartModeToString(servocart_mode_).c_str(),
                            teleop_slave::ServoCartModeToSdkMode(servocart_mode_));
            } else if (stream_command_mode_ == StreamCommandMode::kMoveJ) {
                RCLCPP_INFO(this->get_logger(), "Streaming mode enabled (move_j)");
            } else {
                RCLCPP_INFO(this->get_logger(), "Streaming mode enabled (servo_j)");
            }
        }

        response->success = true;
        response->message = "Streaming enabled";
        return;
    }

    stream_mode_ = false;
    {
        std::lock_guard<std::mutex> lock(stream_mutex_);
        cartesian_stream_has_tracker_target_ = false;
        cartesian_previous_command_pose_valid_ = false;
        joint_filter_time_initialized_ = false;
        stream_has_prev_ = false;
        stream_has_prev_prev_ = false;
        stream_prev_deg_.clear();
        stream_prev_prev_deg_.clear();
    }
    resetPoseTargetSolverState();
    servo_cart_first_command_pending_.store(false);
    response->success = true;
    response->message = "Streaming disabled";
}

void FairinoControllerNode::logJointSmoothingDiagnostics(
    const JointArray& raw_target_deg,
    const JointArray& filtered_target_deg) {
    double max_j123_filter_delta_deg = 0.0;
    for (int i = 0; i < 3; ++i) {
        max_j123_filter_delta_deg = std::max(
            max_j123_filter_delta_deg,
            std::abs(raw_target_deg[i] - filtered_target_deg[i]));
    }

    if (max_j123_filter_delta_deg >= joint_deadband_deg_[0]) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Joint smoother damping J1-J3 by up to %.3f deg relative to raw IK targets.",
            max_j123_filter_delta_deg);
    }
}

void FairinoControllerNode::maybeWriteJointDebugCsv(
    const geometry_msgs::msg::Pose& pose_target,
    const JointArray& raw_target_deg,
    const JointArray& guarded_target_deg,
    const JointArray& filtered_target_deg,
    const std::array<bool, 3>& continuity_guarded_j123,
    const std::array<bool, 3>& reversal_guarded_j123,
    bool j123_direction_reversal_detected,
    double continuity_guard_pose_step_mm,
    double continuity_guard_pose_step_deg,
    double continuity_guard_max_jump_deg) {
    if (!enable_joint_debug_csv_ || !joint_debug_csv_stream_.is_open()) {
        return;
    }

    ++joint_debug_csv_counter_;
    if ((joint_debug_csv_counter_ - 1) % static_cast<size_t>(joint_debug_csv_decimation_) != 0) {
        return;
    }

    JointArray actual_joint_deg{};
    double actual_age_sec = std::numeric_limits<double>::quiet_NaN();
    bool has_actual_joint = false;
    {
        std::lock_guard<std::mutex> lock(actual_joint_mutex_);
        if (actual_joint_received_) {
            actual_joint_deg = latest_actual_joint_deg_;
            actual_age_sec = (this->now() - latest_actual_joint_arrival_).seconds();
            has_actual_joint = true;
        }
    }

    std::lock_guard<std::mutex> file_lock(joint_debug_csv_mutex_);
    const double stamp_sec = this->now().seconds();
    joint_debug_csv_stream_ << stamp_sec << ','
                            << pose_target.position.x << ','
                            << pose_target.position.y << ','
                            << pose_target.position.z << ','
                            << pose_target.orientation.x << ','
                            << pose_target.orientation.y << ','
                            << pose_target.orientation.z << ','
                            << pose_target.orientation.w;

    auto write_joint_array = [this](const JointArray& values) {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_debug_csv_stream_ << ',' << values[i];
        }
    };
    write_joint_array(raw_target_deg);
    write_joint_array(guarded_target_deg);
    write_joint_array(filtered_target_deg);

    if (has_actual_joint) {
        write_joint_array(actual_joint_deg);
    } else {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_debug_csv_stream_ << ',' << std::numeric_limits<double>::quiet_NaN();
        }
    }

    joint_debug_csv_stream_ << ',' << actual_age_sec
                            << ',' << (continuity_guarded_j123[0] ? 1 : 0)
                            << ',' << (continuity_guarded_j123[1] ? 1 : 0)
                            << ',' << (continuity_guarded_j123[2] ? 1 : 0)
                            << ',' << (reversal_guarded_j123[0] ? 1 : 0)
                            << ',' << (reversal_guarded_j123[1] ? 1 : 0)
                            << ',' << (reversal_guarded_j123[2] ? 1 : 0)
                            << ',' << (j123_direction_reversal_detected ? 1 : 0)
                            << ',' << continuity_guard_pose_step_mm
                            << ',' << continuity_guard_pose_step_deg
                            << ',' << continuity_guard_max_jump_deg
                            << '\n';
    joint_debug_csv_stream_.flush();
}

void FairinoControllerNode::logServoCartTrackingDiagnostics(
    const geometry_msgs::msg::Pose& target_pose) {
    geometry_msgs::msg::Pose actual_pose;
    {
        std::lock_guard<std::mutex> lock(actual_pose_mutex_);
        if (!actual_pose_received_) {
            return;
        }
        actual_pose = latest_actual_pose_;
    }

    const tf2::Vector3 delta(
        target_pose.position.x - actual_pose.position.x,
        target_pose.position.y - actual_pose.position.y,
        target_pose.position.z - actual_pose.position.z);
    tf2::Quaternion actual_q;
    tf2::Quaternion target_q;
    tf2::fromMsg(actual_pose.orientation, actual_q);
    tf2::fromMsg(target_pose.orientation, target_q);
    const double translation_error_mm = delta.length() * 1000.0;
    const double orientation_error_deg =
        teleop_slave::QuaternionAngularDistanceDegrees(actual_q, target_q);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "ServoCart target vs actual TCP error: %.1f mm, %.1f deg",
                         translation_error_mm, orientation_error_deg);
}

void FairinoControllerNode::streamLoop() {
    const double command_period_sec =
        stream_command_mode_ == StreamCommandMode::kMoveJ ? movej_cmd_period_sec_ : servo_cmd_period_sec_;
    const auto period = std::chrono::duration<double>(command_period_sec);
    const double timeout_sec = this->get_parameter("pose_target_timeout_sec").as_double();

    while (stream_mode_) {
        const auto start_time = std::chrono::steady_clock::now();

        if (pose_target_stream_active_ && pose_target_received_) {
            const rclcpp::Duration age = this->now() - latest_pose_target_arrival_;
            if (timeout_sec > 0.0 && age.seconds() > timeout_sec) {
                RCLCPP_ERROR(this->get_logger(),
                             "Pose target stream timed out after %.3f seconds. Stopping stream.",
                             age.seconds());
                stream_mode_ = false;
                break;
            }
        }

        if (stream_command_mode_ == StreamCommandMode::kServoCart) {
            geometry_msgs::msg::Pose target_pose;
            geometry_msgs::msg::Pose servocart_command_pose;
            {
                std::lock_guard<std::mutex> lock(stream_mutex_);
                target_pose = cartesian_stream_interpolator_.sample(start_time);
                servocart_command_pose = teleop_slave::BuildServoCartCommandPose(
                    servocart_mode_,
                    target_pose,
                    cartesian_previous_command_pose_,
                    cartesian_previous_command_pose_valid_);
                cartesian_previous_command_pose_ =
                    teleop_slave::NormalizePoseQuaternion(target_pose);
                cartesian_previous_command_pose_valid_ = true;
            }

            if (!executeServoCart(servocart_command_pose)) {
                consecutive_servoj_failures_++;
                if (consecutive_servoj_failures_ >= servoj_failure_limit_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "ServoCart streaming failed %d times in a row. Stopping stream.",
                                 consecutive_servoj_failures_);
                    stream_mode_ = false;
                    break;
                }
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Transient ServoCart failure (%d/%d). Retrying.",
                                     consecutive_servoj_failures_, servoj_failure_limit_);
            } else {
                consecutive_servoj_failures_ = 0;
                logServoCartTrackingDiagnostics(target_pose);
            }
        } else {
            std::vector<double> target;
            {
                std::lock_guard<std::mutex> lock(stream_mutex_);
                const bool interpolate = this->get_parameter("interpolate_stream").as_bool();
                if (interpolate && stream_has_prev_ && !stream_prev_deg_.empty() &&
                    stream_prev_deg_.size() == stream_target_deg_.size()) {
                    const auto now_tp = std::chrono::steady_clock::now();
                    const auto interval = std::chrono::duration<double>(
                        stream_target_time_ - stream_prev_time_).count();
                    const auto elapsed = std::chrono::duration<double>(
                        now_tp - stream_prev_time_).count();
                    const double t = (interval > 1e-6)
                        ? std::clamp(elapsed / interval, 0.0, 1.0)
                        : 1.0;
                    target.resize(stream_target_deg_.size());

                    for (size_t i = 0; i < target.size(); ++i) {
                        target[i] = stream_prev_deg_[i] +
                            t * (stream_target_deg_[i] - stream_prev_deg_[i]);
                    }
                } else {
                    target = stream_target_deg_;
                }
            }

            const bool command_ok =
                stream_command_mode_ == StreamCommandMode::kMoveJ
                    ? executeMoveJ(target)
                    : executeServoJ(target);
            if (!command_ok) {
                consecutive_servoj_failures_++;
                if (consecutive_servoj_failures_ >= servoj_failure_limit_) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "%s streaming failed %d times in a row. Stopping stream.",
                        stream_command_mode_ == StreamCommandMode::kMoveJ ? "MoveJ" : "ServoJ",
                        consecutive_servoj_failures_);
                    stream_mode_ = false;
                    break;
                }
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Transient %s failure (%d/%d). Retrying.",
                    stream_command_mode_ == StreamCommandMode::kMoveJ ? "MoveJ" : "ServoJ",
                    consecutive_servoj_failures_, servoj_failure_limit_);
            } else {
                consecutive_servoj_failures_ = 0;
            }
        }

        const auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    stopServoMotionSession("stream loop exit");
    resetPoseTargetSolverState();
}

void FairinoControllerNode::publishJointStates() {
    if (!connected_) {
        return;
    }

    JointPos jpos;
    if (!dummy_mode_) {
        if (!robot_) {
            return;
        }

        const errno_t ret = robot_->GetActualJointPosDegree(0, &jpos);
        if (ret != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "GetActualJointPosDegree failed: %d", ret);
            return;
        }
    } else {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            jpos.jPos[i] = dummy_joint_positions_[i];
        }
    }

    if (!dummy_mode_) {
        bool all_zero = true;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            if (std::abs(jpos.jPos[i]) > 0.001) {
                all_zero = false;
                break;
            }
        }

        if (!all_zero) {
            last_valid_joints_ = jpos;
            has_valid_joints_ = true;
        } else if (has_valid_joints_) {
            jpos = last_valid_joints_;
        } else {
            return;
        }
    }

    sensor_msgs::msg::JointState joint_msg;
    const rclcpp::Time joint_sample_time = this->now();
    joint_msg.header.stamp = joint_sample_time;
    joint_msg.name = joint_names_;
    joint_msg.position.resize(NUM_JOINTS);
    const JointArray actual_joint_deg = JointPosToArray(jpos);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_msg.position[i] = jpos.jPos[i] * DEG_TO_RAD;
    }
    joint_pub_->publish(joint_msg);
    {
        std::lock_guard<std::mutex> lock(actual_joint_mutex_);
        latest_actual_joint_deg_ = actual_joint_deg;
        latest_actual_joint_arrival_ = joint_sample_time;
        actual_joint_received_ = true;
    }

    if (dummy_mode_) {
        dummy_robot_pose_.header.stamp = this->now();
        {
            std::lock_guard<std::mutex> lock(actual_pose_mutex_);
            latest_actual_pose_ = dummy_robot_pose_.pose;
            actual_pose_received_ = true;
        }
        pose_pub_->publish(dummy_robot_pose_);
        return;
    }

    if (!robot_ || !pose_pub_) {
        return;
    }

    DescPose tcp_pose;
    const errno_t ret_pose = robot_->GetActualTCPPose(0, &tcp_pose);
    if (ret_pose != 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GetActualTCPPose failed: %d", ret_pose);
        return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose = teleop_slave::DescPoseToPose(tcp_pose);
    {
        std::lock_guard<std::mutex> lock(actual_pose_mutex_);
        latest_actual_pose_ = pose_msg.pose;
        actual_pose_received_ = true;
    }
    pose_pub_->publish(pose_msg);
}

static std::shared_ptr<FairinoControllerNode> g_node = nullptr;

void signal_handler(int signum) {
    (void)signum;
    if (g_node) {
        g_node->shutdown();
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    g_node = std::make_shared<FairinoControllerNode>();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    if (!g_node->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to initialize Fairino controller node");
        g_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    try {
        rclcpp::spin(g_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Unhandled exception: %s", e.what());
    }

    if (g_node) {
        g_node->shutdown();
        g_node.reset();
    }
    rclcpp::shutdown();
    return 0;
}
