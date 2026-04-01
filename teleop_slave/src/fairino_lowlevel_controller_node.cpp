#include "teleop_slave/fairino_lowlevel_controller_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
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
    this->declare_parameter("servocart_filter_t", 0.0);
    this->declare_parameter("servocart_gain", 0.0);
    this->declare_parameter("servoj_max_step_deg", 0.8);
    this->declare_parameter("servoj_filter_t", 0.0);
    this->declare_parameter("servoj_gain", 0.0);
    this->declare_parameter("servoj_failure_limit", 3);
    this->declare_parameter("interpolate_stream", true);
    this->declare_parameter("joint_deadband_deg",
                            std::vector<double>{0.08, 0.08, 0.08, 0.12, 0.12, 0.12});
    this->declare_parameter("joint_velocity_limit_deg_s",
                            std::vector<double>{35.0, 35.0, 40.0, 120.0, 120.0, 150.0});
    this->declare_parameter("ruckig_enabled", false);
    this->declare_parameter("joint_acceleration_limit_deg_s2",
                            std::vector<double>{200.0, 200.0, 250.0, 400.0, 400.0, 500.0});
    this->declare_parameter("joint_jerk_limit_deg_s3",
                            std::vector<double>{1000.0, 1000.0, 1200.0, 2000.0, 2000.0, 2500.0});

    robot_ip_ = this->get_parameter("robot_ip").as_string();
    dummy_mode_ = this->get_parameter("dummy_mode").as_bool();
    const std::string stream_command_mode =
        this->get_parameter("stream_command_mode").as_string();
    if (stream_command_mode == "servo_cart") {
        stream_command_mode_ = StreamCommandMode::kServoCart;
    } else if (stream_command_mode == "servo_j") {
        stream_command_mode_ = StreamCommandMode::kServoJ;
    } else {
        throw std::runtime_error(
            "stream_command_mode must be either 'servo_cart' or 'servo_j'");
    }
    global_speed_percent_ = this->get_parameter("global_speed_percent").as_int();
    servo_cmd_period_sec_ = this->get_parameter("servo_cmd_period_sec").as_double();
    pose_target_nominal_period_sec_ =
        this->get_parameter("pose_target_nominal_period_sec").as_double();
    servocart_filter_t_ = this->get_parameter("servocart_filter_t").as_double();
    servocart_gain_ = this->get_parameter("servocart_gain").as_double();
    servoj_max_step_deg_ = this->get_parameter("servoj_max_step_deg").as_double();
    servoj_filter_t_ = this->get_parameter("servoj_filter_t").as_double();
    servoj_gain_ = this->get_parameter("servoj_gain").as_double();
    servoj_failure_limit_ = this->get_parameter("servoj_failure_limit").as_int();
    joint_deadband_deg_ = VectorToJointArray(
        this->get_parameter("joint_deadband_deg").as_double_array(), "joint_deadband_deg");
    joint_velocity_limit_deg_s_ = VectorToJointArray(
        this->get_parameter("joint_velocity_limit_deg_s").as_double_array(),
        "joint_velocity_limit_deg_s");
    joint_target_smoother_.configure(joint_deadband_deg_, joint_velocity_limit_deg_s_);

    ruckig_enabled_ = this->get_parameter("ruckig_enabled").as_bool();
    joint_acceleration_limit_deg_s2_ = VectorToJointArray(
        this->get_parameter("joint_acceleration_limit_deg_s2").as_double_array(),
        "joint_acceleration_limit_deg_s2");
    joint_jerk_limit_deg_s3_ = VectorToJointArray(
        this->get_parameter("joint_jerk_limit_deg_s3").as_double_array(),
        "joint_jerk_limit_deg_s3");
    if (ruckig_enabled_) {
        teleop_slave::RuckigJointTrajectoryGenerator::Config ruckig_config;
        ruckig_config.cycle_time_sec = servo_cmd_period_sec_;
        ruckig_config.max_velocity_deg_s = joint_velocity_limit_deg_s_;
        ruckig_config.max_acceleration_deg_s2 = joint_acceleration_limit_deg_s2_;
        ruckig_config.max_jerk_deg_s3 = joint_jerk_limit_deg_s3_;
        ruckig_trajectory_generator_.configure(ruckig_config);
    }

    RCLCPP_INFO(this->get_logger(), "Fairino Controller Node");
    RCLCPP_INFO(this->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Dummy Mode: %s", dummy_mode_ ? "TRUE (Simulation)" : "FALSE (Hardware)");
    RCLCPP_INFO(this->get_logger(), "  Stream command mode: %s",
                stream_command_mode_ == StreamCommandMode::kServoCart ? "servo_cart" : "servo_j");
    RCLCPP_INFO(this->get_logger(), "  Global speed percent: %d", global_speed_percent_);
    RCLCPP_INFO(this->get_logger(), "  Servo stream cmd period: %.4f s", servo_cmd_period_sec_);
    RCLCPP_INFO(this->get_logger(), "  Pose target nominal period: %.4f s",
                pose_target_nominal_period_sec_);
    RCLCPP_INFO(this->get_logger(), "  ServoCart filter_t: %.4f s", servocart_filter_t_);
    RCLCPP_INFO(this->get_logger(), "  ServoCart gain: %.2f", servocart_gain_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ max step: %.2f deg", servoj_max_step_deg_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ filter_t: %.4f s", servoj_filter_t_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ gain: %.2f", servoj_gain_);
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
    RCLCPP_INFO(this->get_logger(), "  Ruckig trajectory smoother: %s",
                ruckig_enabled_ ? "ENABLED" : "DISABLED");

    traj_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/trajectory_points", 10,
        std::bind(&FairinoControllerNode::trajectoryCallback, this, std::placeholders::_1));

    pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/fr5/pose_target", 10,
        std::bind(&FairinoControllerNode::poseTargetCallback, this, std::placeholders::_1));

    legacy_pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/curobo/pose_target", 10,
        std::bind(&FairinoControllerNode::poseTargetCallback, this, std::placeholders::_1));

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
    if (dummy_mode_) {
        return;
    }

    std::lock_guard<std::mutex> lock(servo_motion_session_mutex_);
    servo_cart_first_command_pending_.store(false);
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
    has_prev_servoj_cmd_ = false;
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

    const auto result = teleop_slave::SolvePoseTargetIK(*robot_, *msg, options);
    if (!result.success) {
        consecutive_ik_failures_++;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Pose target IK rejected: %s", result.message.c_str());
        const double ik_failure_timeout_sec =
            this->get_parameter("ik_failure_timeout_sec").as_double();
        const bool exceeded_count =
            consecutive_ik_failures_ >= this->get_parameter("ik_failure_limit").as_int();
        const bool exceeded_time =
            (this->now() - last_successful_ik_time_).seconds() >= ik_failure_timeout_sec;
        if (exceeded_count && exceeded_time) {
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

    JointArray filtered_target_deg{};
    bool j123_direction_reversal_detected = false;
    {
        std::lock_guard<std::mutex> lock(stream_mutex_);
        const auto now_tp = std::chrono::steady_clock::now();
        const double dt_sec = joint_filter_time_initialized_
            ? std::chrono::duration<double>(now_tp - last_joint_filter_update_time_).count()
            : 0.0;

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
                        break;
                    }
                }
            }
            raw_prev_ik_step_deg_ = current_raw_step_deg;
            raw_has_prev_ik_step_ = true;
        }
        raw_prev_ik_target_deg_ = raw_target_deg;
        raw_has_prev_ik_target_ = true;

        if (ruckig_enabled_) {
            // Ruckig mode: just store the target; streamLoop will consume it
            ruckig_target_deg_ = raw_target_deg;
            ruckig_target_received_ = true;
            filtered_target_deg = raw_target_deg;  // for diagnostics log
        } else {
            // Legacy mode: velocity-only smoother + linear interpolation bookkeeping
            const auto smooth_result = joint_target_smoother_.update(raw_target_deg, dt_sec);
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
    }

    if (j123_direction_reversal_detected) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Detected rapid J1-J3 raw IK direction reversal. This is a likely chatter signature from tracker translation jitter.");
    }
    logJointSmoothingDiagnostics(raw_target_deg, filtered_target_deg);
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

    JointPos cmd;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        const double step = target_deg[i] - prev_servoj_cmd_.jPos[i];
        const double clamped = std::clamp(step, -servoj_max_step_deg_, servoj_max_step_deg_);
        cmd.jPos[i] = prev_servoj_cmd_.jPos[i] + clamped;
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

bool FairinoControllerNode::executeServoCart(const geometry_msgs::msg::Pose& target_pose) {
    if (dummy_mode_) {
        const geometry_msgs::msg::Pose delta_pose =
            teleop_slave::NormalizePoseQuaternion(target_pose);
        dummy_robot_pose_.pose.position.x += delta_pose.position.x;
        dummy_robot_pose_.pose.position.y += delta_pose.position.y;
        dummy_robot_pose_.pose.position.z += delta_pose.position.z;

        tf2::Quaternion current_q;
        tf2::Quaternion delta_q;
        tf2::fromMsg(dummy_robot_pose_.pose.orientation, current_q);
        tf2::fromMsg(delta_pose.orientation, delta_q);
        dummy_robot_pose_.pose.orientation =
            tf2::toMsg(teleop_slave::NormalizeStreamQuaternion(delta_q * current_q));
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
    const bool first_command_after_session_start =
        servo_cart_first_command_pending_.exchange(false);
    const errno_t ret = robot_->ServoCart(1, &desc_pose, exaxis, pos_gain, 0.0f, 0.0f, t, ft, g);
    if (ret != 0) {
        const int err_cnt = servo_error_count_.fetch_add(1) + 1;
        if (err_cnt <= 3 || err_cnt % 100 == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "ServoCart failed (count=%d, %s)",
                        err_cnt, BuildFairinoErrorDetail(robot_.get(), ret).c_str());
            if (first_command_after_session_start) {
                RCLCPP_ERROR(this->get_logger(),
                             "First ServoCart command after servo session start failed (%s). ServoMoveStart succeeded, so the controller is rejecting the Cartesian stream itself or a runtime prerequisite is still unmet.",
                             BuildFairinoErrorDetail(robot_.get(), ret).c_str());
            }
            if (ret == -4) {
                RCLCPP_WARN(this->get_logger(),
                            "ret=-4 is a Fairino XML-RPC execution failure. This node now uses the documented ServoCart pattern: incremental base-frame commands, pos_gain=1, and cmdT=%.3f s.",
                            servo_cmd_period_sec_);
            }
        }
        robot_->ResetAllError();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return false;
    }

    return true;
}

void FairinoControllerNode::streamCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!stream_mode_ || stream_command_mode_ != StreamCommandMode::kServoJ ||
        msg->position.size() != NUM_JOINTS) {
        return;
    }

    std::lock_guard<std::mutex> lock(stream_mutex_);
    pose_target_stream_active_ = false;
    stream_target_deg_.resize(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        stream_target_deg_[i] = msg->position[i] * RAD_TO_DEG;
    }
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
            if (!dummy_mode_) {
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
                            stopServoMotionSession("streaming preflight joint-state failure");
                            std::ostringstream stream;
                            stream << "Controller-side prerequisite failure: failed to query actual joint positions before ServoJ streaming ("
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
                    joint_target_smoother_.reset(actual_joints_deg);
                    if (ruckig_enabled_) {
                        ruckig_trajectory_generator_.reset(actual_joints_deg);
                        ruckig_target_deg_ = actual_joints_deg;
                        ruckig_target_received_ = false;
                    }
                    joint_filter_time_initialized_ = false;
                    raw_has_prev_ik_target_ = false;
                    raw_has_prev_ik_step_ = false;
                    stream_target_deg_ = JointArrayToVector(actual_joints_deg);
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
            RCLCPP_INFO(this->get_logger(), "Streaming mode enabled (%s)",
                        stream_command_mode_ == StreamCommandMode::kServoCart ? "servo_cart" : "servo_j");
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
        raw_has_prev_ik_target_ = false;
        raw_has_prev_ik_step_ = false;
        ruckig_target_received_ = false;
        stream_has_prev_ = false;
        stream_has_prev_prev_ = false;
        stream_prev_deg_.clear();
        stream_prev_prev_deg_.clear();
    }
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
    const auto period = std::chrono::duration<double>(servo_cmd_period_sec_);
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
            geometry_msgs::msg::Pose incremental_target_pose;
            {
                std::lock_guard<std::mutex> lock(stream_mutex_);
                target_pose = cartesian_stream_interpolator_.sample(start_time);
                if (!cartesian_previous_command_pose_valid_) {
                    cartesian_previous_command_pose_ =
                        teleop_slave::NormalizePoseQuaternion(target_pose);
                    cartesian_previous_command_pose_valid_ = true;
                    incremental_target_pose = teleop_slave::MakeIdentityPose();
                } else {
                    incremental_target_pose =
                        teleop_slave::ComputeIncrementalPoseDeltaBaseFrame(
                            cartesian_previous_command_pose_, target_pose);
                    cartesian_previous_command_pose_ =
                        teleop_slave::NormalizePoseQuaternion(target_pose);
                }
            }

            if (!executeServoCart(incremental_target_pose)) {
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
                if (ruckig_enabled_) {
                    // Ruckig mode: advance one jerk-limited timestep
                    if (ruckig_target_received_) {
                        ruckig_trajectory_generator_.setTarget(ruckig_target_deg_);
                    }
                    const auto next_pos = ruckig_trajectory_generator_.update();
                    target.assign(next_pos.begin(), next_pos.end());
                } else {
                    // Legacy mode: linear interpolation between 50Hz targets
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
            }

            if (!executeServoJ(target)) {
                consecutive_servoj_failures_++;
                if (consecutive_servoj_failures_ >= servoj_failure_limit_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "ServoJ streaming failed %d times in a row. Stopping stream.",
                                 consecutive_servoj_failures_);
                    stream_mode_ = false;
                    break;
                }
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Transient ServoJ failure (%d/%d). Retrying.",
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
    joint_msg.header.stamp = this->now();
    joint_msg.name = joint_names_;
    joint_msg.position.resize(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_msg.position[i] = jpos.jPos[i] * DEG_TO_RAD;
    }
    joint_pub_->publish(joint_msg);

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
