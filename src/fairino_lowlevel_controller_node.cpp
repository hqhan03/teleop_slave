#include "teleop_slave/fairino_lowlevel_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <sstream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "teleop_slave/fr5_teleop_utils.hpp"

using namespace std::chrono_literals;

FairinoControllerNode::FairinoControllerNode()
    : Node("fairino_lowlevel_controller_node") {
    this->declare_parameter("robot_ip", "192.168.58.2");
    this->declare_parameter("dummy_mode", false);
    this->declare_parameter("expected_tcp_id", -1);
    this->declare_parameter("expected_wobj_id", -1);
    this->declare_parameter("ik_failure_limit", 5);
    this->declare_parameter("ik_failure_timeout_sec", 0.75);
    this->declare_parameter("allow_orientation_fallback", true);
    this->declare_parameter("ik_position_backoff_steps", 4);
    this->declare_parameter("pose_target_timeout_sec", 0.30);
    this->declare_parameter("servo_cmd_period_sec", 0.008);
    this->declare_parameter("servoj_max_step_deg", 2.0);
    this->declare_parameter("servoj_failure_limit", 3);

    robot_ip_ = this->get_parameter("robot_ip").as_string();
    dummy_mode_ = this->get_parameter("dummy_mode").as_bool();
    servo_cmd_period_sec_ = this->get_parameter("servo_cmd_period_sec").as_double();
    servoj_max_step_deg_ = this->get_parameter("servoj_max_step_deg").as_double();
    servoj_failure_limit_ = this->get_parameter("servoj_failure_limit").as_int();

    RCLCPP_INFO(this->get_logger(), "Fairino Controller Node");
    RCLCPP_INFO(this->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Dummy Mode: %s", dummy_mode_ ? "TRUE (Simulation)" : "FALSE (Hardware)");
    RCLCPP_INFO(this->get_logger(), "  ServoJ cmd period: %.4f s", servo_cmd_period_sec_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ max step: %.2f deg", servoj_max_step_deg_);
    RCLCPP_INFO(this->get_logger(), "  ServoJ failure limit: %d", servoj_failure_limit_);

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
    last_successful_ik_time_ = this->now();
}

FairinoControllerNode::~FairinoControllerNode() {
    shutdown();
}

bool FairinoControllerNode::initialize() {
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
    robot_->Mode(0);
    robot_->RobotEnable(1);
    robot_->SetSpeed(20);

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

    robot_->ServoMoveEnd();
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

    if (dummy_mode_) {
        dummy_robot_pose_ = *msg;
        dummy_robot_pose_.header.stamp = this->now();
    }

    if (!stream_mode_ || !connected_) {
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
    std::lock_guard<std::mutex> lock(stream_mutex_);
    stream_target_deg_.resize(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        stream_target_deg_[i] = result.joint_target_deg[i];
    }
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

    if (!dummy_mode_) {
        robot_->ResetAllError();
        std::this_thread::sleep_for(20ms);
        robot_->Mode(0);
        std::this_thread::sleep_for(20ms);
        robot_->ServoMoveStart();
        std::this_thread::sleep_for(50ms);
    }

    std::thread([this]() {
        controlLoop();
        if (!dummy_mode_ && robot_) {
            robot_->ServoMoveEnd();
        }
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

    JointPos current_actual;
    robot_->GetActualJointPosDegree(0, &current_actual);

    JointPos cmd;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        const double error = target_deg[i] - current_actual.jPos[i];
        const double step = std::clamp(error, -servoj_max_step_deg_, servoj_max_step_deg_);
        cmd.jPos[i] = current_actual.jPos[i] + step;
    }

    ExaxisPos epos(0, 0, 0, 0);
    const float t = static_cast<float>(servo_cmd_period_sec_);

    const errno_t ret = robot_->ServoJ(&cmd, &epos, 0.0f, 0.0f, t, 0.0f, 0.0f, 0);
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
        return false;
    }

    return true;
}

void FairinoControllerNode::streamCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!stream_mode_ || msg->position.size() != NUM_JOINTS) {
        return;
    }

    std::lock_guard<std::mutex> lock(stream_mutex_);
    pose_target_stream_active_ = false;
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
                robot_->ResetAllError();
                std::this_thread::sleep_for(20ms);
                robot_->Mode(0);
                std::this_thread::sleep_for(20ms);
                robot_->ServoMoveStart();
                std::this_thread::sleep_for(50ms);
            }

            JointPos actual;
            if (!dummy_mode_) {
                robot_->GetActualJointPosDegree(0, &actual);
            } else {
                for (int i = 0; i < NUM_JOINTS; ++i) {
                    actual.jPos[i] = dummy_joint_positions_[i];
                }
            }

            {
                std::lock_guard<std::mutex> lock(stream_mutex_);
                stream_target_deg_.resize(NUM_JOINTS);
                for (int i = 0; i < NUM_JOINTS; ++i) {
                    stream_target_deg_[i] = actual.jPos[i];
                }
            }

            consecutive_ik_failures_ = 0;
            last_successful_ik_time_ = this->now();
            pose_target_stream_active_ = false;
            stream_mode_ = true;
            servo_error_count_.store(0);
            consecutive_servoj_failures_ = 0;
            std::thread(&FairinoControllerNode::streamLoop, this).detach();
            RCLCPP_INFO(this->get_logger(), "Streaming mode enabled");
        }

        response->success = true;
        response->message = "Streaming enabled";
        return;
    }

    stream_mode_ = false;
    response->success = true;
    response->message = "Streaming disabled";
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

        std::vector<double> target;
        {
            std::lock_guard<std::mutex> lock(stream_mutex_);
            target = stream_target_deg_;
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

        const auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    if (!dummy_mode_ && robot_) {
        robot_->ServoMoveEnd();
    }
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
