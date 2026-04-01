#include "teleop_slave/fairino_slave_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <termios.h>
#include <unistd.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

FairinoSlaveNode::FairinoSlaveNode() : Node("fairino_slave_node") {
    this->declare_parameter("workspace_radius", 0.85);
    this->declare_parameter("min_xyz", std::vector<double>{-0.85, -0.85, 0.05});
    this->declare_parameter("max_xyz", std::vector<double>{0.85, 0.85, 1.10});
    this->declare_parameter("max_linear_step_m", 0.05);
    this->declare_parameter("max_angular_step_deg", 10.0);
    this->declare_parameter("tracker_to_robot_axes", std::vector<int64_t>{0, 1, 2});
    this->declare_parameter("tracker_to_robot_signs", std::vector<double>{1.0, 1.0, 1.0});
    this->declare_parameter("translation_scale_xyz", std::vector<double>{1.0, 1.0, 1.0});
    this->declare_parameter("tracker_to_robot_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("orientation_mode", "position_only");
    this->declare_parameter("filter_cutoff_hz", 5.0);
    this->declare_parameter("filter_sample_rate_hz", 50.0);
    this->declare_parameter("orientation_smoothing_alpha", 0.3);
    this->declare_parameter("position_deadzone_m", 0.001);

    const auto axes = this->get_parameter("tracker_to_robot_axes").as_integer_array();
    if (axes.size() != 3) {
        throw std::runtime_error("tracker_to_robot_axes must have exactly 3 entries");
    }
    for (size_t i = 0; i < 3; ++i) {
        tracker_to_robot_axes_[i] = static_cast<int>(axes[i]);
    }

    const auto signs = this->get_parameter("tracker_to_robot_signs").as_double_array();
    if (signs.size() != 3) {
        throw std::runtime_error("tracker_to_robot_signs must have exactly 3 entries");
    }
    for (size_t i = 0; i < 3; ++i) {
        tracker_to_robot_signs_[i] = signs[i];
    }

    const auto scales = this->get_parameter("translation_scale_xyz").as_double_array();
    if (scales.size() != 3) {
        throw std::runtime_error("translation_scale_xyz must have exactly 3 entries");
    }
    translation_scale_xyz_.setValue(scales[0], scales[1], scales[2]);

    const auto min_xyz = this->get_parameter("min_xyz").as_double_array();
    const auto max_xyz = this->get_parameter("max_xyz").as_double_array();
    if (min_xyz.size() != 3 || max_xyz.size() != 3) {
        throw std::runtime_error("min_xyz and max_xyz must have exactly 3 entries");
    }
    min_xyz_.setValue(min_xyz[0], min_xyz[1], min_xyz[2]);
    max_xyz_.setValue(max_xyz[0], max_xyz[1], max_xyz[2]);

    const auto basis_rpy = this->get_parameter("tracker_to_robot_rpy_deg").as_double_array();
    if (basis_rpy.size() != 3) {
        throw std::runtime_error("tracker_to_robot_rpy_deg must have exactly 3 entries");
    }
    for (size_t i = 0; i < 3; ++i) {
        tracker_to_robot_basis_rpy_deg_[i] = basis_rpy[i];
    }
    tracker_to_robot_basis_ = teleop_slave::QuaternionFromRPYDegrees(
        tf2::Vector3(tracker_to_robot_basis_rpy_deg_[0],
                     tracker_to_robot_basis_rpy_deg_[1],
                     tracker_to_robot_basis_rpy_deg_[2]));
    orientation_mode_ = teleop_slave::ParseOrientationMode(
        this->get_parameter("orientation_mode").as_string());

    logCalibrationConfiguration();

    manus_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/manus/wrist_pose", 10,
        std::bind(&FairinoSlaveNode::manusPoseCallback, this, std::placeholders::_1));

    robot_actual_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_pose", 10,
        std::bind(&FairinoSlaveNode::robotPoseCallback, this, std::placeholders::_1));

    fr5_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fr5/pose_target", 10);
    legacy_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/curobo/pose_target", 10);
    stream_client_ = this->create_client<std_srvs::srv::SetBool>("/enable_streaming");

    RCLCPP_INFO(this->get_logger(), "FR5 tracker teleop ready. Press SPACE to clutch to current robot pose.");
    RCLCPP_INFO(this->get_logger(), "Orientation mode: %s",
                teleop_slave::OrientationModeToString(orientation_mode_).c_str());

    kb_thread_ = std::thread(&FairinoSlaveNode::keyboardThread, this);
}

FairinoSlaveNode::~FairinoSlaveNode() {
    running_ = false;
    if (kb_thread_.joinable()) {
        kb_thread_.join();
    }
}

void FairinoSlaveNode::logCalibrationConfiguration() const {
    RCLCPP_INFO(this->get_logger(),
                "FR5 tracker config source: values loaded from --params-file / node parameters");
    RCLCPP_INFO(this->get_logger(),
                "Tracker calibration: axes=[%d, %d, %d] signs=[%.1f, %.1f, %.1f] "
                "scale=[%.3f, %.3f, %.3f] raw_basis_rpy_deg=[%.1f, %.1f, %.1f] "
                "basis_quat_xyzw=[%.6f, %.6f, %.6f, %.6f]",
                tracker_to_robot_axes_[0], tracker_to_robot_axes_[1], tracker_to_robot_axes_[2],
                tracker_to_robot_signs_[0], tracker_to_robot_signs_[1], tracker_to_robot_signs_[2],
                translation_scale_xyz_.x(), translation_scale_xyz_.y(), translation_scale_xyz_.z(),
                tracker_to_robot_basis_rpy_deg_[0],
                tracker_to_robot_basis_rpy_deg_[1],
                tracker_to_robot_basis_rpy_deg_[2],
                tracker_to_robot_basis_.x(),
                tracker_to_robot_basis_.y(),
                tracker_to_robot_basis_.z(),
                tracker_to_robot_basis_.w());
}

void FairinoSlaveNode::robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    latest_robot_pose_ = *msg;
    robot_pose_received_ = true;
}

void FairinoSlaveNode::keyboardThread() {
    struct termios oldt;
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (running_ && rclcpp::ok()) {
        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == ' ') {
                if (!streaming_started_) {
                    offset_set_ = true;
                    clutch_initialized_ = false;
                    startStreaming();
                } else {
                    stopStreaming();
                }
            } else if (ch == 'q' || ch == 27) {
                running_ = false;
                rclcpp::shutdown();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

geometry_msgs::msg::PoseStamped FairinoSlaveNode::buildTargetPose(
    const geometry_msgs::msg::PoseStamped& manus_pose) const {
    const tf2::Vector3 raw_tracker_position(
        manus_pose.pose.position.x,
        manus_pose.pose.position.y,
        manus_pose.pose.position.z);
    const tf2::Vector3 mapped_tracker_position = teleop_slave::ScaleVector(
        teleop_slave::ApplyAxisMapping(raw_tracker_position, tracker_to_robot_axes_, tracker_to_robot_signs_),
        translation_scale_xyz_);
    const tf2::Vector3 delta_position = mapped_tracker_position - clutch_tracker_position_;
    const tf2::Vector3 desired_position = base_robot_position_ + delta_position;

    tf2::Quaternion current_tracker_orientation;
    tf2::fromMsg(manus_pose.pose.orientation, current_tracker_orientation);

    const tf2::Quaternion desired_orientation = teleop_slave::ComputeMappedOrientation(
        base_robot_orientation_,
        tracker_zero_orientation_,
        current_tracker_orientation,
        tracker_to_robot_basis_,
        orientation_mode_);

    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = this->now();
    target.header.frame_id = "base_link";
    target.pose.position.x = desired_position.x();
    target.pose.position.y = desired_position.y();
    target.pose.position.z = desired_position.z();
    target.pose.orientation = tf2::toMsg(desired_orientation);

    const double deadzone = this->get_parameter("position_deadzone_m").as_double();
    if (clutch_initialized_ && deadzone > 0.0) {
        const tf2::Vector3 last_pos(last_target_pose_.position.x,
                                    last_target_pose_.position.y,
                                    last_target_pose_.position.z);
        if ((desired_position - last_pos).length() < deadzone) {
            target.pose.position.x = last_pos.x();
            target.pose.position.y = last_pos.y();
            target.pose.position.z = last_pos.z();
        }
    }

    const geometry_msgs::msg::Pose* previous = clutch_initialized_ ? &last_target_pose_ : nullptr;
    target.pose = teleop_slave::ClampPoseTarget(
        target.pose,
        previous,
        min_xyz_,
        max_xyz_,
        this->get_parameter("workspace_radius").as_double(),
        this->get_parameter("max_linear_step_m").as_double(),
        this->get_parameter("max_angular_step_deg").as_double());
    return target;
}

void FairinoSlaveNode::manusPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!streaming_started_ && !offset_set_) {
        return;
    }

    if (offset_set_) {
        if (!robot_pose_received_) {
            RCLCPP_WARN(this->get_logger(),
                        "Robot pose not yet received on /robot_pose. Is fairino_lowlevel_controller_node running?");
            offset_set_ = false;
            streaming_started_ = false;
            return;
        }

        const tf2::Vector3 raw_tracker_position(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z);
        clutch_tracker_position_ = teleop_slave::ScaleVector(
            teleop_slave::ApplyAxisMapping(raw_tracker_position, tracker_to_robot_axes_, tracker_to_robot_signs_),
            translation_scale_xyz_);
        base_robot_position_.setValue(
            latest_robot_pose_.pose.position.x,
            latest_robot_pose_.pose.position.y,
            latest_robot_pose_.pose.position.z);
        tf2::fromMsg(latest_robot_pose_.pose.orientation, base_robot_orientation_);
        tf2::fromMsg(msg->pose.orientation, tracker_zero_orientation_);
        last_target_pose_ = latest_robot_pose_.pose;
        smoothed_target_pose_ = latest_robot_pose_.pose;

        const double cutoff = this->get_parameter("filter_cutoff_hz").as_double();
        const double sample_rate = this->get_parameter("filter_sample_rate_hz").as_double();
        for (auto& f : position_filters_) {
            f.configure(cutoff, sample_rate);
        }
        position_filters_[0].reset(latest_robot_pose_.pose.position.x);
        position_filters_[1].reset(latest_robot_pose_.pose.position.y);
        position_filters_[2].reset(latest_robot_pose_.pose.position.z);
        butterworth_configured_ = true;

        smoothing_initialized_ = true;
        clutch_initialized_ = true;
        clutch_warmup_remaining_ = CLUTCH_WARMUP_SAMPLES;
        offset_set_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "Tracker clutch set to robot pose [%.3f, %.3f, %.3f]. "
                    "Warming up filter (%d samples)...",
                    base_robot_position_.x(), base_robot_position_.y(), base_robot_position_.z(),
                    CLUTCH_WARMUP_SAMPLES);
    }

    geometry_msgs::msg::PoseStamped target = buildTargetPose(*msg);

    if (butterworth_configured_ && smoothing_initialized_) {
        target.pose.position.x = position_filters_[0].filter(target.pose.position.x);
        target.pose.position.y = position_filters_[1].filter(target.pose.position.y);
        target.pose.position.z = position_filters_[2].filter(target.pose.position.z);

        const double ori_alpha = std::clamp(
            this->get_parameter("orientation_smoothing_alpha").as_double(), 0.0, 1.0);
        if (ori_alpha < 1.0) {
            tf2::Quaternion prev_q, new_q;
            tf2::fromMsg(smoothed_target_pose_.orientation, prev_q);
            tf2::fromMsg(target.pose.orientation, new_q);
            target.pose.orientation = tf2::toMsg(prev_q.slerp(new_q, ori_alpha).normalized());
        }
    }
    smoothed_target_pose_ = target.pose;
    smoothing_initialized_ = true;

    last_target_pose_ = target.pose;

    if (clutch_warmup_remaining_ > 0) {
        --clutch_warmup_remaining_;
        if (clutch_warmup_remaining_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Filter warmup complete. Streaming active.");
        }
        return;
    }

    if (streaming_started_) {
        fr5_pose_pub_->publish(target);
        legacy_pose_pub_->publish(target);
    }
}

void FairinoSlaveNode::startStreaming() {
    if (!stream_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "/enable_streaming service is not available yet");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    stream_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            const auto response = future.get();
            if (response->success) {
                streaming_started_ = true;
                RCLCPP_INFO(this->get_logger(), "FR5 pose streaming enabled");
            } else {
                streaming_started_ = false;
                RCLCPP_ERROR(this->get_logger(), "Failed to enable streaming: %s",
                             response->message.c_str());
            }
        });
}

void FairinoSlaveNode::stopStreaming() {
    if (!stream_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "/enable_streaming service is not available yet");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    stream_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            const auto response = future.get();
            if (response->success) {
                streaming_started_ = false;
                clutch_initialized_ = false;
                smoothing_initialized_ = false;
                butterworth_configured_ = false;
                RCLCPP_INFO(this->get_logger(), "FR5 pose streaming disabled");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to disable streaming: %s",
                             response->message.c_str());
            }
        });
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FairinoSlaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
