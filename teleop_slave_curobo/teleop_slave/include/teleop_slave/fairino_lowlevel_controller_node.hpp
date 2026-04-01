#pragma once

#include <array>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "teleop_slave/cartesian_pose_stream_interpolator.hpp"
#include "teleop_slave/fairino_robot_interface.hpp"
#include "teleop_slave/joint_target_smoother.hpp"

class FairinoControllerNode : public rclcpp::Node {
public:
    FairinoControllerNode();
    ~FairinoControllerNode();

    bool initialize();
    void shutdown();

private:
    using JointArray = teleop_slave::JointTargetSmoother::JointArray;
    enum class StreamCommandMode {
        kServoJ,
        kServoCart,
    };

    bool connectRobot();
    void disconnectRobot();
    bool validateControllerState();
    bool startServoMotionSession(const char* context, std::string* failure_message = nullptr);
    void stopServoMotionSession(const char* context);

    void trajectoryCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void executeTrajectoryService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void controlLoop();
    void streamLoop();
    void streamCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void streamService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    bool executeServoJ(const std::vector<double>& target_deg);
    bool executeServoCart(const geometry_msgs::msg::Pose& target_pose);
    void publishJointStates();
    void logJointSmoothingDiagnostics(const JointArray& raw_target_deg,
                                      const JointArray& filtered_target_deg);
    void logServoCartTrackingDiagnostics(const geometry_msgs::msg::Pose& target_pose);

    std::string robot_ip_;
    std::unique_ptr<IFairinoRobot> robot_;
    bool connected_{false};
    bool shutdown_done_{false};

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr traj_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr legacy_pose_target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_srv_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr stream_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stream_srv_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::mutex traj_mutex_;
    std::deque<std::vector<double>> trajectory_queue_;
    bool trajectory_loaded_{false};
    std::atomic<bool> executing_{false};
    size_t current_traj_idx_{0};

    std::atomic<bool> stream_mode_{false};
    std::mutex stream_mutex_;
    std::vector<double> stream_target_deg_;
    std::vector<double> stream_prev_deg_;
    std::vector<double> stream_prev_prev_deg_;
    std::chrono::steady_clock::time_point stream_prev_time_;
    std::chrono::steady_clock::time_point stream_prev_prev_time_;
    std::chrono::steady_clock::time_point stream_target_time_;
    bool stream_has_prev_{false};
    bool stream_has_prev_prev_{false};
    teleop_slave::CartesianPoseStreamInterpolator cartesian_stream_interpolator_;
    bool cartesian_stream_has_tracker_target_{false};
    geometry_msgs::msg::Pose cartesian_previous_command_pose_{};
    bool cartesian_previous_command_pose_valid_{false};
    teleop_slave::JointTargetSmoother joint_target_smoother_;
    JointArray joint_deadband_deg_{};
    JointArray joint_velocity_limit_deg_s_{};
    std::chrono::steady_clock::time_point last_joint_filter_update_time_{};
    bool joint_filter_time_initialized_{false};
    JointArray raw_prev_ik_target_deg_{};
    bool raw_has_prev_ik_target_{false};
    JointArray raw_prev_ik_step_deg_{};
    bool raw_has_prev_ik_step_{false};

    std::mutex pose_target_mutex_;
    geometry_msgs::msg::PoseStamped latest_pose_target_;
    rclcpp::Time latest_pose_target_arrival_{0, 0, RCL_SYSTEM_TIME};
    bool pose_target_received_{false};
    bool pose_target_stream_active_{false};
    std::mutex actual_pose_mutex_;
    geometry_msgs::msg::Pose latest_actual_pose_;
    bool actual_pose_received_{false};
    int consecutive_ik_failures_{0};
    rclcpp::Time last_successful_ik_time_{0, 0, RCL_SYSTEM_TIME};

    bool dummy_mode_{false};
    std::vector<double> dummy_joint_positions_;
    geometry_msgs::msg::PoseStamped dummy_robot_pose_;
    StreamCommandMode stream_command_mode_{StreamCommandMode::kServoCart};
    std::atomic<int> servo_error_count_{0};
    int consecutive_servoj_failures_{0};
    int global_speed_percent_{30};
    double servo_cmd_period_sec_{0.008};
    double pose_target_nominal_period_sec_{0.02};
    double servocart_filter_t_{0.0};
    double servocart_gain_{0.0};
    double servoj_max_step_deg_{0.8};
    double servoj_filter_t_{0.0};
    double servoj_gain_{0.0};
    int servoj_failure_limit_{3};
    std::mutex servo_motion_session_mutex_;
    bool servo_motion_session_active_{false};
    std::atomic<bool> servo_cart_first_command_pending_{false};

    JointPos last_valid_joints_{};
    bool has_valid_joints_{false};

    JointPos prev_servoj_cmd_{};
    bool has_prev_servoj_cmd_{false};

    static constexpr int NUM_JOINTS = 6;
    const std::vector<std::string> joint_names_ = {
        "j1", "j2", "j3", "j4", "j5", "j6"
    };

    static constexpr double RAD_TO_DEG = 180.0 / M_PI;
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
};
