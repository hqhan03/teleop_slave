#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <deque>
#include <cmath>

#include "robot.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class FairinoControllerNode : public rclcpp::Node {
public:
    FairinoControllerNode();
    ~FairinoControllerNode();

    bool initialize();
    void shutdown();

private:
    bool connectRobot();
    void disconnectRobot();

    void trajectoryCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void executeTrajectoryService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void controlLoop();
    void streamLoop();
    void streamCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void streamService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    bool executeServoJ(const std::vector<double>& target_deg);
    void publishJointStates();

    // Parameters
    std::string robot_ip_;

    // Robot SDK
    std::unique_ptr<FRRobot> robot_;
    bool connected_{false};
    bool shutdown_done_{false};

    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr traj_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; // [추가] 실제 TCP 포즈 퍼블리셔
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_srv_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr stream_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stream_srv_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Trajectory data
    std::mutex traj_mutex_;
    std::deque<std::vector<double>> trajectory_queue_;
    bool trajectory_loaded_{false};
    std::atomic<bool> executing_{false};
    size_t current_traj_idx_{0};

    // Streaming data
    std::atomic<bool> stream_mode_{false};
    std::mutex stream_mutex_;
    std::vector<double> stream_target_deg_;

    // Dummy mode for visualization
    bool dummy_mode_{false};
    std::vector<double> dummy_joint_positions_;
    // ServoJ error tracking
    std::atomic<int> servo_error_count_{0};

    // Glitch filter cache
    JointPos last_valid_joints_{};
    bool has_valid_joints_{false};

    static constexpr int NUM_JOINTS = 6;
    const std::vector<std::string> joint_names_ = {
        "j1", "j2", "j3", "j4", "j5", "j6"
    };

    static constexpr double CONTROL_FREQUENCY_HZ = 250.0;
    static constexpr double CONTROL_PERIOD_MS = 4.0;
    static constexpr double RAD_TO_DEG = 180.0 / M_PI;
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
};
