#pragma once

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

#include "teleop_slave/fairino_robot_interface.hpp"

class FairinoControllerNode : public rclcpp::Node {
public:
    FairinoControllerNode();
    ~FairinoControllerNode();

    bool initialize();
    void shutdown();

private:
    bool connectRobot();
    void disconnectRobot();
    bool validateControllerState();

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
    void publishJointStates();

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

    std::mutex pose_target_mutex_;
    geometry_msgs::msg::PoseStamped latest_pose_target_;
    rclcpp::Time latest_pose_target_arrival_{0, 0, RCL_SYSTEM_TIME};
    bool pose_target_received_{false};
    bool pose_target_stream_active_{false};
    int consecutive_ik_failures_{0};
    rclcpp::Time last_successful_ik_time_{0, 0, RCL_SYSTEM_TIME};

    bool dummy_mode_{false};
    std::vector<double> dummy_joint_positions_;
    geometry_msgs::msg::PoseStamped dummy_robot_pose_;
    std::atomic<int> servo_error_count_{0};
    int consecutive_servoj_failures_{0};
    double servo_cmd_period_sec_{0.008};
    double servoj_max_step_deg_{2.0};
    int servoj_failure_limit_{3};

    JointPos last_valid_joints_{};
    bool has_valid_joints_{false};

    static constexpr int NUM_JOINTS = 6;
    const std::vector<std::string> joint_names_ = {
        "j1", "j2", "j3", "j4", "j5", "j6"
    };

    static constexpr double RAD_TO_DEG = 180.0 / M_PI;
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
};
