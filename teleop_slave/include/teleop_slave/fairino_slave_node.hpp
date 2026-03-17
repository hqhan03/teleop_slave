#ifndef FAIRINO_SLAVE_NODE_HPP_
#define FAIRINO_SLAVE_NODE_HPP_

#include <array>
#include <atomic>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "teleop_slave/fr5_teleop_utils.hpp"

class FairinoSlaveNode : public rclcpp::Node {
public:
    FairinoSlaveNode();
    ~FairinoSlaveNode();

private:
    void manusPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void startStreaming();
    void stopStreaming();
    void keyboardThread();
    void loadCalibrationOverrides();
    void logCalibrationConfiguration() const;

    geometry_msgs::msg::PoseStamped buildTargetPose(
        const geometry_msgs::msg::PoseStamped& manus_pose) const;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manus_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_actual_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fr5_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr legacy_pose_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stream_client_;

    std::atomic<bool> streaming_started_{false};
    std::atomic<bool> running_{true};
    std::thread kb_thread_;

    bool offset_set_{false};
    bool robot_pose_received_{false};
    bool clutch_initialized_{false};
    geometry_msgs::msg::PoseStamped latest_robot_pose_;
    geometry_msgs::msg::Pose last_target_pose_;

    std::array<int, 3> tracker_to_robot_axes_{{0, 1, 2}};
    std::array<double, 3> tracker_to_robot_signs_{{1.0, 1.0, 1.0}};
    std::array<double, 3> tracker_to_robot_basis_rpy_deg_{{0.0, 0.0, 0.0}};
    tf2::Vector3 translation_scale_xyz_{1.0, 1.0, 1.0};
    tf2::Vector3 min_xyz_{-0.85, -0.85, 0.05};
    tf2::Vector3 max_xyz_{0.85, 0.85, 1.10};
    tf2::Quaternion tracker_to_robot_basis_{0.0, 0.0, 0.0, 1.0};
    teleop_slave::OrientationMode orientation_mode_{teleop_slave::OrientationMode::kPositionOnly};
    std::string calibration_file_checked_;
    std::string calibration_source_{"base_params"};
    bool calibration_override_loaded_{false};

    tf2::Vector3 clutch_tracker_position_;
    tf2::Vector3 base_robot_position_;
    tf2::Quaternion tracker_zero_orientation_{0.0, 0.0, 0.0, 1.0};
    tf2::Quaternion base_robot_orientation_{0.0, 0.0, 0.0, 1.0};
};

#endif  // FAIRINO_SLAVE_NODE_HPP_
