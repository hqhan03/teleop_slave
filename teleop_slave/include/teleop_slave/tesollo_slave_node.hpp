#ifndef TESOLLO_SLAVE_NODE_HPP_
#define TESOLLO_SLAVE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <array>
#include <vector>
#include <mutex>

static constexpr int NUM_JOINTS = 20;
static constexpr int NUM_CALIB_POSES = 5;

enum class CalibState { IDLE, WAITING_FOR_CAPTURE, AVERAGING, NEUTRAL_WAITING, NEUTRAL_AVERAGING };

class TesolloSlaveNode : public rclcpp::Node {
public:
    TesolloSlaveNode();

private:
    // ── Callbacks ──
    void fingerJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ── Single-pose calibration services ──
    void calibrateNeutralCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void calibrateNeutralCaptureCb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void finishNeutralAveraging();
    void calibrateResetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ── Multi-pose calibration services ──
    void calibMPStartCb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void calibMPCaptureCb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void calibMPResetCb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ── Multi-pose calibration helpers ──
    void initCalibrationPoses();
    void commandCalibPose(int pose_idx);
    void finishAveraging();
    void computeMultiPoseCalibration();
    double getEffectiveManusValue(int joint_idx, const double* manus_raw);

    // ── Offset persistence ──
    void saveOffsets(const std::string& filepath);
    bool loadOffsets(const std::string& filepath);
    std::string getDefaultCalibrationPath() const;

    // ── Full calibration persistence ──
    void saveFullCalibration();
    bool loadFullCalibration();
    std::string getFullCalibrationPath() const;

    // ── ROS interfaces ──
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr finger_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_neutral_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_neutral_capture_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_reset_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_mp_start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_mp_capture_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_mp_reset_srv_;

    // ── State ──
    bool dummy_mode_ = false;
    bool calibrated_ = false;

    // ── Per-joint mapping (loaded from ROS params) ──
    std::array<int, NUM_JOINTS>    manus_idx_;
    std::array<double, NUM_JOINTS> gain_;               // includes sign, from param
    std::array<double, NUM_JOINTS> offset_;             // computed from neutral capture
    std::array<double, NUM_JOINTS> motor_min_rad_;
    std::array<double, NUM_JOINTS> motor_max_rad_;
    std::array<double, NUM_JOINTS> neutral_tesollo_rad_;

    // ── Latest Manus reading cache (for calibration capture) ──
    std::mutex latest_manus_mutex_;
    std::array<double, NUM_JOINTS> latest_manus_;
    bool has_manus_data_ = false;

    // ── Multi-pose calibration state ──
    CalibState calib_state_ = CalibState::IDLE;
    int current_pose_idx_ = 0;
    bool calib_in_progress_ = false;
    std::array<std::array<double, NUM_JOINTS>, NUM_CALIB_POSES> calib_pose_targets_;   // known Tesollo angles (rad)
    std::array<std::array<double, NUM_JOINTS>, NUM_CALIB_POSES> calib_manus_captures_; // captured Manus readings
    std::vector<std::array<double, NUM_JOINTS>> calib_avg_buffer_;
    rclcpp::TimerBase::SharedPtr calib_avg_timer_;

    // ── Calibration persistence ──
    std::string calibration_file_path_;
};

#endif // TESOLLO_SLAVE_NODE_HPP_
