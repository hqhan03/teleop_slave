#include "teleop_slave/tesollo_slave_node.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <sys/stat.h>

static constexpr double D2R = M_PI / 180.0;

// Pose names for logging
static const char* poseNames[NUM_CALIB_POSES] = {
    "FLAT OPEN — extend all fingers, thumb spread wide",
    "RELAXED FIST — curl fingers moderately, thumb resting across index",
    "PINCH — touch thumb+index tips together, others extended and spread",
    "POINTING — extend index only, curl others, thumb tucked beside index",
    "ROCK ON — index+pinky extended, middle+ring curled, thumb mid"
};

// Joint type labels for logging
static const char* motorLabel(int motor_idx) {
    static const char* labels[NUM_JOINTS] = {
        "Thumb MCPSpread",  "Thumb Opposition",  "Thumb PIP",        "Thumb DIP",
        "Index MCP Ab/Ad",  "Index MCP Fl/Ex",  "Index PIP Fl/Ex",  "Index DIP Fl/Ex",
        "Middle MCP Ab/Ad", "Middle MCP Fl/Ex", "Middle PIP Fl/Ex", "Middle DIP Fl/Ex",
        "Ring MCP Ab/Ad",   "Ring MCP Fl/Ex",   "Ring PIP Fl/Ex",   "Ring DIP Fl/Ex",
        "Pinky CMC Ab/Ad",  "Pinky MCP Ab/Ad",  "Pinky PIP Fl/Ex",  "Pinky DIP(PIP+DIP)"
    };
    if (motor_idx >= 0 && motor_idx < NUM_JOINTS) return labels[motor_idx];
    return "Unknown";
}

TesolloSlaveNode::TesolloSlaveNode() : Node("tesollo_slave_node") {
    this->declare_parameter<bool>("dummy_mode", false);
    this->get_parameter("dummy_mode", dummy_mode_);

    // ── Calibration file path ──
    this->declare_parameter<std::string>("calibration_file", "");
    this->get_parameter("calibration_file", calibration_file_path_);
    if (calibration_file_path_.empty()) {
        calibration_file_path_ = getDefaultCalibrationPath();
    }

    // ── Per-joint mapping parameters ──
    std::vector<long int> default_manus_idx = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1, 16, 17, 18};
    std::vector<double> default_gain = {-6.0, 6.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 0.0, -1.0, 1.0, 1.0};
    std::vector<double> default_neutral = {77.0, -80.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> default_min = {-22, -155, -90, -90, -31, 0, -90, -90, -30, 0, -90, -90, -15, 0, -90, -90, 0, -15, -90, -90};
    std::vector<double> default_max = {77, 0, 90, 90, 20, 115, 90, 90, 30, 115, 90, 90, 32, 110, 90, 90, 60, 90, 90, 90};

    this->declare_parameter<std::vector<long int>>("manus_idx", default_manus_idx);
    this->declare_parameter<std::vector<double>>("gain", default_gain);
    this->declare_parameter<std::vector<double>>("neutral_tesollo_deg", default_neutral);
    this->declare_parameter<std::vector<double>>("motor_min_deg", default_min);
    this->declare_parameter<std::vector<double>>("motor_max_deg", default_max);

    auto manus_idx_vec = this->get_parameter("manus_idx").as_integer_array();
    auto gain_vec = this->get_parameter("gain").as_double_array();
    auto neutral_vec = this->get_parameter("neutral_tesollo_deg").as_double_array();
    auto min_vec = this->get_parameter("motor_min_deg").as_double_array();
    auto max_vec = this->get_parameter("motor_max_deg").as_double_array();

    // Validate sizes
    if (static_cast<int>(manus_idx_vec.size()) != NUM_JOINTS ||
        static_cast<int>(gain_vec.size()) != NUM_JOINTS ||
        static_cast<int>(neutral_vec.size()) != NUM_JOINTS ||
        static_cast<int>(min_vec.size()) != NUM_JOINTS ||
        static_cast<int>(max_vec.size()) != NUM_JOINTS) {
        RCLCPP_FATAL(this->get_logger(),
            "All mapping parameters must have exactly %d elements. Check your params file.", NUM_JOINTS);
        throw std::runtime_error("Invalid parameter array sizes");
    }

    for (int i = 0; i < NUM_JOINTS; ++i) {
        manus_idx_[i] = static_cast<int>(manus_idx_vec[i]);
        gain_[i] = gain_vec[i];
        neutral_tesollo_rad_[i] = neutral_vec[i] * D2R;
        motor_min_rad_[i] = min_vec[i] * D2R;
        motor_max_rad_[i] = max_vec[i] * D2R;
    }

    // Initialize offsets to zero (uncalibrated)
    offset_.fill(0.0);
    latest_manus_.fill(0.0);

    // Initialize multi-pose calibration poses
    initCalibrationPoses();

    // ── ROS2 interfaces ──
    finger_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/manus/finger_joints", 10,
        std::bind(&TesolloSlaveNode::fingerJointsCallback, this, std::placeholders::_1));

    std::string traj_topic = dummy_mode_ ?
        "/joint_trajectory_controller/joint_trajectory" :
        "/dg5f_right/dg5f_right_controller/joint_trajectory";
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic, 10);

    // ── Single-pose calibration services ──
    calib_neutral_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/calibrate_neutral",
        std::bind(&TesolloSlaveNode::calibrateNeutralCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    calib_neutral_capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/calibrate_neutral_capture",
        std::bind(&TesolloSlaveNode::calibrateNeutralCaptureCb, this,
                  std::placeholders::_1, std::placeholders::_2));

    calib_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/calibrate_reset",
        std::bind(&TesolloSlaveNode::calibrateResetCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // ── Multi-pose calibration services ──
    calib_mp_start_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/calibrate_multipose_start",
        std::bind(&TesolloSlaveNode::calibMPStartCb, this,
                  std::placeholders::_1, std::placeholders::_2));

    calib_mp_capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/calibrate_multipose_capture",
        std::bind(&TesolloSlaveNode::calibMPCaptureCb, this,
                  std::placeholders::_1, std::placeholders::_2));

    calib_mp_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/calibrate_multipose_reset",
        std::bind(&TesolloSlaveNode::calibMPResetCb, this,
                  std::placeholders::_1, std::placeholders::_2));

    if (dummy_mode_) {
        RCLCPP_INFO(this->get_logger(), "Tesollo Slave Node Started in DUMMY_MODE");
    } else {
        RCLCPP_INFO(this->get_logger(), "Tesollo Slave Node Started. Ensure dg5f_driver is running!");
    }

    // ── Auto-load saved calibration (full first, then offsets-only fallback) ──
    if (loadFullCalibration()) {
        RCLCPP_INFO(this->get_logger(), "Loaded full calibration (gains+offsets) from: %s",
                    getFullCalibrationPath().c_str());
    } else if (loadOffsets(calibration_file_path_)) {
        RCLCPP_INFO(this->get_logger(), "Loaded saved offsets from: %s", calibration_file_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "No saved calibration found. Running with default gains and zero offsets.");
    }

    // ── Log mapping configuration ──
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "=== JOINT MAPPING ===");
    for (int i = 0; i < NUM_JOINTS; ++i) {
        RCLCPP_INFO(this->get_logger(), "  Motor %2d %-22s manus[%2d] gain=%+.2f offset=%+.1f deg  [%+.0f, %+.0f] deg",
                    i + 1, motorLabel(i), manus_idx_[i], gain_[i], offset_[i] / D2R,
                    motor_min_rad_[i] / D2R, motor_max_rad_[i] / D2R);
    }
    RCLCPP_INFO(this->get_logger(), "=====================");

    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "=== CALIBRATION ===");
    RCLCPP_INFO(this->get_logger(), "  Single-pose (offsets only, two steps):");
    RCLCPP_INFO(this->get_logger(), "    1) ros2 service call /calibrate_neutral std_srvs/srv/Trigger \"{}\"");
    RCLCPP_INFO(this->get_logger(), "       -> Tesollo moves to neutral pose, match it with your hand");
    RCLCPP_INFO(this->get_logger(), "    2) ros2 service call /calibrate_neutral_capture std_srvs/srv/Trigger \"{}\"");
    RCLCPP_INFO(this->get_logger(), "       -> 1-sec average capture, offsets computed and saved");
    RCLCPP_INFO(this->get_logger(), "  Multi-pose (gains + offsets, 5 poses):");
    RCLCPP_INFO(this->get_logger(), "    ros2 service call /calibrate_multipose_start std_srvs/srv/Trigger \"{}\"");
    RCLCPP_INFO(this->get_logger(), "  Reset offsets:");
    RCLCPP_INFO(this->get_logger(), "    ros2 service call /calibrate_reset std_srvs/srv/Trigger \"{}\"");
    RCLCPP_INFO(this->get_logger(), "===================");
}

// =============================================================================
// Initialize the 5 calibration target poses (Tesollo angles in radians)
// =============================================================================
void TesolloSlaveNode::initCalibrationPoses() {
    // Pose targets in degrees — converted to radians below
    // Each row: 20 motor targets matching motor indices 0..19
    const double poses_deg[NUM_CALIB_POSES][NUM_JOINTS] = {
        // Pose 1: "Flat Open" — all fingers extended, thumb spread wide
        //         Thmb:Spr Opp PIP DIP  Idx:Ab Fl  PIP DIP  Mid:Ab Fl  PIP DIP  Rng:Ab Fl  PIP DIP  Pnk:CMC Ab PIP DIP
        {  77,    0,   0,   0,   20,   0,   0,   0,    0,   0,   0,   0,  -15,   0,   0,   0,   0,  -15,   0,   0},
        // Pose 2: "Relaxed Fist" — fingers moderately curled, thumb wrapped and tucked
        {  40, -90,   0,   0,    0,  80,  60,  60,    0,  80,  60,  60,    0,  75,  60,  60,   0,    0,  60,  60},
        // Pose 3: "Pinch" — thumb+index tips touching, others extended and spread
        {  40,  -70, -20, -20,    0,  40,  30,  30,   15,   0,   0,   0,   15,   0,   0,   0,   0,   40,   0,   0},
        // Pose 4: "Pointing" — index extended, others curled, thumb relaxed alongside
        {  60,  -30,   0,   0,  -10,   0,   0,   0,  -10,  70,  50,  50,   -5,  70,  50,  50,   0,   -5,  50,  50},
        // Pose 5: "Rock On" — index+pinky extended, middle+ring curled, thumb tucked
        {  40,  -90,   0,   0,   10,   0,   0,   0,    0, 100,  70,  70,   15, 100,  70,  70,   0,   50,   0,   0}
    };

    for (int p = 0; p < NUM_CALIB_POSES; ++p) {
        for (int j = 0; j < NUM_JOINTS; ++j) {
            calib_pose_targets_[p][j] = poses_deg[p][j] * D2R;
        }
    }
}

// =============================================================================
// Helper: get effective Manus value for a joint (handles combined-input joints)
// =============================================================================
double TesolloSlaveNode::getEffectiveManusValue(int joint_idx, const double* manus_raw) {
    int m_idx = manus_idx_[joint_idx];
    if (m_idx < 0 || gain_[joint_idx] == 0.0) {
        return 0.0;
    }

    // Motor 19 (Pinky DIP): combine PIP[18] + DIP[19]
    if (joint_idx == 19) {
        return manus_raw[18] + manus_raw[19];
    }

    return manus_raw[m_idx];
}

// =============================================================================
// Main control callback — maps Manus joint angles to Tesollo motor commands
// =============================================================================
void TesolloSlaveNode::fingerJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < 20) return;

    // Cache latest reading for calibration capture
    {
        std::lock_guard<std::mutex> lock(latest_manus_mutex_);
        for (int i = 0; i < NUM_JOINTS; ++i) {
            latest_manus_[i] = msg->position[i];
        }
        has_manus_data_ = true;
    }

    // During calibration, collect averaging samples but don't teleop
    if (calib_in_progress_) {
        if (calib_state_ == CalibState::AVERAGING || calib_state_ == CalibState::NEUTRAL_AVERAGING) {
            std::array<double, NUM_JOINTS> sample;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                sample[i] = getEffectiveManusValue(i, msg->position.data());
            }
            calib_avg_buffer_.push_back(sample);
        }
        return;  // Don't publish teleop commands during calibration
    }

    std::vector<double> delto_target(NUM_JOINTS, 0.0);

    for (int i = 0; i < NUM_JOINTS; ++i) {
        int m_idx = manus_idx_[i];

        // Unmapped motor (e.g., Motor 17)
        if (m_idx < 0 || gain_[i] == 0.0) {
            delto_target[i] = 0.0;
            continue;
        }

        double manus_val = getEffectiveManusValue(i, msg->position.data());
        double target = gain_[i] * manus_val + offset_[i];
        delto_target[i] = std::clamp(target, motor_min_rad_[i], motor_max_rad_[i]);
    }

    // ── Build and publish JointTrajectory ──
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.joint_names = DG5F_JOINT_NAMES;

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = delto_target;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 100000000;  // 0.1s

    traj_msg.points.push_back(point);
    traj_pub_->publish(traj_msg);
}

// =============================================================================
// Calibration: Step 1 — Command neutral pose and wait for user to match it
// =============================================================================
void TesolloSlaveNode::calibrateNeutralCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (calib_in_progress_) {
        res->success = false;
        res->message = "Calibration already in progress. Call /calibrate_multipose_reset or wait for completion.";
        return;
    }

    calib_in_progress_ = true;
    calib_state_ = CalibState::NEUTRAL_WAITING;

    // Command the Tesollo to the neutral pose
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.joint_names = DG5F_JOINT_NAMES;
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions.assign(neutral_tesollo_rad_.begin(), neutral_tesollo_rad_.end());
    point.time_from_start.sec = 2;
    point.time_from_start.nanosec = 0;
    traj_msg.points.push_back(point);
    traj_pub_->publish(traj_msg);

    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Tesollo moved to neutral pose. Match it with your gloved hand, then call:");
    RCLCPP_INFO(this->get_logger(), "  ros2 service call /calibrate_neutral_capture std_srvs/srv/Trigger \"{}\"");
    RCLCPP_INFO(this->get_logger(), "========================================");

    res->success = true;
    res->message = "Tesollo moved to neutral pose. Match it with your hand, then call /calibrate_neutral_capture";
}

// =============================================================================
// Calibration: Step 2 — Capture 1-second average and compute offsets
// =============================================================================
void TesolloSlaveNode::calibrateNeutralCaptureCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (calib_state_ != CalibState::NEUTRAL_WAITING) {
        res->success = false;
        res->message = "Not ready for neutral capture. Call /calibrate_neutral first.";
        return;
    }

    if (!has_manus_data_) {
        res->success = false;
        res->message = "No Manus data received yet. Is master_bridge_node running?";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Capturing neutral pose — averaging for 1 second...");

    calib_avg_buffer_.clear();
    calib_avg_buffer_.reserve(60);
    calib_state_ = CalibState::NEUTRAL_AVERAGING;

    calib_avg_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            calib_avg_timer_->cancel();
            finishNeutralAveraging();
        });

    res->success = true;
    res->message = "Capturing neutral pose — hold still for 1 second...";
}

// =============================================================================
// Calibration: Finish neutral averaging and compute offsets
// =============================================================================
void TesolloSlaveNode::finishNeutralAveraging() {
    if (calib_avg_buffer_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[CALIBRATION] No samples collected! Is Manus data streaming?");
        calib_state_ = CalibState::NEUTRAL_WAITING;
        return;
    }

    // Compute mean per joint
    std::array<double, NUM_JOINTS> mean;
    mean.fill(0.0);
    for (const auto& sample : calib_avg_buffer_) {
        for (int j = 0; j < NUM_JOINTS; ++j) {
            mean[j] += sample[j];
        }
    }
    double n = static_cast<double>(calib_avg_buffer_.size());
    for (int j = 0; j < NUM_JOINTS; ++j) {
        mean[j] /= n;
    }

    // Compute offsets: offset = neutral_target - gain * manus_mean
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (manus_idx_[i] < 0 || gain_[i] == 0.0) {
            offset_[i] = 0.0;
            continue;
        }
        offset_[i] = neutral_tesollo_rad_[i] - gain_[i] * mean[i];
    }

    calibrated_ = true;

    // Log results
    RCLCPP_INFO(this->get_logger(), "=== NEUTRAL CALIBRATION COMPLETE (%zu samples averaged) ===", calib_avg_buffer_.size());
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (manus_idx_[i] < 0) continue;
        RCLCPP_INFO(this->get_logger(), "  Motor %2d %-22s manus=%+6.1f deg -> offset=%+6.1f deg",
                    i + 1, motorLabel(i), mean[i] / D2R, offset_[i] / D2R);
    }
    RCLCPP_INFO(this->get_logger(), "====================================");

    // Auto-save
    try {
        saveOffsets(calibration_file_path_);
        RCLCPP_INFO(this->get_logger(), "Offsets saved to: %s", calibration_file_path_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to save offsets: %s", e.what());
    }

    calib_in_progress_ = false;
    calib_state_ = CalibState::IDLE;
    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Neutral calibration complete. Teleoperation resumed.");
}

// =============================================================================
// Calibration: Reset offsets to zero
// =============================================================================
void TesolloSlaveNode::calibrateResetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    offset_.fill(0.0);
    calibrated_ = false;

    RCLCPP_INFO(this->get_logger(), "Offsets reset to zero.");
    res->success = true;
    res->message = "Offsets reset to zero. Call /calibrate_neutral to recalibrate.";
}

// =============================================================================
// Multi-pose calibration: Start
// =============================================================================
void TesolloSlaveNode::calibMPStartCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (calib_in_progress_) {
        res->success = false;
        res->message = "Multi-pose calibration already in progress. Call /calibrate_multipose_reset to abort.";
        return;
    }

    calib_in_progress_ = true;
    current_pose_idx_ = 0;
    calib_state_ = CalibState::WAITING_FOR_CAPTURE;

    commandCalibPose(0);

    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Multi-pose calibration started (5 poses)");
    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Pose 1/%d: %s", NUM_CALIB_POSES, poseNames[0]);
    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Match this pose with your gloved hand, then call:");
    RCLCPP_INFO(this->get_logger(), "  ros2 service call /calibrate_multipose_capture std_srvs/srv/Trigger \"{}\"");
    RCLCPP_INFO(this->get_logger(), "========================================");

    res->success = true;
    res->message = "Multi-pose calibration started. Pose 1/5: " + std::string(poseNames[0]) +
                   ". Match pose, then call /calibrate_multipose_capture";
}

// =============================================================================
// Multi-pose calibration: Capture current pose
// =============================================================================
void TesolloSlaveNode::calibMPCaptureCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!calib_in_progress_ || calib_state_ != CalibState::WAITING_FOR_CAPTURE) {
        res->success = false;
        res->message = "Not ready for capture. Call /calibrate_multipose_start first.";
        return;
    }

    if (!has_manus_data_) {
        res->success = false;
        res->message = "No Manus data received yet. Is master_bridge_node running?";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Capturing pose %d/%d — averaging for 1 second...",
                current_pose_idx_ + 1, NUM_CALIB_POSES);

    // Begin 1-second averaging
    calib_avg_buffer_.clear();
    calib_avg_buffer_.reserve(60);  // ~50Hz expected
    calib_state_ = CalibState::AVERAGING;

    // One-shot timer: 1 second
    calib_avg_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            calib_avg_timer_->cancel();
            finishAveraging();
        });

    res->success = true;
    res->message = "Capturing pose " + std::to_string(current_pose_idx_ + 1) + "/" +
                   std::to_string(NUM_CALIB_POSES) + " — hold still for 1 second...";
}

// =============================================================================
// Multi-pose calibration: Finish averaging and advance to next pose
// =============================================================================
void TesolloSlaveNode::finishAveraging() {
    if (calib_avg_buffer_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[CALIBRATION] No samples collected! Is Manus data streaming?");
        calib_state_ = CalibState::WAITING_FOR_CAPTURE;
        return;
    }

    // Compute mean per joint
    std::array<double, NUM_JOINTS> mean;
    mean.fill(0.0);
    for (const auto& sample : calib_avg_buffer_) {
        for (int j = 0; j < NUM_JOINTS; ++j) {
            mean[j] += sample[j];
        }
    }
    double n = static_cast<double>(calib_avg_buffer_.size());
    for (int j = 0; j < NUM_JOINTS; ++j) {
        mean[j] /= n;
    }

    calib_manus_captures_[current_pose_idx_] = mean;

    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Pose %d/%d captured (%zu samples averaged)",
                current_pose_idx_ + 1, NUM_CALIB_POSES, calib_avg_buffer_.size());

    current_pose_idx_++;

    if (current_pose_idx_ < NUM_CALIB_POSES) {
        // Move to next pose
        calib_state_ = CalibState::WAITING_FOR_CAPTURE;
        commandCalibPose(current_pose_idx_);

        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Pose %d/%d: %s",
                    current_pose_idx_ + 1, NUM_CALIB_POSES, poseNames[current_pose_idx_]);
        RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Match this pose, then call:");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /calibrate_multipose_capture std_srvs/srv/Trigger \"{}\"");
    } else {
        // All poses captured — compute calibration
        computeMultiPoseCalibration();
        calib_in_progress_ = false;
        calib_state_ = CalibState::IDLE;
    }
}

// =============================================================================
// Multi-pose calibration: Compute least-squares gain and offset per joint
// =============================================================================
void TesolloSlaveNode::computeMultiPoseCalibration() {
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Computing least-squares fit per joint...");

    std::array<double, NUM_JOINTS> r_squared;
    r_squared.fill(0.0);
    int accepted = 0, rejected = 0, warned = 0;

    for (int j = 0; j < NUM_JOINTS; ++j) {
        int m_idx = manus_idx_[j];
        // Skip unmapped motors
        if (m_idx < 0 || (j == 16)) {
            RCLCPP_INFO(this->get_logger(), "  Motor %2d %-22s SKIPPED (unmapped)", j + 1, motorLabel(j));
            continue;
        }

        // x = Manus values, y = Tesollo target values
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        for (int p = 0; p < NUM_CALIB_POSES; ++p) {
            double x = calib_manus_captures_[p][j];
            double y = calib_pose_targets_[p][j];
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }

        double N = static_cast<double>(NUM_CALIB_POSES);
        double denom = N * sum_x2 - sum_x * sum_x;

        double new_gain, new_offset;

        if (std::abs(denom) < 1e-10) {
            // Manus didn't vary for this joint — keep existing gain, compute offset from mean
            RCLCPP_WARN(this->get_logger(),
                "  Motor %2d %-22s Manus didn't vary — keeping gain=%.2f, computing offset from mean",
                j + 1, motorLabel(j), gain_[j]);
            new_gain = gain_[j];
            new_offset = (sum_y / N) - new_gain * (sum_x / N);
            r_squared[j] = 0.0;
            gain_[j] = new_gain;
            offset_[j] = new_offset;
            warned++;
            continue;
        }

        new_gain = (N * sum_xy - sum_x * sum_y) / denom;
        new_offset = (sum_y - new_gain * sum_x) / N;

        // Compute R²
        double y_mean = sum_y / N;
        double ss_tot = 0.0, ss_res = 0.0;
        for (int p = 0; p < NUM_CALIB_POSES; ++p) {
            double x = calib_manus_captures_[p][j];
            double y = calib_pose_targets_[p][j];
            double y_pred = new_gain * x + new_offset;
            ss_res += (y - y_pred) * (y - y_pred);
            ss_tot += (y - y_mean) * (y - y_mean);
        }

        double r2 = (ss_tot > 1e-10) ? (1.0 - ss_res / ss_tot) : 0.0;
        r_squared[j] = r2;

        if (r2 < 0.70) {
            RCLCPP_ERROR(this->get_logger(),
                "  Motor %2d %-22s R²=%.3f < 0.70 — REJECTED (keeping gain=%.2f, offset=%.1f deg)",
                j + 1, motorLabel(j), r2, gain_[j], offset_[j] / D2R);
            rejected++;
        } else if (r2 < 0.90) {
            RCLCPP_WARN(this->get_logger(),
                "  Motor %2d %-22s R²=%.3f < 0.90 — accepted with warning: gain=%.3f, offset=%.1f deg",
                j + 1, motorLabel(j), r2, new_gain, new_offset / D2R);
            gain_[j] = new_gain;
            offset_[j] = new_offset;
            warned++;
            accepted++;
        } else {
            RCLCPP_INFO(this->get_logger(),
                "  Motor %2d %-22s R²=%.3f — gain=%.3f, offset=%.1f deg",
                j + 1, motorLabel(j), r2, new_gain, new_offset / D2R);
            gain_[j] = new_gain;
            offset_[j] = new_offset;
            accepted++;
        }
    }

    calibrated_ = true;

    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Results: %d accepted, %d rejected, %d warnings",
                accepted, rejected, warned);

    // Save full calibration
    try {
        saveFullCalibration();
        RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Full calibration saved to: %s",
                    getFullCalibrationPath().c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[CALIBRATION] Failed to save calibration: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Multi-pose calibration complete. Normal teleoperation resumed.");
    RCLCPP_INFO(this->get_logger(), "========================================");
}

// =============================================================================
// Multi-pose calibration: Reset/abort
// =============================================================================
void TesolloSlaveNode::calibMPResetCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (calib_avg_timer_) {
        calib_avg_timer_->cancel();
        calib_avg_timer_.reset();
    }

    bool was_neutral = (calib_state_ == CalibState::NEUTRAL_WAITING ||
                        calib_state_ == CalibState::NEUTRAL_AVERAGING);

    calib_in_progress_ = false;
    calib_state_ = CalibState::IDLE;
    calib_avg_buffer_.clear();

    if (was_neutral) {
        RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Neutral calibration aborted. Normal teleoperation resumed.");
        res->message = "Neutral calibration aborted. Use /calibrate_reset to also reset offsets.";
    } else {
        RCLCPP_INFO(this->get_logger(), "[CALIBRATION] Multi-pose calibration aborted. Normal teleoperation resumed.");
        res->message = "Multi-pose calibration aborted. Use /calibrate_reset to also reset gains/offsets.";
    }

    res->success = true;
}

// =============================================================================
// Command the Tesollo to a calibration pose
// =============================================================================
void TesolloSlaveNode::commandCalibPose(int pose_idx) {
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.joint_names = DG5F_JOINT_NAMES;

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions.assign(calib_pose_targets_[pose_idx].begin(),
                           calib_pose_targets_[pose_idx].end());
    point.time_from_start.sec = 2;
    point.time_from_start.nanosec = 0;

    traj_msg.points.push_back(point);
    traj_pub_->publish(traj_msg);
}

// =============================================================================
// Save full calibration (gains + offsets + R²) to YAML
// =============================================================================
void TesolloSlaveNode::saveFullCalibration() {
    std::string filepath = getFullCalibrationPath();

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "calibration_version" << YAML::Value << 2;
    out << YAML::Key << "calibrated" << YAML::Value << true;
    out << YAML::Key << "gains" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < NUM_JOINTS; ++i) out << gain_[i];
    out << YAML::EndSeq;
    out << YAML::Key << "offsets_rad" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < NUM_JOINTS; ++i) out << offset_[i];
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath);
    }
    fout << out.c_str();
    fout.close();
}

// =============================================================================
// Load full calibration from YAML
// =============================================================================
bool TesolloSlaveNode::loadFullCalibration() {
    std::string filepath = getFullCalibrationPath();

    struct stat buffer;
    if (stat(filepath.c_str(), &buffer) != 0) {
        return false;
    }

    try {
        YAML::Node root = YAML::LoadFile(filepath);

        if (!root["calibration_version"] || root["calibration_version"].as<int>() != 2) {
            return false;
        }

        if (!root["calibrated"] || !root["calibrated"].as<bool>()) {
            return false;
        }

        if (!root["gains"] || root["gains"].size() != NUM_JOINTS ||
            !root["offsets_rad"] || root["offsets_rad"].size() != NUM_JOINTS) {
            RCLCPP_WARN(this->get_logger(), "Invalid full calibration file: %s", filepath.c_str());
            return false;
        }

        for (int i = 0; i < NUM_JOINTS; ++i) {
            gain_[i] = root["gains"][i].as<double>();
            offset_[i] = root["offsets_rad"][i].as<double>();
        }
        calibrated_ = true;

        RCLCPP_INFO(this->get_logger(), "Loaded full calibration:");
        for (int i = 0; i < NUM_JOINTS; ++i) {
            if (manus_idx_[i] < 0) continue;
            RCLCPP_INFO(this->get_logger(), "  Motor %2d %-22s gain=%+.3f offset=%+.1f deg",
                        i + 1, motorLabel(i), gain_[i], offset_[i] / D2R);
        }
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse full calibration file %s: %s",
                    filepath.c_str(), e.what());
        return false;
    }
}

// =============================================================================
// Full calibration file path
// =============================================================================
std::string TesolloSlaveNode::getFullCalibrationPath() const {
    const char* home = std::getenv("HOME");
    if (home) {
        std::string ros_dir = std::string(home) + "/.ros";
        mkdir(ros_dir.c_str(), 0755);
        return ros_dir + "/tesollo_calibration.yaml";
    }
    return "/tmp/tesollo_calibration.yaml";
}

// =============================================================================
// Save offsets to YAML file
// =============================================================================
void TesolloSlaveNode::saveOffsets(const std::string& filepath) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "calibrated" << YAML::Value << calibrated_;
    out << YAML::Key << "offsets_rad" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < NUM_JOINTS; ++i) out << offset_[i];
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath);
    }
    fout << out.c_str();
    fout.close();
}

// =============================================================================
// Load offsets from YAML file
// =============================================================================
bool TesolloSlaveNode::loadOffsets(const std::string& filepath) {
    struct stat buffer;
    if (stat(filepath.c_str(), &buffer) != 0) {
        return false;
    }

    try {
        YAML::Node root = YAML::LoadFile(filepath);

        if (!root["calibrated"] || !root["calibrated"].as<bool>()) {
            return false;
        }

        if (!root["offsets_rad"] || root["offsets_rad"].size() != NUM_JOINTS) {
            RCLCPP_WARN(this->get_logger(), "Invalid offsets file: %s", filepath.c_str());
            return false;
        }

        for (int i = 0; i < NUM_JOINTS; ++i) {
            offset_[i] = root["offsets_rad"][i].as<double>();
        }
        calibrated_ = true;

        RCLCPP_INFO(this->get_logger(), "Loaded offsets:");
        for (int i = 0; i < NUM_JOINTS; ++i) {
            if (manus_idx_[i] < 0) continue;
            RCLCPP_INFO(this->get_logger(), "  Motor %2d %-22s offset=%+.1f deg",
                        i + 1, motorLabel(i), offset_[i] / D2R);
        }
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse offsets file %s: %s", filepath.c_str(), e.what());
        return false;
    }
}

// =============================================================================
// Default calibration file path
// =============================================================================
std::string TesolloSlaveNode::getDefaultCalibrationPath() const {
    const char* home = std::getenv("HOME");
    if (home) {
        std::string ros_dir = std::string(home) + "/.ros";
        mkdir(ros_dir.c_str(), 0755);
        return ros_dir + "/tesollo_offsets.yaml";
    }
    return "/tmp/tesollo_offsets.yaml";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TesolloSlaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
