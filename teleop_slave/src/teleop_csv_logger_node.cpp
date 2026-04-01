#include "teleop_slave/teleop_csv_logger_node.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rmw/qos_profiles.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

constexpr std::size_t kDefaultTesolloJointCount = 20;

std::string ExpandHomePath(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    const char* home = std::getenv("HOME");
    if (home == nullptr) {
        return path;
    }

    if (path.size() == 1) {
        return std::string(home);
    }

    if (path[1] == '/') {
        return std::string(home) + path.substr(1);
    }

    return path;
}

bool EnsureDirectoryExists(const std::string& directory, std::string* error_message) {
    if (directory.empty()) {
        if (error_message != nullptr) {
            *error_message = "output directory is empty";
        }
        return false;
    }

    std::string normalized = directory;
    while (normalized.size() > 1 && normalized.back() == '/') {
        normalized.pop_back();
    }

    std::string current;
    std::size_t start = 0;
    if (!normalized.empty() && normalized.front() == '/') {
        current = "/";
        start = 1;
    }

    while (start <= normalized.size()) {
        const std::size_t end = normalized.find('/', start);
        const std::string segment = normalized.substr(start, end - start);
        if (!segment.empty()) {
            if (!current.empty() && current.back() != '/') {
                current += '/';
            }
            current += segment;

            if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                if (error_message != nullptr) {
                    std::ostringstream stream;
                    stream << "failed to create directory '" << current
                           << "': " << std::strerror(errno);
                    *error_message = stream.str();
                }
                return false;
            }
        }

        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }

    return true;
}

std::string SanitizeCsvLabel(const std::string& label) {
    if (label.empty()) {
        return "unnamed_joint";
    }

    std::string sanitized = label;
    for (char& ch : sanitized) {
        const bool keep = (ch >= '0' && ch <= '9') ||
                          (ch >= 'a' && ch <= 'z') ||
                          (ch >= 'A' && ch <= 'Z') ||
                          ch == '_';
        if (!keep) {
            ch = '_';
        }
    }
    return sanitized;
}

std::vector<std::string> BuildJointLabels(const sensor_msgs::msg::JointState& msg) {
    const std::size_t count = msg.position.empty() ? kDefaultTesolloJointCount : msg.position.size();
    std::vector<std::string> labels;
    labels.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        if (i < msg.name.size() && !msg.name[i].empty()) {
            labels.push_back(SanitizeCsvLabel(msg.name[i]));
        } else {
            std::ostringstream stream;
            stream << "tesollo_joint_" << std::setw(2) << std::setfill('0') << (i + 1);
            labels.push_back(stream.str());
        }
    }
    return labels;
}

std::string BuildSessionFilename(const std::string& prefix) {
    const auto now = std::chrono::system_clock::now();
    const auto epoch_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    const std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    localtime_r(&now_time_t, &local_tm);

    char time_buffer[32];
    std::strftime(time_buffer, sizeof(time_buffer), "%Y%m%d_%H%M%S", &local_tm);

    std::ostringstream stream;
    stream << SanitizeCsvLabel(prefix) << "_" << time_buffer << "_"
           << std::setw(3) << std::setfill('0') << (epoch_ms % 1000) << ".csv";
    return stream.str();
}

rclcpp::Time ResolveMessageTime(const builtin_interfaces::msg::Time& stamp,
                                const rclcpp::Time& fallback_time) {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
        return fallback_time;
    }
    return rclcpp::Time(stamp);
}

uint64_t TimeToNanoseconds(const rclcpp::Time& time) {
    return static_cast<uint64_t>(time.nanoseconds());
}

std::string FormatSeconds(const rclcpp::Time& time) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(9) << time.seconds();
    return stream.str();
}

tf2::Quaternion NormalizeQuaternion(const geometry_msgs::msg::Quaternion& orientation) {
    tf2::Quaternion q;
    tf2::fromMsg(orientation, q);
    if (q.length2() < 1e-12) {
        q.setValue(0.0, 0.0, 0.0, 1.0);
        return q;
    }
    q.normalize();
    return q;
}

void QuaternionToRpyDegrees(const tf2::Quaternion& q,
                            double* roll_deg,
                            double* pitch_deg,
                            double* yaw_deg) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    *roll_deg = roll * 180.0 / M_PI;
    *pitch_deg = pitch * 180.0 / M_PI;
    *yaw_deg = yaw * 180.0 / M_PI;
}

}  // namespace

TeleopCsvLoggerNode::TeleopCsvLoggerNode()
    : Node("teleop_csv_logger_node") {
    this->declare_parameter("tesollo_joint_topic", "/dg5f_right/joint_states");
    this->declare_parameter("fairino_pose_topic", "/robot_pose");
    this->declare_parameter("output_directory", "~/.ros/teleop_csv_logs");
    this->declare_parameter("file_prefix", "teleop_record");
    this->declare_parameter("sample_rate_hz", 10.0);
    this->declare_parameter("sync_queue_size", 50);
    this->declare_parameter("sync_max_interval_sec", 0.02);

    tesollo_joint_topic_ = this->get_parameter("tesollo_joint_topic").as_string();
    fairino_pose_topic_ = this->get_parameter("fairino_pose_topic").as_string();
    output_directory_ = ExpandHomePath(this->get_parameter("output_directory").as_string());
    file_prefix_ = this->get_parameter("file_prefix").as_string();
    sample_rate_hz_ = this->get_parameter("sample_rate_hz").as_double();
    sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();
    sync_max_interval_sec_ = this->get_parameter("sync_max_interval_sec").as_double();

    if (sample_rate_hz_ <= 0.0) {
        throw std::runtime_error("sample_rate_hz must be > 0");
    }
    if (sync_queue_size_ <= 0) {
        throw std::runtime_error("sync_queue_size must be > 0");
    }
    if (sync_max_interval_sec_ < 0.0) {
        throw std::runtime_error("sync_max_interval_sec must be >= 0");
    }

    sample_period_ns_ = static_cast<int64_t>(std::llround(1.0e9 / sample_rate_hz_));

    tesollo_joint_sub_.subscribe(this, tesollo_joint_topic_, rmw_qos_profile_sensor_data);
    fairino_pose_sub_.subscribe(this, fairino_pose_topic_, rmw_qos_profile_sensor_data);

    SyncPolicy sync_policy(static_cast<uint32_t>(sync_queue_size_));
    sync_policy.setMaxIntervalDuration(
        rclcpp::Duration::from_nanoseconds(
            static_cast<rcl_duration_value_t>(std::llround(sync_max_interval_sec_ * 1.0e9))));
    synchronizer_ = std::shared_ptr<Synchronizer>(
        new Synchronizer(
            static_cast<const SyncPolicy&>(sync_policy),
            tesollo_joint_sub_,
            fairino_pose_sub_));
    synchronizer_->registerCallback(
        std::bind(
            &TeleopCsvLoggerNode::synchronizedSampleCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Teleop CSV logger ready. Press SPACE to start/stop recording.");
    RCLCPP_INFO(this->get_logger(),
                "  Tesollo joint topic: %s", tesollo_joint_topic_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "  Fairino pose topic: %s", fairino_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "  Output directory: %s", output_directory_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "  Sample rate: %.1f Hz", sample_rate_hz_);
    RCLCPP_INFO(this->get_logger(),
                "  Sync queue size: %d", sync_queue_size_);
    RCLCPP_INFO(this->get_logger(),
                "  Sync max interval: %.4f s", sync_max_interval_sec_);
    RCLCPP_INFO(this->get_logger(),
                "  Press q or ESC to exit the logger node.");

    keyboard_thread_ = std::thread(&TeleopCsvLoggerNode::keyboardThread, this);
}

TeleopCsvLoggerNode::~TeleopCsvLoggerNode() {
    running_ = false;
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }

    std::lock_guard<std::mutex> lock(file_mutex_);
    closeCsvLocked();
}

void TeleopCsvLoggerNode::keyboardThread() {
    if (!::isatty(STDIN_FILENO)) {
        RCLCPP_WARN(this->get_logger(),
                    "STDIN is not a TTY. Keyboard toggle is disabled for teleop_csv_logger_node.");
        return;
    }

    struct termios oldt {};
    struct termios newt {};
    if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read terminal settings for keyboard input.");
        return;
    }

    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 1;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch terminal to raw mode.");
        return;
    }

    while (running_ && rclcpp::ok()) {
        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == ' ') {
                if (recording_) {
                    stopRecordingSession();
                } else {
                    startRecordingSession();
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

bool TeleopCsvLoggerNode::startRecordingSession() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    if (recording_) {
        RCLCPP_WARN(this->get_logger(), "Recording is already active: %s", active_csv_path_.c_str());
        return false;
    }

    std::string mkdir_error;
    if (!EnsureDirectoryExists(output_directory_, &mkdir_error)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to prepare output directory: %s", mkdir_error.c_str());
        return false;
    }

    active_csv_path_ = output_directory_;
    if (!active_csv_path_.empty() && active_csv_path_.back() != '/') {
        active_csv_path_ += '/';
    }
    active_csv_path_ += BuildSessionFilename(file_prefix_);

    csv_file_.open(active_csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", active_csv_path_.c_str());
        active_csv_path_.clear();
        return false;
    }

    session_joint_names_.clear();
    session_joint_count_ = 0;
    header_written_ = false;
    rows_written_ = 0;
    last_written_pair_time_ns_ = 0;
    last_written_tesollo_stamp_ns_ = 0;
    last_written_fairino_stamp_ns_ = 0;
    recording_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Recording started: %s", active_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Waiting for synchronized Tesollo/Fairino message pairs.");
    return true;
}

void TeleopCsvLoggerNode::stopRecordingSession() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    if (!recording_) {
        return;
    }

    recording_ = false;
    const std::string finished_path = active_csv_path_;
    const std::size_t finished_rows = rows_written_;
    closeCsvLocked();

    RCLCPP_INFO(this->get_logger(),
                "Recording stopped. Wrote %zu rows to %s",
                finished_rows,
                finished_path.c_str());
}

void TeleopCsvLoggerNode::writeCsvHeaderLocked() {
    csv_file_ << "record_time_sec,tesollo_time_sec,fairino_time_sec";
    for (const auto& joint_name : session_joint_names_) {
        csv_file_ << "," << joint_name << "_rad";
    }
    csv_file_ << ",fairino_x_m"
              << ",fairino_y_m"
              << ",fairino_z_m"
              << ",fairino_qx"
              << ",fairino_qy"
              << ",fairino_qz"
              << ",fairino_qw"
              << ",fairino_roll_deg"
              << ",fairino_pitch_deg"
              << ",fairino_yaw_deg"
              << "\n";
    csv_file_.flush();
}

void TeleopCsvLoggerNode::closeCsvLocked() {
    if (csv_file_.is_open()) {
        csv_file_.flush();
        csv_file_.close();
    }
    active_csv_path_.clear();
    session_joint_names_.clear();
    session_joint_count_ = 0;
    header_written_ = false;
}

void TeleopCsvLoggerNode::synchronizedSampleCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr& tesollo_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& fairino_msg) {
    if (!recording_) {
        return;
    }

    const rclcpp::Time record_time = this->now();
    const rclcpp::Time tesollo_time =
        ResolveMessageTime(tesollo_msg->header.stamp, record_time);
    const rclcpp::Time fairino_time =
        ResolveMessageTime(fairino_msg->header.stamp, record_time);
    const uint64_t tesollo_stamp_ns = TimeToNanoseconds(tesollo_time);
    const uint64_t fairino_stamp_ns = TimeToNanoseconds(fairino_time);
    const int64_t pair_time_ns = std::max(
        static_cast<int64_t>(tesollo_stamp_ns),
        static_cast<int64_t>(fairino_stamp_ns));

    std::lock_guard<std::mutex> lock(file_mutex_);
    if (!recording_ || !csv_file_.is_open()) {
        return;
    }

    if (tesollo_stamp_ns == last_written_tesollo_stamp_ns_ &&
        fairino_stamp_ns == last_written_fairino_stamp_ns_) {
        return;
    }

    if (last_written_pair_time_ns_ != 0) {
        if (pair_time_ns <= last_written_pair_time_ns_) {
            return;
        }
        if ((pair_time_ns - last_written_pair_time_ns_) < sample_period_ns_) {
            return;
        }
    }

    if (!header_written_) {
        session_joint_names_ = BuildJointLabels(*tesollo_msg);
        session_joint_count_ = session_joint_names_.size();
        writeCsvHeaderLocked();
        header_written_ = true;
    }

    const tf2::Quaternion pose_q = NormalizeQuaternion(fairino_msg->pose.orientation);
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    QuaternionToRpyDegrees(pose_q, &roll_deg, &pitch_deg, &yaw_deg);

    csv_file_ << FormatSeconds(record_time)
              << "," << FormatSeconds(tesollo_time)
              << "," << FormatSeconds(fairino_time);

    for (std::size_t i = 0; i < session_joint_count_; ++i) {
        csv_file_ << ",";
        if (i < tesollo_msg->position.size()) {
            csv_file_ << std::fixed << std::setprecision(9) << tesollo_msg->position[i];
        }
    }

    csv_file_ << "," << std::fixed << std::setprecision(9) << fairino_msg->pose.position.x
              << "," << std::fixed << std::setprecision(9) << fairino_msg->pose.position.y
              << "," << std::fixed << std::setprecision(9) << fairino_msg->pose.position.z
              << "," << std::fixed << std::setprecision(9) << pose_q.x()
              << "," << std::fixed << std::setprecision(9) << pose_q.y()
              << "," << std::fixed << std::setprecision(9) << pose_q.z()
              << "," << std::fixed << std::setprecision(9) << pose_q.w()
              << "," << std::fixed << std::setprecision(6) << roll_deg
              << "," << std::fixed << std::setprecision(6) << pitch_deg
              << "," << std::fixed << std::setprecision(6) << yaw_deg
              << "\n";
    csv_file_.flush();

    last_written_pair_time_ns_ = pair_time_ns;
    last_written_tesollo_stamp_ns_ = tesollo_stamp_ns;
    last_written_fairino_stamp_ns_ = fairino_stamp_ns;
    ++rows_written_;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TeleopCsvLoggerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
