#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class TeleopCsvLoggerNode : public rclcpp::Node {
public:
    TeleopCsvLoggerNode();
    ~TeleopCsvLoggerNode();

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::JointState,
        geometry_msgs::msg::PoseStamped>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    void synchronizedSampleCallback(
        const sensor_msgs::msg::JointState::ConstSharedPtr& tesollo_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& fairino_msg);
    void keyboardThread();

    bool startRecordingSession();
    void stopRecordingSession();
    void writeCsvHeaderLocked();
    void closeCsvLocked();

    message_filters::Subscriber<sensor_msgs::msg::JointState> tesollo_joint_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> fairino_pose_sub_;
    std::shared_ptr<Synchronizer> synchronizer_;

    std::mutex file_mutex_;
    std::ofstream csv_file_;
    std::string active_csv_path_;
    std::vector<std::string> session_joint_names_;
    std::size_t session_joint_count_{0};
    bool header_written_{false};
    int64_t sample_period_ns_{100000000};
    int64_t last_written_pair_time_ns_{0};
    uint64_t last_written_tesollo_stamp_ns_{0};
    uint64_t last_written_fairino_stamp_ns_{0};
    std::size_t rows_written_{0};

    std::string tesollo_joint_topic_;
    std::string fairino_pose_topic_;
    std::string output_directory_;
    std::string file_prefix_;
    double sample_rate_hz_{10.0};
    int sync_queue_size_{50};
    double sync_max_interval_sec_{0.02};

    std::atomic<bool> recording_{false};
    std::atomic<bool> running_{true};
    std::thread keyboard_thread_;
};
