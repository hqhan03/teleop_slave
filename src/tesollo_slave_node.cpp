#include "teleop_slave/tesollo_slave_node.hpp"
#include <cmath>
#include <algorithm>

TesolloSlaveNode::TesolloSlaveNode() : Node("tesollo_slave_node") {
    this->declare_parameter("ip", "169.254.186.72");
    this->declare_parameter("port", 502);
    this->declare_parameter("dummy_mode", false);
    
    ip_ = this->get_parameter("ip").as_string();
    port_ = this->get_parameter("port").as_int();
    dummy_mode_ = this->get_parameter("dummy_mode").as_bool();

    finger_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/manus/finger_joints", 10,
        std::bind(&TesolloSlaveNode::fingerJointsCallback, this, std::placeholders::_1));

    if (dummy_mode_) {
        RCLCPP_INFO(this->get_logger(), "Tesollo Slave Node Started in DUMMY_MODE (Gazebo Simulation)");
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        connected_ = true; // Bypasses ModbusTCP connection check 
    } else {
        try {
            RCLCPP_INFO(this->get_logger(), "Connecting to Tesollo DG-5F at %s:%d", ip_.c_str(), port_);
            delto_client_ = std::make_unique<DG5F_TCP>(ip_, port_);
            delto_client_->connect();
            delto_client_->start_control();
            connected_ = true;
            RCLCPP_INFO(this->get_logger(), "Tesollo Slave Node Started and Connected");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to DG-5F: %s", e.what());
        }
    }
}

void TesolloSlaveNode::fingerJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!connected_ || msg->position.size() < 20) {
        return;
    }
    
    if (!dummy_mode_ && !delto_client_) {
        return;
    }

    std::vector<double> delto_target(20, 0.0);

    constexpr double D2R = M_PI / 180.0;
    auto clamp_rad = [](double angle_rad, double min_deg, double max_deg) {
        return std::clamp(angle_rad, min_deg * D2R, max_deg * D2R);
    };

    // Thumb (Motors 1-4)
    delto_target[0] = -clamp_rad(msg->position[1], -77, 22);  // Motor 1: CMC_Ab/Ad (Sign Inversion)
    delto_target[1] = -clamp_rad(msg->position[0], -155, 0);  // Motor 2: CMC_Fl/Ex (Sign Inversion)
    delto_target[2] = clamp_rad(msg->position[2], -90, 90);   // Motor 3: MCP_Fl/Ex
    delto_target[3] = clamp_rad(msg->position[3], -90, 90);   // Motor 4: IP_Fl/Ex

    // Index (Motors 5-8)
    // delto_target[4] = -clamp_rad(msg->position[4], -20, 31);  // Motor 5: MCP_Ab/Ad (Sign Inversion)
    delto_target[4] = 0.0; // Temporarily disabled
    delto_target[5] = clamp_rad(msg->position[5], 0, 115);    // Motor 6: MCP_Fl/Ex
    delto_target[6] = clamp_rad(msg->position[6], -90, 90);   // Motor 7: PIP_Fl/Ex
    delto_target[7] = clamp_rad(msg->position[7], -90, 90);   // Motor 8: DIP_Fl/Ex

    // Middle (Motors 9-12)
    // delto_target[8] = -clamp_rad(msg->position[8], -30, 30);  // Motor 9: MCP_Ab/Ad (Sign Inversion)
    delto_target[8] = 0.0; // Temporarily disabled
    delto_target[9] = clamp_rad(msg->position[9], 0, 115);    // Motor 10: MCP_Fl/Ex
    delto_target[10] = clamp_rad(msg->position[10], -90, 90); // Motor 11: PIP_Fl/Ex
    delto_target[11] = clamp_rad(msg->position[11], -90, 90); // Motor 12: DIP_Fl/Ex

    // Ring (Motors 13-16)
    // delto_target[12] = -clamp_rad(msg->position[12], -32, 15);// Motor 13: MCP_Ab/Ad (Sign Inversion)
    delto_target[12] = 0.0; // Temporarily disabled
    delto_target[13] = clamp_rad(msg->position[13], 0, 110);  // Motor 14: MCP_Fl/Ex
    delto_target[14] = clamp_rad(msg->position[14], -90, 90); // Motor 15: PIP_Fl/Ex
    delto_target[15] = clamp_rad(msg->position[15], -90, 90); // Motor 16: DIP_Fl/Ex

    // Pinky (Motors 17-20)
    delto_target[16] = clamp_rad(0.0, 0, 60);                 // Motor 17: Fixed (0.0 rad)
    // delto_target[17] = -clamp_rad(msg->position[16], -90, 15);// Motor 18: MCP_Ab/Ad (Sign Inversion)
    delto_target[17] = 0.0; // Temporarily disabled
    delto_target[18] = clamp_rad(msg->position[17], -90, 90); // Motor 19: MCP_Fl/Ex
    
    // Combine Pinky PIP and DIP into the distal Motor 20
    delto_target[19] = clamp_rad(msg->position[18] + msg->position[19], -90, 90); // Motor 20: IP_Fl/Ex

    if (dummy_mode_) {
        // Build the JointTrajectory for ros2_control in Gazebo
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        // Do NOT set traj_msg.header.stamp = now() here because Gazebo uses use_sim_time (starts from 0sec). 
        // An empty stamp defaults to '0' which means the controller executes it immediately upon receipt.
        
        // The Gazebo standard names from Tesollo's configuration right hand "rj_dg_X_X"
        traj_msg.joint_names = {
            "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
            "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
            "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
            "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
            "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4"
        };
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = delto_target;
        // The time from start tells the simulated trajectory controller how fast to reach the point
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 100000000; // 0.1s for smooth interpolation
        
        traj_msg.points.push_back(point);
        traj_pub_->publish(traj_msg);
    } else {
        // Send the positional targets down to the RS485 chain via ModbusTCP
        delto_client_->set_position_rad(delto_target);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TesolloSlaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
