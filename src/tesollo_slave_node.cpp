#include "teleop_slave/tesollo_slave_node.hpp"
#include <cmath>
#include <algorithm>

TesolloSlaveNode::TesolloSlaveNode() : Node("tesollo_slave_node") {
    this->declare_parameter<bool>("dummy_mode", false);
    
    this->get_parameter("dummy_mode", dummy_mode_);

    finger_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/manus/finger_joints", 10,
        std::bind(&TesolloSlaveNode::fingerJointsCallback, this, std::placeholders::_1));

    if (dummy_mode_) {
        RCLCPP_INFO(this->get_logger(), "Tesollo Slave Node Started in DUMMY_MODE (Gazebo Simulation)");
    } else {
        RCLCPP_INFO(this->get_logger(), "Tesollo Slave Node Started. Ensure the official dg5f_driver is running!");
    }

    std::string traj_topic = dummy_mode_ ? 
        "/joint_trajectory_controller/joint_trajectory" : 
        "/dg5f_right/dg5f_right_controller/joint_trajectory";
        
    connected_ = true;
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic, 10);
}

void TesolloSlaveNode::fingerJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!connected_ || msg->position.size() < 20) {
        return;
    }

    std::vector<double> delto_target(20, 0.0);

    constexpr double D2R = M_PI / 180.0;
    auto clamp_rad = [](double angle_rad, double min_deg, double max_deg) {
        return std::clamp(angle_rad, min_deg * D2R, max_deg * D2R);
    };

    // Thumb (Motors 1-4)
    // delto_target[0] = -clamp_rad(msg->position[1], -76, 21);  // Motor 1: CMC_Ab/Ad (Sign Inversion)
    // delto_target[1] = -clamp_rad(msg->position[0], -154, -1); // Motor 2: CMC_Fl/Ex (Sign Inversion)
    // delto_target[2] = clamp_rad(msg->position[2], -89, 89);   // Motor 3: MCP_Fl/Ex
    // delto_target[3] = clamp_rad(msg->position[3], -89, 89);   // Motor 4: IP_Fl/Ex
    delto_target[0] = 0.0;
    delto_target[1] = 0.0;
    delto_target[2] = 0.0;
    delto_target[3] = 0.0;

    // Index (Motors 5-8)
    // delto_target[4] = -clamp_rad(msg->position[4], -19, 30);  // Motor 5: MCP_Ab/Ad (Sign Inversion)
    delto_target[4] = 0.0; // Temporarily disabled
    delto_target[5] = clamp_rad(msg->position[5], 1, 114);    // Motor 6: MCP_Fl/Ex
    delto_target[6] = clamp_rad(msg->position[6], -89, 89);   // Motor 7: PIP_Fl/Ex
    delto_target[7] = clamp_rad(msg->position[7], -89, 89);   // Motor 8: DIP_Fl/Ex

    // Middle (Motors 9-12)
    // delto_target[8] = -clamp_rad(msg->position[8], -29, 29);  // Motor 9: MCP_Ab/Ad (Sign Inversion)
    delto_target[8] = 0.0; // Temporarily disabled
    delto_target[9] = clamp_rad(msg->position[9], 1, 114);    // Motor 10: MCP_Fl/Ex
    delto_target[10] = clamp_rad(msg->position[10], -89, 89); // Motor 11: PIP_Fl/Ex
    delto_target[11] = clamp_rad(msg->position[11], -89, 89); // Motor 12: DIP_Fl/Ex

    // Ring (Motors 13-16)
    // delto_target[12] = -clamp_rad(msg->position[12], -31, 14);// Motor 13: MCP_Ab/Ad (Sign Inversion)
    delto_target[12] = 0.0; // Temporarily disabled
    delto_target[13] = clamp_rad(msg->position[13], 1, 109);  // Motor 14: MCP_Fl/Ex
    delto_target[14] = clamp_rad(msg->position[14], -89, 89); // Motor 15: PIP_Fl/Ex
    delto_target[15] = clamp_rad(msg->position[15], -89, 89); // Motor 16: DIP_Fl/Ex

    // Pinky (Motors 17-20)
    delto_target[16] = 0.0;                                   // Motor 17: Fixed (0.0 rad)
    // delto_target[17] = -clamp_rad(msg->position[16], -89, 14);// Motor 18: MCP_Ab/Ad (Sign Inversion)
    delto_target[17] = 0.0; // Temporarily disabled
    delto_target[18] = clamp_rad(msg->position[17], -89, 89); // Motor 19: MCP_Fl/Ex
    
    // Combine Pinky PIP and DIP into the distal Motor 20
    delto_target[19] = clamp_rad(msg->position[18] + msg->position[19], -89, 89); // Motor 20: IP_Fl/Ex

    // Build the JointTrajectory for ros2_control in Gazebo AND Physical Hardware
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
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TesolloSlaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
