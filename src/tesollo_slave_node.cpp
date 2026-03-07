#include "teleop_slave/tesollo_slave_node.hpp"

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

    // Motors 1-16 (Thumb, Index, Middle, Ring) direct mapping
    for (int i = 0; i < 16; ++i) {
        delto_target[i] = msg->position[i];
    }

    // Pinky Finger Custom Kinematics
    // Motor 17 (idx 16) is a fixed DOF not present in the human hand
    delto_target[16] = 0.0; 
    
    // Shift Pinky MCP Roll and Pitch
    delto_target[17] = msg->position[16];   // Pinky MCP Spread
    delto_target[18] = msg->position[17];   // Pinky MCP Stretch
    
    // Combine PIP and DIP into the distal Motor 20
    delto_target[19] = msg->position[18] + msg->position[19];

    if (dummy_mode_) {
        // Build the JointTrajectory for ros2_control in Gazebo
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        traj_msg.header.stamp = this->get_clock()->now();
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
        point.time_from_start.nanosec = 25000000; // 0.025s (40 Hz tracking curve)
        
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
