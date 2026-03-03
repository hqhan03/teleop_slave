#include "teleop_slave/tesollo_slave_node.hpp"

TesolloSlaveNode::TesolloSlaveNode() : Node("tesollo_slave_node") {
    finger_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/manus/finger_joints", 10,
        std::bind(&TesolloSlaveNode::fingerJointsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Tesollo Slave Node Started");
}

void TesolloSlaveNode::fingerJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Implement Tesollo gripper control logic here
    (void)msg;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TesolloSlaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
