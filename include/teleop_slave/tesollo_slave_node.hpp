#ifndef TESOLLO_SLAVE_NODE_HPP_
#define TESOLLO_SLAVE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class TesolloSlaveNode : public rclcpp::Node {
public:
    TesolloSlaveNode();

private:
    void fingerJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr finger_sub_;
};

#endif // TESOLLO_SLAVE_NODE_HPP_
