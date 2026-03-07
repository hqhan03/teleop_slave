#ifndef TESOLLO_SLAVE_NODE_HPP_
#define TESOLLO_SLAVE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "teleop_slave/dg5f_operator_TCP.hpp"
#include <memory>
#include <string>

class TesolloSlaveNode : public rclcpp::Node {
public:
    TesolloSlaveNode();

private:
    void fingerJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr finger_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

    std::string ip_;
    uint16_t port_;
    bool dummy_mode_ = false;
    std::unique_ptr<DG5F_TCP> delto_client_;
    bool connected_ = false;
};

#endif // TESOLLO_SLAVE_NODE_HPP_
