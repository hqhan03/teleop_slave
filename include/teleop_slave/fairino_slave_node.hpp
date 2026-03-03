#ifndef FAIRINO_SLAVE_NODE_HPP_
#define FAIRINO_SLAVE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class FairinoSlaveNode : public rclcpp::Node {
public:
    FairinoSlaveNode();

private:
    void manusPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void startStreaming();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manus_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr curobo_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stream_client_;

    bool streaming_started_;
};

#endif // FAIRINO_SLAVE_NODE_HPP_
