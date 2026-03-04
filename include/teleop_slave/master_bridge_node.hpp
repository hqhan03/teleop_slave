#ifndef MANUS_RECEIVER_NODE_HPP_
#define MANUS_RECEIVER_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

// UDP & Socket
#include <sys/socket.h>
#include <netinet/in.h>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Windows와 데이터 포맷을 맞추기 위한 구조체
#pragma pack(push, 1)
struct HandDataPacket {
    uint32_t frame;
    float wristPos[3];
    float wristEuler[3];
    float fingerFlexion[20]; // <-- 15에서 20으로 확장
};
#pragma pack(pop)

class ManusReceiverNode : public rclcpp::Node {
public:
    ManusReceiverNode();
    ~ManusReceiverNode();

private:
    // 초기화 및 콜백
    void setup_udp();
    void receive_callback();
    void publish_data(const HandDataPacket& packet);
    
    // 유틸리티
    void set_quaternion_from_euler(geometry_msgs::msg::Quaternion& q, float r, float p, float y);

    // 멤버 변수
    int sockfd_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wrist_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif