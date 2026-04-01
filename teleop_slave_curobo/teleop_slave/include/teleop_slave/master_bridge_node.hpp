#ifndef MANUS_RECEIVER_NODE_HPP_
#define MANUS_RECEIVER_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <cstdint>

// UDP & Socket
#include <sys/socket.h>
#include <netinet/in.h>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Windows와 데이터 포맷을 맞추기 위한 구조체
#pragma pack(push, 1)
struct HandDataPacket {
    uint32_t frame;
    float wristPos[3];
    float wristQuaternion[4]; // w, x, y, z
    float fingerFlexion[20];
    float fingertipPos[15]; // 5 fingertips × (X, Y, Z) in MANUS wrist-local frame
    float landmarkPos[75]; // 25 hand landmarks × (X, Y, Z) in wrist-local frame
                           // Order: 5 fingers × 5 joints (Metacarpal, Proximal, Intermediate, Distal, Tip)
                           // Finger order: Thumb, Index, Middle, Ring, Pinky
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
    
    
    // 멤버 변수
    int sockfd_;
    int listen_port_ = 12345;
    uint64_t packet_count_ = 0;
    uint64_t no_packet_count_ = 0;
    bool first_packet_logged_ = false;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wrist_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr fingertip_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmark_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
