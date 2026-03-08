#include "teleop_slave/master_bridge_node.hpp"
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio> // printf 사용

ManusReceiverNode::ManusReceiverNode() : Node("manus_receiver_cpp"), sockfd_(-1) {
    wrist_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("manus/wrist_pose", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("manus/finger_joints", 10);

    setup_udp();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ManusReceiverNode::receive_callback, this));

    RCLCPP_INFO(this->get_logger(), "NREL MANUS C++ Receiver (HPP/CPP Split) Started on Port 12345");
}

ManusReceiverNode::~ManusReceiverNode() {
    if (sockfd_ != -1) close(sockfd_);
}

void ManusReceiverNode::setup_udp() {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(12345);

    bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr));
    fcntl(sockfd_, F_SETFL, O_NONBLOCK);
}

void ManusReceiverNode::receive_callback() {
    HandDataPacket packet;
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);

    ssize_t n = recvfrom(sockfd_, &packet, sizeof(packet), 0, (struct sockaddr *)&cliaddr, &len);
    if (n == sizeof(HandDataPacket)) {
        publish_data(packet);
    }
}

void ManusReceiverNode::publish_data(const HandDataPacket& packet) {
    auto now = this->get_clock()->now();

    // 1. 데이터 퍼블리시 (Wrist Pose)
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = packet.wristPos[0];
    pose_msg.pose.position.y = packet.wristPos[1];
    pose_msg.pose.position.z = packet.wristPos[2];
    
    // Assign quaternion directly: w, x, y, z
    pose_msg.pose.orientation.w = packet.wristQuaternion[0];
    pose_msg.pose.orientation.x = packet.wristQuaternion[1];
    pose_msg.pose.orientation.y = packet.wristQuaternion[2];
    pose_msg.pose.orientation.z = packet.wristQuaternion[3];
    
    wrist_pub_->publish(pose_msg);

    // 2. 데이터 퍼블리시 (Finger Joints - 20개로 확장)
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = now;
    
    // 조인트 이름을 20개에 맞게 업데이트 (Spread와 Stretch를 분리)
    joint_msg.name = {
        "thumb_mcp_spread", "thumb_mcp_stretch", "thumb_pip", "thumb_dip",
        "index_mcp_spread", "index_mcp_stretch", "index_pip", "index_dip",
        "middle_mcp_spread", "middle_mcp_stretch", "middle_pip", "middle_dip",
        "ring_mcp_spread", "ring_mcp_stretch", "ring_pip", "ring_dip",
        "pinky_mcp_spread", "pinky_mcp_stretch", "pinky_pip", "pinky_dip"
    };

    // 20개의 손가락 관절 데이터를 퍼블리시
    for (int i = 0; i < 20; ++i) {
        joint_msg.position.push_back(packet.fingerFlexion[i] * M_PI / 180.0);
    }
    joint_pub_->publish(joint_msg);

    // 3. 터미널 데이터 출력 (20개의 데이터 실시간 출력)
    printf("\033[2J\033[H");
    printf("=== [KAIST NREL] MANUS -> ROS2 Humble (UDP 50Hz) ===\n");
    printf("[Wrist] Pos: X:%.3f Y:%.3f Z:%.3f | Quat: W:%.3f X:%.3f Y:%.3f Z:%.3f\n",
        packet.wristPos[0], packet.wristPos[1], packet.wristPos[2], 
        packet.wristQuaternion[0], packet.wristQuaternion[1], packet.wristQuaternion[2], packet.wristQuaternion[3]);

    printf("[Received UDP Data] Sending 20 Finger Joints...\n");
    printf("  Thumb:  CMC_Fl/Ex=%.2f CMC_Ab/Ad=%.2f MCP_Fl/Ex=%.2f IP_Fl/Ex=%.2f\n",
        packet.fingerFlexion[0], packet.fingerFlexion[1], packet.fingerFlexion[2], packet.fingerFlexion[3]);
    printf("  Index:  MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
        packet.fingerFlexion[4], packet.fingerFlexion[5], packet.fingerFlexion[6], packet.fingerFlexion[7]);
    printf("  Middle: MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
        packet.fingerFlexion[8], packet.fingerFlexion[9], packet.fingerFlexion[10], packet.fingerFlexion[11]);
    printf("  Ring:   MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
        packet.fingerFlexion[12], packet.fingerFlexion[13], packet.fingerFlexion[14], packet.fingerFlexion[15]);
    printf("  Pinky:  MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
        packet.fingerFlexion[16], packet.fingerFlexion[17], packet.fingerFlexion[18], packet.fingerFlexion[19]);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManusReceiverNode>());
    rclcpp::shutdown();
    return 0;
}