#include "teleop_slave/master_bridge_node.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio> // printf 사용
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace {
constexpr char kManusHandFrameId[] = "manus_wrist_local";
}

ManusReceiverNode::ManusReceiverNode() : Node("manus_receiver_cpp"), sockfd_(-1) {
    this->declare_parameter("listen_port", 12345);
    listen_port_ = this->get_parameter("listen_port").as_int();

    wrist_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("manus/wrist_pose", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("manus/finger_joints", 10);
    fingertip_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("manus/fingertip_positions", 10);
    landmark_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("manus/hand_landmarks", 10);

    setup_udp();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ManusReceiverNode::receive_callback, this));

    RCLCPP_INFO(this->get_logger(), "NREL MANUS C++ Receiver started on UDP port %d", listen_port_);
}

ManusReceiverNode::~ManusReceiverNode() {
    if (sockfd_ != -1) close(sockfd_);
}

void ManusReceiverNode::setup_udp() {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket: %s", std::strerror(errno));
        throw std::runtime_error("UDP socket creation failed");
    }

    int reuse = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to enable SO_REUSEADDR: %s", std::strerror(errno));
    }

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(listen_port_);

    if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to bind UDP port %d: %s", listen_port_, std::strerror(errno));
        throw std::runtime_error("UDP bind failed");
    }

    if (fcntl(sockfd_, F_SETFL, O_NONBLOCK) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to enable non-blocking UDP socket: %s", std::strerror(errno));
        throw std::runtime_error("UDP socket setup failed");
    }

    RCLCPP_INFO(this->get_logger(), "UDP socket bound to 0.0.0.0:%d", listen_port_);
}

void ManusReceiverNode::receive_callback() {
    HandDataPacket packet;
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);

    ssize_t n = recvfrom(sockfd_, &packet, sizeof(packet), 0, (struct sockaddr *)&cliaddr, &len);

    const ssize_t min_size = (ssize_t)(
        sizeof(HandDataPacket) - sizeof(packet.fingertipPos) - sizeof(packet.landmarkPos));

    if (n > 0 && n < min_size) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Received undersized UDP packet: %zd bytes (need >= %zd)", n, min_size);
        return;
    }

    if (n >= min_size) {
        if (!first_packet_logged_) {
            first_packet_logged_ = true;
            RCLCPP_INFO(get_logger(), "First UDP packet received: %zd bytes, frame=%u from %s",
                n, packet.frame, inet_ntoa(cliaddr.sin_addr));
        }
        packet_count_++;
        // Zero out fields if old sender sends smaller packet (backward compat)
        const ssize_t size_without_landmarks = (ssize_t)(sizeof(HandDataPacket) - sizeof(packet.landmarkPos));
        if (n < (ssize_t)sizeof(HandDataPacket)) {
            memset(packet.landmarkPos, 0, sizeof(packet.landmarkPos));
        }
        if (n < size_without_landmarks) {
            memset(packet.fingertipPos, 0, sizeof(packet.fingertipPos));
        }
        publish_data(packet);
    } else {
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "recvfrom failed on UDP port %d: %s", listen_port_, std::strerror(errno));
            return;
        }

        // n <= 0: no packet available (EAGAIN/EWOULDBLOCK)
        no_packet_count_++;
        if (no_packet_count_ % 500 == 0 && packet_count_ == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "No UDP packets received yet (listening on port %d). "
                "Check: sender running? correct IP? firewall? "
                "TeleopMasterClient currently hardcodes its destination IP in TeleopMasterClient.cpp. "
                "Try: sudo tcpdump -i any udp port %d -c 5",
                listen_port_, listen_port_);
        }
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
    tf2::Quaternion q(
        packet.wristQuaternion[1], // x
        packet.wristQuaternion[2], // y
        packet.wristQuaternion[3], // z
        packet.wristQuaternion[0]  // w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert radians to degrees for terminal print
    double roll_deg = roll * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;
    double yaw_deg = yaw * 180.0 / M_PI;

    printf("\033[2J\033[H");
    printf("=== MANUS Core -> ROS2 Humble (UDP 50Hz) ===\n");
    printf("[VIVE Tracker] Pos: X:%.3f Y:%.3f Z:%.3f\n",
        packet.wristPos[0], packet.wristPos[1], packet.wristPos[2]);
    printf("        Quat: W:%.3f X:%.3f Y:%.3f Z:%.3f\n",
        packet.wristQuaternion[0], packet.wristQuaternion[1], packet.wristQuaternion[2], packet.wristQuaternion[3]);
    printf("        Euler: R:%.3f P:%.3f Y:%.3f\n",
        roll_deg, pitch_deg, yaw_deg);

    printf("[MANUS] Sending 20 Finger Joints...\n");
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

    // 3. 데이터 퍼블리시 (wrist-local raw skeleton fingertip positions)
    auto fingertip_msg = geometry_msgs::msg::PoseArray();
    fingertip_msg.header.stamp = now;
    fingertip_msg.header.frame_id = kManusHandFrameId;

    const char* fingerNames[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
    bool hasFingertipData = false;
    for (int i = 0; i < 5; ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = packet.fingertipPos[i * 3 + 0];
        pose.position.y = packet.fingertipPos[i * 3 + 1];
        pose.position.z = packet.fingertipPos[i * 3 + 2];
        pose.orientation.w = 1.0;  // Identity quaternion (position-only)
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        fingertip_msg.poses.push_back(pose);
        if (pose.position.x != 0.0f || pose.position.y != 0.0f || pose.position.z != 0.0f) {
            hasFingertipData = true;
        }
    }
    fingertip_pub_->publish(fingertip_msg);

    // 터미널에 Fingertip 위치 출력
    if (hasFingertipData) {
        printf("[Fingertip Positions (Raw Skeleton Wrist-Local Frame)]\n");
        for (int f = 0; f < 5; f++) {
            printf("  %-7s: X:%.4f Y:%.4f Z:%.4f\n", fingerNames[f],
                packet.fingertipPos[f * 3 + 0], packet.fingertipPos[f * 3 + 1], packet.fingertipPos[f * 3 + 2]);
        }
    }

    // 4. 데이터 퍼블리시 (25 hand landmarks for keyvector retargeting)
    // Order: 5 fingers × 5 joints (Metacarpal, Proximal, Intermediate, Distal, Tip)
    bool hasLandmarkData = false;
    for (int i = 0; i < 75; i++) {
        if (packet.landmarkPos[i] != 0.0f) { hasLandmarkData = true; break; }
    }
    if (hasLandmarkData) {
        auto landmark_msg = geometry_msgs::msg::PoseArray();
        landmark_msg.header.stamp = now;
        landmark_msg.header.frame_id = kManusHandFrameId;
        for (int i = 0; i < 25; i++) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = packet.landmarkPos[i * 3 + 0];
            pose.position.y = packet.landmarkPos[i * 3 + 1];
            pose.position.z = packet.landmarkPos[i * 3 + 2];
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            landmark_msg.poses.push_back(pose);
        }
        landmark_pub_->publish(landmark_msg);
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManusReceiverNode>());
    rclcpp::shutdown();
    return 0;
}
