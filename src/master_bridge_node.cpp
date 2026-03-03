#include "teleop_slave/master_bridge_node.hpp"
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

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

    // Wrist Pose
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = packet.wristPos[0];
    pose_msg.pose.position.y = packet.wristPos[1];
    pose_msg.pose.position.z = packet.wristPos[2];
    set_quaternion_from_euler(pose_msg.pose.orientation, packet.wristEuler[0], packet.wristEuler[1], packet.wristEuler[2]);
    wrist_pub_->publish(pose_msg);

    // Finger Joints
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = now;
    joint_msg.name = {"thumb_mcp", "thumb_pip", "thumb_dip", "index_mcp", "index_pip", "index_dip",
                      "middle_mcp", "middle_pip", "middle_dip", "ring_mcp", "ring_pip", "ring_dip",
                      "pinky_mcp", "pinky_pip", "pinky_dip"};

    for (int i = 0; i < 15; ++i) {
        joint_msg.position.push_back(packet.fingerFlexion[i] * M_PI / 180.0);
    }
    joint_pub_->publish(joint_msg);
}

void ManusReceiverNode::set_quaternion_from_euler(geometry_msgs::msg::Quaternion& q, float r, float p, float y) {
    float roll = r * M_PI / 180.0;
    float pitch = p * M_PI / 180.0;
    float yaw = y * M_PI / 180.0;

    double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5), sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManusReceiverNode>());
    rclcpp::shutdown();
    return 0;
}