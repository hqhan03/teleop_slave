#include <memory>
#include <string>
#include <vector>
#include <cmath>

// UDP 통신을 위한 헤더 (Linux 표준 소켓)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

// ROS2 헤더
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// 1. Windows C++ 쪽과 1:1로 매칭되는 데이터 구조체 (바이트 정렬 필수)
#pragma pack(push, 1)
struct HandDataPacket {
    uint32_t frame;
    float wristPos[3];   // X, Y, Z (meters)
    float wristEuler[3]; // Roll, Pitch, Yaw (degrees)
    float fingerFlexion[15]; // 5 fingers x 3 joints (degrees)
};
#pragma pack(pop)

class ManusReceiverNode : public rclcpp::Node {
public:
    ManusReceiverNode() : Node("manus_receiver_cpp") {
        // ROS2 Publishers 설정
        wrist_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("manus/wrist_pose", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("manus/finger_joints", 10);

        // 2. UDP 소켓 초기화
        setup_udp();

        // 3. 수신 타이머 (10ms 주기 = 100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ManusReceiverNode::receive_callback, this));

        RCLCPP_INFO(this->get_logger(), "NREL MANUS C++ Receiver Node Started (Port: 12345)");
    }

    ~ManusReceiverNode() {
        if (sockfd_ != -1) close(sockfd_);
    }

private:
    void setup_udp() {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
            return;
        }

        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(12345);

        if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket bind failed!");
            return;
        }

        // 소켓을 Non-blocking으로 설정 (타이머 루프 방해 금지)
        fcntl(sockfd_, F_SETFL, O_NONBLOCK);
    }

    void receive_callback() {
        HandDataPacket packet;
        struct sockaddr_in cliaddr;
        socklen_t len = sizeof(cliaddr);

        // 데이터 수신 시도
        ssize_t n = recvfrom(sockfd_, &packet, sizeof(packet), 0, (struct sockaddr *)&cliaddr, &len);

        if (n == sizeof(HandDataPacket)) {
            // 수신 성공 시 데이터 발행
            publish_data(packet);
        } else if (n > 0) {
            RCLCPP_WARN(this->get_logger(), "Received packet size mismatch: %ld bytes", n);
        }
    }

    void publish_data(const HandDataPacket& packet) {
        auto now = this->get_clock()->now();

        // 1. Wrist PoseStamped 발행
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = packet.wristPos[0];
        pose_msg.pose.position.y = packet.wristPos[1];
        pose_msg.pose.position.z = packet.wristPos[2];

        // Euler (Degrees) -> Quaternion 변환
        set_quaternion_from_euler(pose_msg.pose.orientation, 
                                 packet.wristEuler[0], packet.wristEuler[1], packet.wristEuler[2]);
        wrist_pub_->publish(pose_msg);

        // 2. Finger JointState 발행 (Degree -> Radian 변환)
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = now;
        static const std::vector<std::string> joint_names = {
            "thumb_mcp", "thumb_pip", "thumb_dip", "index_mcp", "index_pip", "index_dip",
            "middle_mcp", "middle_pip", "middle_dip", "ring_mcp", "ring_pip", "ring_dip",
            "pinky_mcp", "pinky_pip", "pinky_dip"
        };
        joint_msg.name = joint_names;

        for (int i = 0; i < 15; ++i) {
            joint_msg.position.push_back(packet.fingerFlexion[i] * M_PI / 180.0);
        }
        joint_pub_->publish(joint_msg);
    }

    // Euler(Deg) -> Quaternion 헬퍼 함수
    void set_quaternion_from_euler(geometry_msgs::msg::Quaternion& q, float r, float p, float y) {
        float roll = r * M_PI / 180.0;
        float pitch = p * M_PI / 180.0;
        float yaw = y * M_PI / 180.0;

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
    }

    int sockfd_ = -1;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wrist_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManusReceiverNode>());
    rclcpp::shutdown();
    return 0;
}