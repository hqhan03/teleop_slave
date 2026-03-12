#ifndef FAIRINO_SLAVE_NODE_HPP_
#define FAIRINO_SLAVE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h> // read() 함수를 위해 추가

class FairinoSlaveNode : public rclcpp::Node {
public:
    FairinoSlaveNode();
    ~FairinoSlaveNode();

private:
    // 콜백 및 주요 함수
    void manusPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void startStreaming();
    void stopStreaming();
    void keyboardThread();
    
    // [추가] 안전 제한 적용 함수
    void applySafetyLimits(geometry_msgs::msg::PoseStamped& msg);

    // ROS 인터페이스
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manus_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_actual_sub_; // [추가] 로봇 위치 구독
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr curobo_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stream_client_;

    // 상태 제어 변수 (atomic으로 안전하게 관리)
    std::atomic<bool> streaming_started_;
    std::atomic<bool> running_;
    std::thread kb_thread_;

    // [추가] 동적 영점 조절용 변수
    bool offset_set_;
    bool robot_pose_received_; // /robot_pose 수신 여부
    geometry_msgs::msg::PoseStamped latest_robot_pose_; // 현재 로봇의 실제 위치 저장용
    
    // 계산된 실시간 오프셋 값
    double zero_offset_x_;
    double zero_offset_y_;
    double zero_offset_z_;
    
    tf2::Quaternion target_q_inv_; // 손의 초기 회전 역행렬
    tf2::Quaternion base_robot_q_; // [추가] 로봇의 기준 회전값 저장

    // YAML 파라미터 (동적 영점 조절 미사용 시 또는 기본값으로 활용)
    double param_offset_x_;
    double param_offset_y_;
    double param_offset_z_;
    double param_offset_qx_;
    double param_offset_qy_;
    double param_offset_qz_;
    double param_offset_qw_;
};

#endif // FAIRINO_SLAVE_NODE_HPP_