#include "teleop_slave/fairino_slave_node.hpp"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

FairinoSlaveNode::FairinoSlaveNode() 
    : Node("fairino_slave_node"), streaming_started_(false), running_(true), offset_set_(false), robot_pose_received_(false) 
{
    // 안전 및 제한 파라미터만 선언
    this->declare_parameter("workspace_radius", 0.85);
    this->declare_parameter("min_z", 0.05);
    
    // 초기 오프셋 변수 초기화
    zero_offset_x_ = 0.0; zero_offset_y_ = 0.0; zero_offset_z_ = 0.0;
    target_q_inv_.setW(1.0); target_q_inv_.setX(0.0); target_q_inv_.setY(0.0); target_q_inv_.setZ(0.0);
    base_robot_q_.setW(1.0); // 로봇의 기준 회전값

    // 1. Manus 손 위치 구독
    manus_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/manus/wrist_pose", 10,
        std::bind(&FairinoSlaveNode::manusPoseCallback, this, std::placeholders::_1));

    // 2. 로봇 실제 위치 구독 (Dynamic Zeroing을 위해 추가)
    robot_actual_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            latest_robot_pose_ = *msg;
            robot_pose_received_ = true;
        });

    curobo_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/curobo/pose_target", 10);

    stream_client_ = this->create_client<std_srvs::srv::SetBool>("/enable_streaming");

    RCLCPP_INFO(this->get_logger(), "Dynamic Zeroing Mode: Press SPACE to sync with CURRENT robot pose.");

    kb_thread_ = std::thread(&FairinoSlaveNode::keyboardThread, this);
}

FairinoSlaveNode::~FairinoSlaveNode() {
    running_ = false;
    if (kb_thread_.joinable()) kb_thread_.join();
}

void FairinoSlaveNode::keyboardThread() {
    // (기존 터미널 설정 로직 동일...)
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (running_ && rclcpp::ok()) {
        char ch_buf = 0;
        if (read(STDIN_FILENO, &ch_buf, 1) > 0) {
            if (ch_buf == ' ') {
                if (!streaming_started_) {
                    offset_set_ = true; // 다음 콜백에서 현재 로봇 위치로 영점 잡기
                    startStreaming();
                } else {
                    stopStreaming();
                }
            } else if (ch_buf == 'q' || ch_buf == 27) {
                running_ = false;
                rclcpp::shutdown();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void FairinoSlaveNode::manusPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!streaming_started_ && !offset_set_) return;

    // 현재 손의 회전
    tf2::Quaternion current_manus_q(
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w
    );

    // [영점 조절 로직] 스페이스바를 누른 시점의 처리
    if (offset_set_) {
        if (!robot_pose_received_) {
            RCLCPP_WARN(this->get_logger(),
                "Robot pose not yet received on /robot_pose — is fairino_lowlevel_controller_node running?");
            offset_set_ = false;
            streaming_started_ = false;
            return;
        }

        // 1. 위치 오프셋: 로봇의 현재 위치와 손의 현재 위치 차이 계산
        zero_offset_x_ = latest_robot_pose_.pose.position.x - msg->pose.position.x;
        zero_offset_y_ = latest_robot_pose_.pose.position.y - msg->pose.position.y;
        zero_offset_z_ = latest_robot_pose_.pose.position.z - msg->pose.position.z;

        // 2. 회전 오프셋:
        // - base_robot_q_: 스페이스바 누를 때 로봇이 바라보던 방향을 기준으로 삼음
        // - target_q_inv_: 스페이스바 누를 때 내 손의 회전을 '영점(Identity)'으로 만듦
        tf2::fromMsg(latest_robot_pose_.pose.orientation, base_robot_q_);
        target_q_inv_ = current_manus_q.inverse();

        offset_set_ = false;
        RCLCPP_INFO(this->get_logger(), "Synced to Robot Pose: [%.3f, %.3f, %.3f]",
                    latest_robot_pose_.pose.position.x,
                    latest_robot_pose_.pose.position.y,
                    latest_robot_pose_.pose.position.z);
    }

    // [명령 생성 로직]
    auto out_msg = geometry_msgs::msg::PoseStamped();
    out_msg.header.stamp = this->now();
    out_msg.header.frame_id = "robot_base";
    
    // 1. 위치 계산: 손의 위치 + 계산된 오프셋
    out_msg.pose.position.x = msg->pose.position.x + zero_offset_x_;
    out_msg.pose.position.y = msg->pose.position.y + zero_offset_y_;
    out_msg.pose.position.z = msg->pose.position.z + zero_offset_z_;
    
    // 2. 회전 계산: 로봇의 기준 방향 * (현재 손 회전 * 초기 손 회전의 역행렬)
    tf2::Quaternion relative_manus_q = current_manus_q * target_q_inv_;
    tf2::Quaternion out_q = base_robot_q_ * relative_manus_q;
    out_q.normalize();

    out_msg.pose.orientation = tf2::toMsg(out_q);

    // 3. 안전 제한 (Workspace & Min Z) 적용
    applySafetyLimits(out_msg);

    if (streaming_started_) {
        curobo_pub_->publish(out_msg);
    }
}

void FairinoSlaveNode::applySafetyLimits(geometry_msgs::msg::PoseStamped& msg) {
    double max_r = this->get_parameter("workspace_radius").as_double();
    double min_z = this->get_parameter("min_z").as_double();

    if (msg.pose.position.z < min_z) msg.pose.position.z = min_z;
    
    double r = std::sqrt(std::pow(msg.pose.position.x, 2) + 
                         std::pow(msg.pose.position.y, 2) +
                         std::pow(msg.pose.position.z, 2));
    if (r > max_r) {
        double scale = max_r / r;
        msg.pose.position.x *= scale;
        msg.pose.position.y *= scale;
        msg.pose.position.z *= scale;
    }
}

void FairinoSlaveNode::startStreaming() {
    if (!stream_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "/enable_streaming 서비스 대기중...");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto result_future = stream_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "\U0001F7E2 스트리밍 시작 (제어 중)");
                streaming_started_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "스트리밍 실패: %s", response->message.c_str());
            }
        });
}

void FairinoSlaveNode::stopStreaming() {
    if (!stream_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "/enable_streaming 서비스 대기중...");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    auto result_future = stream_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "\U0001F534 스트리밍 종료 (대기 중)");
                streaming_started_ = false;
            } else {
                RCLCPP_ERROR(this->get_logger(), "스트리밍 종료 실패: %s", response->message.c_str());
            }
        });
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FairinoSlaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
