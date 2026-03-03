#include "teleop_slave/fairino_slave_node.hpp"

FairinoSlaveNode::FairinoSlaveNode() : Node("fairino_slave_node"), streaming_started_(false) {
    this->declare_parameter("pose_offset_x", 0.0);
    this->declare_parameter("pose_offset_y", 0.0);
    this->declare_parameter("pose_offset_z", 0.0);
    this->declare_parameter("workspace_radius", 0.85);
    this->declare_parameter("min_z", 0.05);

    manus_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/manus/wrist_pose", 10,
        std::bind(&FairinoSlaveNode::manusPoseCallback, this, std::placeholders::_1));

    curobo_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/curobo/pose_target", 10);
        
    stream_client_ = this->create_client<std_srvs::srv::SetBool>("/enable_streaming");

    RCLCPP_INFO(this->get_logger(), "Fairino Slave Node Started");
}

void FairinoSlaveNode::manusPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!streaming_started_) {
        startStreaming();
    }

    auto out_msg = geometry_msgs::msg::PoseStamped();
    out_msg.header.stamp = this->now();
    out_msg.header.frame_id = "robot_base";
    
    double offset_x = this->get_parameter("pose_offset_x").as_double();
    double offset_y = this->get_parameter("pose_offset_y").as_double();
    double offset_z = this->get_parameter("pose_offset_z").as_double();
    double max_r = this->get_parameter("workspace_radius").as_double();
    double min_z = this->get_parameter("min_z").as_double();

    out_msg.pose.position.x = msg->pose.position.x + offset_x;
    out_msg.pose.position.y = msg->pose.position.y + offset_y;
    out_msg.pose.position.z = msg->pose.position.z + offset_z;
    
    out_msg.pose.orientation = msg->pose.orientation;

    if (out_msg.pose.position.z < min_z) {
         out_msg.pose.position.z = min_z;
    }
    
    double r = std::sqrt(out_msg.pose.position.x * out_msg.pose.position.x + 
                         out_msg.pose.position.y * out_msg.pose.position.y +
                         out_msg.pose.position.z * out_msg.pose.position.z);
    if (r > max_r) {
        double scale = max_r / r;
        out_msg.pose.position.x *= scale;
        out_msg.pose.position.y *= scale;
        out_msg.pose.position.z *= scale;
    }

    curobo_pub_->publish(out_msg);
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
                RCLCPP_INFO(this->get_logger(), "스트리밍 켜짐 성공.");
                streaming_started_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "스트리밍 실패: %s", response->message.c_str());
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
