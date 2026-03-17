#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cstdio>
#include <cmath>

class FairinoStatePrinterNode : public rclcpp::Node
{
public:
    FairinoStatePrinterNode()
        : Node("fairino_state_printer_node")
    {
        this->declare_parameter("joint_topic", "/robot_joint_states");
        this->declare_parameter("pose_topic", "/robot_pose");

        std::string joint_topic = this->get_parameter("joint_topic").as_string();
        std::string pose_topic  = this->get_parameter("pose_topic").as_string();

        RCLCPP_INFO(this->get_logger(),
            "Subscribing to joint topic : %s", joint_topic.c_str());
        RCLCPP_INFO(this->get_logger(),
            "Subscribing to pose topic  : %s", pose_topic.c_str());

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_topic, 10,
            std::bind(&FairinoStatePrinterNode::jointCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10,
            std::bind(&FairinoStatePrinterNode::poseCallback, this, std::placeholders::_1));

        print_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FairinoStatePrinterNode::printState, this));
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_msg_ = msg;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_msg_ = msg;
    }

    void printState()
    {
        printf("\033[2J\033[H"); // Clear screen and move cursor to home
        printf("================ Fairino Robot State ================\n");

        // ---- Joint Angles ----
        if (joint_msg_) {
            const auto& pos = joint_msg_->position;
            size_t n = pos.size();
            printf("Joint Angles (deg): [");
            for (size_t i = 0; i < n; ++i) {
                double deg = pos[i] * 180.0 / M_PI;
                if (i > 0) printf(", ");
                printf("J%zu: %7.2f", i + 1, deg);
            }
            printf("]\n");
        } else {
            printf("Joint Angles (deg): -- waiting for /robot_joint_states --\n");
        }

        // ---- TCP Pose ----
        if (pose_msg_) {
            const auto& p = pose_msg_->pose.position;
            const auto& q = pose_msg_->pose.orientation;

            // Convert quaternion to RPY (degrees)
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            printf("End Effector TCP  :\n");
            printf("  Position (m)    : [x: %8.4f, y: %8.4f, z: %8.4f]\n",
                   p.x, p.y, p.z);
            printf("  Rotation (deg)  : [rx: %7.2f, ry: %7.2f, rz: %7.2f]\n",
                   roll  * 180.0 / M_PI,
                   pitch * 180.0 / M_PI,
                   yaw   * 180.0 / M_PI);
        } else {
            printf("End Effector TCP  : -- waiting for /robot_pose --\n");
        }

        printf("=====================================================\n");
        fflush(stdout);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr print_timer_;

    sensor_msgs::msg::JointState::SharedPtr   joint_msg_;
    geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FairinoStatePrinterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
