#include "teleop_slave/fairino_lowlevel_controller_node.hpp"
#include <chrono>
#include <algorithm>
#include <csignal>

using namespace std::chrono_literals;

FairinoControllerNode::FairinoControllerNode()
    : Node("fairino_lowlevel_controller_node")
{
    this->declare_parameter("robot_ip", "192.168.58.2");    
    robot_ip_ = this->get_parameter("robot_ip").as_string();

    this->declare_parameter("dummy_mode", false);
    dummy_mode_ = this->get_parameter("dummy_mode").as_bool();

    RCLCPP_INFO(this->get_logger(), "Fairino Controller Node");
    RCLCPP_INFO(this->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Dummy Mode: %s", dummy_mode_ ? "TRUE (Simulation)" : "FALSE (Hardware)");

    traj_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/trajectory_points", 10,
        std::bind(&FairinoControllerNode::trajectoryCallback, this, std::placeholders::_1));

    std::string joint_topic = dummy_mode_ ? "/joint_states" : "/robot_joint_states";
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_topic, 10);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/robot_pose", 10);

    execute_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/execute_trajectory",
        std::bind(&FairinoControllerNode::executeTrajectoryService, this,
                  std::placeholders::_1, std::placeholders::_2));

    stream_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo_target", 10,
        std::bind(&FairinoControllerNode::streamCallback, this, std::placeholders::_1));

    stream_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/enable_streaming",
        std::bind(&FairinoControllerNode::streamService, this,
                  std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = this->create_wall_timer(
        20ms, std::bind(&FairinoControllerNode::publishJointStates, this));
}

FairinoControllerNode::~FairinoControllerNode()
{
    shutdown();
}

bool FairinoControllerNode::initialize()
{
    return connectRobot();
}

void FairinoControllerNode::shutdown()
{
    if (shutdown_done_) return;
    shutdown_done_ = true;

    RCLCPP_INFO(this->get_logger(), "안전 종료 시작...");

    executing_ = false;
    stream_mode_ = false;
    std::this_thread::sleep_for(100ms);

    disconnectRobot();

    RCLCPP_INFO(this->get_logger(), "안전 종료 완료");
}

bool FairinoControllerNode::connectRobot()
{
    if (dummy_mode_) {
        RCLCPP_INFO(this->get_logger(), "Dummy Mode: 로봇 연결 우회됨 (Simulation)");
        dummy_joint_positions_.resize(NUM_JOINTS, 0.0);
        connected_ = true;
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "로봇 연결 중... (%s)", robot_ip_.c_str());

    robot_ = std::make_unique<FRRobot>();
    int ret = robot_->RPC(robot_ip_.c_str());
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(),
            "로봇 연결 실패 (IP: %s, Error: %d)", robot_ip_.c_str(), ret);
        robot_.reset();
        return false;
    }

    robot_->SetReConnectParam(true, 30000, 2000);
    robot_->ResetAllError();
    robot_->Mode(0);
    robot_->RobotEnable(1);
    robot_->SetSpeed(20);

    connected_ = true;
    RCLCPP_INFO(this->get_logger(), "로봇 연결 성공: %s", robot_ip_.c_str());
    return true;
}

void FairinoControllerNode::disconnectRobot()
{
    if (!connected_) return;

    if (dummy_mode_) {
        connected_ = false;
        RCLCPP_INFO(this->get_logger(), "Dummy Mode: 로봇 연결 해제됨");
        return;
    }

    if (!robot_) return;

    RCLCPP_INFO(this->get_logger(), "로봇 연결 해제 중...");

    robot_->ServoMoveEnd();
    robot_->RobotEnable(0);
    robot_->CloseRPC();

    connected_ = false;
    RCLCPP_INFO(this->get_logger(), "로봇 연결 해제 완료");
}

// ============== Trajectory Reception ==============

void FairinoControllerNode::trajectoryCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (executing_ || stream_mode_) {
        RCLCPP_WARN(this->get_logger(), "실행 중이거나 스트리밍 중에는 궤적 수신을 무시합니다.");
        return;
    }

    std::lock_guard<std::mutex> lock(traj_mutex_);
    trajectory_queue_.clear();

    size_t n_points = msg->data.size() / NUM_JOINTS;

    if (msg->data.size() % NUM_JOINTS != 0 || n_points == 0) {
        RCLCPP_ERROR(this->get_logger(),
            "잘못된 궤적 데이터 크기: %zu (6의 배수여야 함)", msg->data.size());
        return;
    }

    for (size_t i = 0; i < n_points; ++i) {
        std::vector<double> point(NUM_JOINTS);
        for (int j = 0; j < NUM_JOINTS; ++j) {
            point[j] = msg->data[i * NUM_JOINTS + j];
        }
        trajectory_queue_.push_back(point);
    }

    trajectory_loaded_ = true;
    current_traj_idx_ = 0;

    RCLCPP_INFO(this->get_logger(), "궤적 수신: %zu 포인트 (%.1f초 @ 250Hz)",
        n_points, n_points / CONTROL_FREQUENCY_HZ);
}

// ============== Trajectory Execution ==============

void FairinoControllerNode::executeTrajectoryService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!connected_) {
        response->success = false;
        response->message = "로봇이 연결되지 않았습니다.";
        return;
    }

    if (!trajectory_loaded_ || trajectory_queue_.empty()) {
        response->success = false;
        response->message = "궤적이 로드되지 않았습니다.";
        return;
    }

    if (executing_ || stream_mode_) {
        response->success = false;
        response->message = "이미 실행 중이거나 스트리밍 중입니다.";
        return;
    }

    executing_ = true;
    current_traj_idx_ = 0;
    servo_error_count_.store(0);

    size_t n_points = trajectory_queue_.size();
    RCLCPP_INFO(this->get_logger(), "궤적 실행 시작: %zu 포인트", n_points);

    robot_->ResetAllError();
    std::this_thread::sleep_for(20ms);
    robot_->Mode(0);
    std::this_thread::sleep_for(20ms);

    robot_->ServoMoveStart();
    std::this_thread::sleep_for(50ms);

    std::thread([this, n_points]() {
        controlLoop();

        // 진단: 루프 종료 직후 실제 위치 vs 목표 비교
        auto& final_pt = trajectory_queue_.back();  // rad
        std::vector<double> final_deg(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; ++i)
            final_deg[i] = final_pt[i] * RAD_TO_DEG;

        JointPos actual_after;
        if (!dummy_mode_) {
            robot_->GetActualJointPosDegree(0, &actual_after);
        } else {
            for(int i=0; i<NUM_JOINTS; ++i) actual_after.jPos[i] = dummy_joint_positions_[i];
        }

        double loop_max_err = 0;
        for (int i = 0; i < NUM_JOINTS; ++i)
            loop_max_err = std::max(loop_max_err,
                std::abs(final_deg[i] - actual_after.jPos[i]));

        RCLCPP_INFO(this->get_logger(),
            "루프 종료 | 목표(deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
            final_deg[0], final_deg[1], final_deg[2],
            final_deg[3], final_deg[4], final_deg[5]);
        RCLCPP_INFO(this->get_logger(),
            "루프 종료 | 실제(deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] (max_err=%.2f)",
            actual_after.jPos[0], actual_after.jPos[1], actual_after.jPos[2],
            actual_after.jPos[3], actual_after.jPos[4], actual_after.jPos[5],
            loop_max_err);

        // 수렴 대기: 로봇이 최종 목표에 도달할 때까지 ServoJ 계속 전송
        constexpr double CONVERGE_THRESHOLD_DEG = 0.5;
        constexpr int    MAX_WAIT_MS = 5000;
        auto wait_start = std::chrono::steady_clock::now();

        while (true) {
            JointPos actual;
            if (!dummy_mode_) {
                robot_->GetActualJointPosDegree(0, &actual);
            } else {
                for(int i=0; i<NUM_JOINTS; ++i) actual.jPos[i] = dummy_joint_positions_[i];
            }

            double max_err = 0;
            for (int i = 0; i < NUM_JOINTS; ++i)
                max_err = std::max(max_err,
                    std::abs(final_deg[i] - actual.jPos[i]));

            if (max_err < CONVERGE_THRESHOLD_DEG) {
                RCLCPP_INFO(this->get_logger(),
                    "수렴 완료: max_err=%.2f deg", max_err);
                break;
            }

            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - wait_start).count();

            if (elapsed_ms > MAX_WAIT_MS) {
                RCLCPP_WARN(this->get_logger(),
                    "수렴 타임아웃 (%.2f deg, %lld ms)", max_err, (long long)elapsed_ms);
                break;
            }

            executeServoJ(final_deg);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }

        robot_->ServoMoveEnd();

        int errs = servo_error_count_.load();
        if (errs > 0) {
            RCLCPP_WARN(this->get_logger(),
                "실행 중 ServoJ 오류: %d / %zu 포인트", errs, n_points);
        }

        executing_ = false;
        RCLCPP_INFO(this->get_logger(), "궤적 실행 완료");
    }).detach();

    response->success = true;
    response->message = "궤적 실행 시작됨";
}

void FairinoControllerNode::controlLoop()
{
    auto period = std::chrono::duration<double, std::milli>(CONTROL_PERIOD_MS);

    while (executing_ && current_traj_idx_ < trajectory_queue_.size()) {
        auto start_time = std::chrono::steady_clock::now();

        std::vector<double> point;
        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            point = trajectory_queue_[current_traj_idx_];
        }

        std::vector<double> joints_deg(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joints_deg[i] = point[i] * RAD_TO_DEG;
        }

        executeServoJ(joints_deg);
        current_traj_idx_++;

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

bool FairinoControllerNode::executeServoJ(const std::vector<double>& target_deg)
{
    if (target_deg.size() != NUM_JOINTS) return false;

    if (dummy_mode_) {
        // Dummy execution immediately sets current state to target
        for (int i = 0; i < NUM_JOINTS; ++i) {
            dummy_joint_positions_[i] = target_deg[i];
        }
        return true;
    }

    if (!robot_) return false;

    JointPos current_actual;
    robot_->GetActualJointPosDegree(0, &current_actual);

    static constexpr double SERVOJ_MAX_STEP_DEG = 2.0;

    JointPos cmd;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        double error = target_deg[i] - current_actual.jPos[i];
        double step = std::clamp(error, -SERVOJ_MAX_STEP_DEG, SERVOJ_MAX_STEP_DEG);
        cmd.jPos[i] = current_actual.jPos[i] + step;
    }

    ExaxisPos epos(0, 0, 0, 0);
    float t = static_cast<float>(CONTROL_PERIOD_MS / 1000.0);

    errno_t ret = robot_->ServoJ(&cmd, &epos, 0.0f, 0.0f, t, 0.0f, 0.0f, 0);

    if (ret != 0) {
        int err_cnt = servo_error_count_.fetch_add(1) + 1;

        if (err_cnt <= 3 || err_cnt % 100 == 0) {
            int maincode = 0, subcode = 0;
            robot_->GetRobotErrorCode(&maincode, &subcode);
            RCLCPP_WARN(this->get_logger(),
                "ServoJ 오류 (cnt=%d): ret=%d, main=%d, sub=%d",
                err_cnt, ret, maincode, subcode);
        }

        robot_->ResetAllError();
        return false;
    }

    return true;
}

// ============== Real-time Streaming ==============

void FairinoControllerNode::streamCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!stream_mode_ || msg->position.size() != NUM_JOINTS) return;

    std::lock_guard<std::mutex> lock(stream_mutex_);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        stream_target_deg_[i] = msg->position[i] * RAD_TO_DEG;
    }
}

void FairinoControllerNode::streamService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (!connected_) {
        response->success = false;
        response->message = "로봇이 연결되지 않았습니다.";
        return;
    }

    if (request->data) {
        if (executing_) {
            response->success = false;
            response->message = "궤적 실행 중입니다.";
            return;
        }

        if (!stream_mode_) {
            if (!dummy_mode_) {
                robot_->ResetAllError();
                std::this_thread::sleep_for(20ms);
                robot_->Mode(0);
                std::this_thread::sleep_for(20ms);
                robot_->ServoMoveStart();
                std::this_thread::sleep_for(50ms);
            }

            JointPos actual;
            if (!dummy_mode_) {
                robot_->GetActualJointPosDegree(0, &actual);
            } else {
                for(int i=0; i<NUM_JOINTS; ++i) actual.jPos[i] = dummy_joint_positions_[i];
            }
            {
                std::lock_guard<std::mutex> lock(stream_mutex_);
                stream_target_deg_.resize(NUM_JOINTS);
                for (int i = 0; i < NUM_JOINTS; ++i) {
                    stream_target_deg_[i] = actual.jPos[i];
                }
            }

            stream_mode_ = true;
            servo_error_count_.store(0);
            std::thread(&FairinoControllerNode::streamLoop, this).detach();
            RCLCPP_INFO(this->get_logger(), "스트리밍 모드 시작됨");
        }
        response->success = true;
        response->message = "스트리밍 시작됨";
    } else {
        if (stream_mode_) {
            stream_mode_ = false;
            // The streamLoop will auto-end ServoMoveEnd when it exits the while loop
            RCLCPP_INFO(this->get_logger(), "스트리밍 모드 중지 예약됨");
        }
        response->success = true;
        response->message = "스트리밍 중지됨";
    }
}

void FairinoControllerNode::streamLoop()
{
    auto period = std::chrono::duration<double, std::milli>(CONTROL_PERIOD_MS);

    while (stream_mode_) {
        auto start_time = std::chrono::steady_clock::now();

        std::vector<double> target;
        {
            std::lock_guard<std::mutex> lock(stream_mutex_);
            target = stream_target_deg_;
        }

        if(!executeServoJ(target)) {
            RCLCPP_ERROR(this->get_logger(), "스트리밍 중 ServoJ 전송 실패. 스트리밍 종료.");
            stream_mode_ = false;
            break;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    robot_->ServoMoveEnd();
}

// ============== Joint State Publishing (50Hz) ==============

void FairinoControllerNode::publishJointStates()
{
    if (!connected_) return;

    // ==========================================
    // 1. Joint State (관절 각도) 가져오기 및 발행
    // ==========================================
    JointPos jpos;
    if (!dummy_mode_) {
        if (!robot_) return;
        errno_t ret = robot_->GetActualJointPosDegree(0, &jpos);
        if (ret != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "GetActualJointPosDegree 실패: %d", ret);
            return;
        }
    } else {
        for(int i=0; i<NUM_JOINTS; ++i) jpos.jPos[i] = dummy_joint_positions_[i];
    }

    if (!dummy_mode_) {
        bool all_zero = true;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            if (std::abs(jpos.jPos[i]) > 0.001) {
                all_zero = false;
                break;
            }
        }

        if (!all_zero) {
            last_valid_joints_ = jpos;
            has_valid_joints_ = true;
        } else if (has_valid_joints_) {
            jpos = last_valid_joints_;
        } else {
            return; // 유효한 데이터가 없으면 중단
        }
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.resize(NUM_JOINTS);

    for (int i = 0; i < NUM_JOINTS; ++i) {
        msg.position[i] = jpos.jPos[i] * DEG_TO_RAD; // Degree to Radian
    }

    joint_pub_->publish(msg);


    // ==========================================
    // 2. [추가됨] TCP Pose (말단 장치 위치) 가져오기 및 발행
    // ==========================================
    if (!dummy_mode_ && robot_ && pose_pub_) {
        DescPose tcp_pose;
        // 0: 차단 모드 (또는 1: 비차단 모드)로 현재 TCP 위치 요청
        errno_t ret_pose = robot_->GetActualTCPPose(0, &tcp_pose); 
        
        if (ret_pose == 0) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "base_link"; // fr5.yml의 base_link 이름과 맞춤

            // 위치 변환: Fairino SDK는 mm 단위이므로 m 단위로 변환
            pose_msg.pose.position.x = tcp_pose.tran.x / 1000.0;
            pose_msg.pose.position.y = tcp_pose.tran.y / 1000.0;
            pose_msg.pose.position.z = tcp_pose.tran.z / 1000.0;

            // 회전 변환: Euler Degree -> Quaternion
            tf2::Quaternion q;
            q.setRPY(
                tcp_pose.rpy.rx * M_PI / 180.0, 
                tcp_pose.rpy.ry * M_PI / 180.0, 
                tcp_pose.rpy.rz * M_PI / 180.0
            );

            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            pose_pub_->publish(pose_msg);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "GetActualTCPPose 실패: %d", ret_pose);
        }
    }
}

// ============== Main ==============

static std::shared_ptr<FairinoControllerNode> g_node = nullptr;

void signal_handler(int signum)
{
    (void)signum;
    if (g_node) {
        g_node->shutdown();
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    g_node = std::make_shared<FairinoControllerNode>();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    if (!g_node->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "로봇 초기화 실패");
        g_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Fairino Controller 시작");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  /trajectory_points -> /execute_trajectory");

    try {
        rclcpp::spin(g_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "예외: %s", e.what());
    }

    if (g_node) {
        g_node->shutdown();
        g_node.reset();
    }
    rclcpp::shutdown();
    return 0;
}
