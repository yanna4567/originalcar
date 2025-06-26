#include "origincar_competition/origincar_competition.h"
#include <string>
#include <stdexcept> // For std::stoi exceptions
#include <cmath>     // For M_PI, sin, std::abs, std::min, std::max

CompleteControl::CompleteControl() : Node("complete_control_node"),
                                     is_teleoperation_active_(false),
                                     last_qr_code_command_(QR_NOTHING),
                                     current_behavior_state_(RobotBehaviorState::LINE_FOLLOWING),
                                     last_avoidance_turn_direction_(LastAvoidanceTurnDirection::NONE)
{
    // 参数声明
    this->declare_parameter<double>("line_following_speed", 0.45);
    this->declare_parameter<double>("line_kp", 1.0 / 320.0);
    this->declare_parameter<double>("cone_avoidance_speed", 0.25);
    this->declare_parameter<double>("cone_detection_y_threshold", 220.0);
    this->declare_parameter<double>("cone_critical_y_threshold", 300.0);
    this->declare_parameter<double>("cone_avoidance_steering_gain", 1.2);
    this->declare_parameter<double>("cone_lateral_offset_threshold", 40.0); // 用于判断锥桶是否在"正前方"区域
    this->declare_parameter<double>("centered_cone_avoid_turn_bias", 1.0);  // 正前方锥桶默认向左避让 (1.0 for left, -1.0 for right)
    this->declare_parameter<double>("post_avoidance_forward_search_duration", 0.1); // 避障后向前短时直行
    this->declare_parameter<double>("post_avoidance_recovery_turn_duration", 0.0); // 0.0 表示无限恢复转向
    this->declare_parameter<double>("recovery_turn_linear_speed_ratio", 0.35);
    this->declare_parameter<double>("search_swing_frequency", 0.3);

    // 获取参数
    this->get_parameter("line_following_speed", line_following_speed_);
    this->get_parameter("line_kp", line_kp_);
    this->get_parameter("cone_avoidance_speed", cone_avoidance_speed_);
    this->get_parameter("cone_detection_y_threshold", cone_detection_y_threshold_);
    this->get_parameter("cone_critical_y_threshold", cone_critical_y_threshold_);
    this->get_parameter("cone_avoidance_steering_gain", cone_avoidance_steering_gain_);
    this->get_parameter("cone_lateral_offset_threshold", cone_lateral_offset_threshold_);
    this->get_parameter("centered_cone_avoid_turn_bias", centered_cone_avoid_turn_bias_);
    this->get_parameter("post_avoidance_forward_search_duration", post_avoidance_forward_duration_sec_);
    this->get_parameter("post_avoidance_recovery_turn_duration", post_avoidance_recovery_turn_duration_sec_);
    this->get_parameter("recovery_turn_linear_speed_ratio", recovery_turn_linear_speed_ratio_);
    this->get_parameter("search_swing_frequency", search_swing_frequency_);

    RCLCPP_INFO(this->get_logger(), "CompleteControl Node started.");
    RCLCPP_INFO(this->get_logger(), "  Centered cone avoid bias: %.1f (1=Left, -1=Right)", centered_cone_avoid_turn_bias_);
    RCLCPP_INFO(this->get_logger(), "  Post-Avoidance Forward Search: %.2fs", post_avoidance_forward_duration_sec_);


    perception_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "origincar_competition", 10, std::bind(&CompleteControl::handleAiMsg, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    follower_line_publisher_ = this->create_publisher<std_msgs::msg::Int32>("follower_line", 5);
    stop_publisher_ = this->create_publisher<std_msgs::msg::Int32>("stop", 1);
    sign_to_upper_computer_pub_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);
    qr_code_raw_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/sign", 10, std::bind(&CompleteControl::qrCodeRawCallback, this, std::placeholders::_1));
    teleop_signal_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/sign4return", 10, std::bind(&CompleteControl::teleopSignalCallback, this, std::placeholders::_1));
}

void CompleteControl::qrCodeRawCallback(const std_msgs::msg::String::SharedPtr msg)
{
    origincar_msg::msg::Sign sign_to_publish;
    std::string data_str = msg->data;
    bool valid_qr_command_received = false;
    try {
        int number = std::stoi(data_str);
        if (number >= 1 && number <= 9999) {
            if (number % 2 != 0) {
                last_qr_code_command_ = QR_CLOCKWISE;
                sign_to_publish.sign_data = QR_CLOCKWISE;
            } else {
                last_qr_code_command_ = QR_ANTICLOCKWISE;
                sign_to_publish.sign_data = QR_ANTICLOCKWISE;
            }
            sign_to_upper_computer_pub_->publish(sign_to_publish);
            valid_qr_command_received = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "QR Number %d out of range.", number);
            last_qr_code_command_ = QR_NOTHING;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "QR data '%s' invalid: %s", data_str.c_str(), e.what());
        last_qr_code_command_ = QR_NOTHING;
    }

    if (valid_qr_command_received) {
        RCLCPP_INFO(this->get_logger(), "Valid QR command (%d) received. -> QR_CODE_WAITING (Stopping).", last_qr_code_command_);
        current_behavior_state_ = RobotBehaviorState::QR_CODE_WAITING;
        setSpeed(0.0, 0.0);
    }
}

void CompleteControl::teleopSignalCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (msg->data == SIG_START_TELEOPERATION) {
        RCLCPP_INFO(this->get_logger(), "Teleoperation STARTED.");
        is_teleoperation_active_ = true;
    } else if (msg->data == SIG_ENDING_TELEOPERATION) {
        RCLCPP_INFO(this->get_logger(), "Teleoperation ENDED. Attempting to resume LINE_FOLLOWING.");
        is_teleoperation_active_ = false;
        last_qr_code_command_ = QR_NOTHING;
        current_behavior_state_ = RobotBehaviorState::LINE_FOLLOWING;
    }
}

void CompleteControl::setSpeed(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_x;
    cmd_vel_msg.angular.z = angular_z;
    cmd_vel_publisher_->publish(cmd_vel_msg);
}

void CompleteControl::publishFollowerLineState(bool is_following) {
    std_msgs::msg::Int32 msg;
    msg.data = is_following ? 1 : 0;
    follower_line_publisher_->publish(msg);
}

void CompleteControl::publishHardStop() {
    std_msgs::msg::Int32 msg;
    msg.data = 0;
    stop_publisher_->publish(msg);
    RCLCPP_ERROR(this->get_logger(), "HARD STOP SIGNAL PUBLISHED!");
}

void CompleteControl::handleAiMsg(const ai_msgs::msg::PerceptionTargets::SharedPtr msg_ptr)
{
    // 0. 最高优先级检查
    if (last_qr_code_command_ != QR_NOTHING) {
        if (current_behavior_state_ != RobotBehaviorState::QR_CODE_WAITING) {
            current_behavior_state_ = RobotBehaviorState::QR_CODE_WAITING;
            setSpeed(0.0, 0.0);
        }
        publishFollowerLineState(false); return;
    }
    if (is_teleoperation_active_) {
        publishFollowerLineState(false); return;
    }

    // 收集感知数据
    std::vector<const ai_msgs::msg::Roi*> line_rois;
    std::vector<const ai_msgs::msg::Roi*> cone_rois;
    for (const auto &target : msg_ptr->targets) {
        if (!target.rois.empty()) {
            if (target.type == "line") line_rois.push_back(&target.rois[0]);
            else if (target.type == "zt") cone_rois.push_back(&target.rois[0]);
        }
    }

    double target_linear_x = 0.0;
    double target_angular_z = 0.0;
    bool is_cone_threat_detected_this_cycle = false;

    // 1. 锥桶威胁检测 (仅更新标志和状态，具体避障动作在状态机内)
    if (current_behavior_state_ != RobotBehaviorState::QR_CODE_WAITING && !is_teleoperation_active_) {
        if (!cone_rois.empty()) {
            float max_cone_height = 0.0f;
            for (const auto* cone_roi_ptr : cone_rois) {
                max_cone_height = std::max(max_cone_height, static_cast<float>(cone_roi_ptr->rect.height));
            }
            if (max_cone_height > cone_detection_y_threshold_) {
                is_cone_threat_detected_this_cycle = true;
                if (current_behavior_state_ != RobotBehaviorState::CONE_AVOIDANCE_ACTIVE) {
                    RCLCPP_INFO(this->get_logger(), "Cone threat (H:%.1f). State %d -> CONE_AVOIDANCE_ACTIVE", max_cone_height, static_cast<int>(current_behavior_state_));
                    current_behavior_state_ = RobotBehaviorState::CONE_AVOIDANCE_ACTIVE;
                    last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::NONE; // 进入时重置
                }
            }
        }
    }

    // 2. 核心状态机
    RobotBehaviorState next_behavior_state = current_behavior_state_;

    switch (current_behavior_state_)
    {
    case RobotBehaviorState::QR_CODE_WAITING:
        target_linear_x = 0.0; target_angular_z = 0.0; // 明确停车
        RCLCPP_DEBUG(this->get_logger(), "State: QR_CODE_WAITING");
        break;

    case RobotBehaviorState::LINE_FOLLOWING:
        // 暂时禁用AI巡线功能，保持停止状态
        target_linear_x = 0.0; target_angular_z = 0.0;
        RCLCPP_INFO(this->get_logger(), "AI巡线功能已禁用，使用CV巡线系统");
        break;

    case RobotBehaviorState::CONE_AVOIDANCE_ACTIVE:
        // 暂时禁用AI避障功能，保持停止状态
        target_linear_x = 0.0; target_angular_z = 0.0;
        RCLCPP_INFO(this->get_logger(), "AI避障功能已禁用");
        break;

    case RobotBehaviorState::POST_AVOIDANCE_SEARCH_FORWARD:
        // 暂时禁用AI搜索功能，保持停止状态
        target_linear_x = 0.0; target_angular_z = 0.0;
        RCLCPP_INFO(this->get_logger(), "AI搜索功能已禁用");
        break;

    case RobotBehaviorState::POST_AVOIDANCE_RECOVERY_TURN:
        // 暂时禁用AI恢复功能，保持停止状态
        target_linear_x = 0.0; target_angular_z = 0.0;
        RCLCPP_INFO(this->get_logger(), "AI恢复功能已禁用");
        break;
    }
    current_behavior_state_ = next_behavior_state;

    // 暂时禁用AI巡线状态发布，只发布停止状态
    publishFollowerLineState(false);
    setSpeed(target_linear_x, target_angular_z);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompleteControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}