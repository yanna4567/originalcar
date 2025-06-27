#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/time.hpp"
#include <chrono>

class PriorityController : public rclcpp::Node
{
public:
    PriorityController() : Node("priority_controller")
    {
        // 声明参数
        this->declare_parameter<int>("ai_priority", 1);           // AI系统优先级 (1=高, 0=低)
        this->declare_parameter<int>("cv_priority", 0);           // CV系统优先级 (1=高, 0=低)
        this->declare_parameter<double>("timeout_ai", 0.5);       // AI系统超时时间(秒)
        this->declare_parameter<double>("timeout_cv", 0.3);       // CV系统超时时间(秒)
        this->declare_parameter<double>("emergency_stop_timeout", 1.0); // 紧急停止超时时间
        
        // 获取参数
        this->get_parameter("ai_priority", ai_priority_);
        this->get_parameter("cv_priority", cv_priority_);
        this->get_parameter("timeout_ai", timeout_ai_);
        this->get_parameter("timeout_cv", timeout_cv_);
        this->get_parameter("emergency_stop_timeout", emergency_stop_timeout_);
        
        // 订阅AI系统的速度指令
        ai_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/ai_cmd_vel", 10, 
            std::bind(&PriorityController::ai_cmd_vel_callback, this, std::placeholders::_1));
        
        // 订阅CV系统的速度指令
        cv_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cv_cmd_vel", 10, 
            std::bind(&PriorityController::cv_cmd_vel_callback, this, std::placeholders::_1));
        
        // 发布最终的速度指令
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 发布系统状态
        system_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/system_status", 10);
        
        // 初始化时间戳
        last_ai_time_ = this->now();
        last_cv_time_ = this->now();
        
        // 创建定时器，定期检查和发布速度指令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&PriorityController::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Priority Controller started");
        RCLCPP_INFO(this->get_logger(), "  AI Priority: %d, CV Priority: %d", ai_priority_, cv_priority_);
        RCLCPP_INFO(this->get_logger(), "  AI Timeout: %.2fs, CV Timeout: %.2fs", timeout_ai_, timeout_cv_);
    }

private:
    void ai_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_ai_cmd_vel_ = *msg;
        last_ai_time_ = this->now();
        RCLCPP_DEBUG(this->get_logger(), "Received AI command: linear=%.2f, angular=%.2f", 
                    msg->linear.x, msg->angular.z);
    }
    
    void cv_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cv_cmd_vel_ = *msg;
        last_cv_time_ = this->now();
        RCLCPP_DEBUG(this->get_logger(), "Received CV command: linear=%.2f, angular=%.2f", 
                    msg->linear.x, msg->angular.z);
    }
    
    void timer_callback()
    {
        auto current_time = this->now();
        bool ai_active = (current_time - last_ai_time_).seconds() < timeout_ai_;
        bool cv_active = (current_time - last_cv_time_).seconds() < timeout_cv_;
        
        geometry_msgs::msg::Twist final_cmd_vel;
        std_msgs::msg::Int32 status_msg;
        
        // 优先级决策逻辑
        if (ai_priority_ > cv_priority_) {
            // AI优先级更高
            if (ai_active) {
                final_cmd_vel = last_ai_cmd_vel_;
                status_msg.data = 1; // AI控制
                RCLCPP_DEBUG(this->get_logger(), "Using AI command (higher priority)");
            } else if (cv_active) {
                final_cmd_vel = last_cv_cmd_vel_;
                status_msg.data = 2; // CV控制
                RCLCPP_DEBUG(this->get_logger(), "Using CV command (AI timeout)");
            } else {
                // 两个系统都超时，紧急停止
                final_cmd_vel.linear.x = 0.0;
                final_cmd_vel.angular.z = 0.0;
                status_msg.data = 0; // 停止
                RCLCPP_WARN(this->get_logger(), "Both systems timeout, emergency stop");
            }
        } else {
            // CV优先级更高
            if (cv_active) {
                final_cmd_vel = last_cv_cmd_vel_;
                status_msg.data = 2; // CV控制
                RCLCPP_DEBUG(this->get_logger(), "Using CV command (higher priority)");
            } else if (ai_active) {
                final_cmd_vel = last_ai_cmd_vel_;
                status_msg.data = 1; // AI控制
                RCLCPP_DEBUG(this->get_logger(), "Using AI command (CV timeout)");
            } else {
                // 两个系统都超时，紧急停止
                final_cmd_vel.linear.x = 0.0;
                final_cmd_vel.angular.z = 0.0;
                status_msg.data = 0; // 停止
                RCLCPP_WARN(this->get_logger(), "Both systems timeout, emergency stop");
            }
        }
        
        // 发布最终速度指令和状态
        cmd_vel_pub_->publish(final_cmd_vel);
        system_status_pub_->publish(status_msg);
    }
    
    // 参数
    int ai_priority_;
    int cv_priority_;
    double timeout_ai_;
    double timeout_cv_;
    double emergency_stop_timeout_;
    
    // 状态变量
    geometry_msgs::msg::Twist last_ai_cmd_vel_;
    geometry_msgs::msg::Twist last_cv_cmd_vel_;
    rclcpp::Time last_ai_time_;
    rclcpp::Time last_cv_time_;
    
    // ROS接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ai_cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cv_cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr system_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PriorityController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 