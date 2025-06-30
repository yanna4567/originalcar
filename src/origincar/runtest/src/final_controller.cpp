#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using geometry_msgs::msg::Twist;
using std::placeholders::_1;
using namespace std::chrono_literals;

class FinalController
    : public rclcpp::Node
{
    public:
        FinalController()
            :Node("FinalController")
            ,m_isObstacle(false)
        {
            // 初始化变量
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                m_trackTwist.linear.x = 0.0;
                m_trackTwist.angular.z = 0.0;
                m_obstacleTwist.linear.x = 0.0;
                m_obstacleTwist.angular.z = 0.0;
            }

            // 订阅 发布控制实现
            m_pTrackSub = this->create_subscription<Twist>("/path_track_vel", 10, std::bind(&FinalController::pathTrack_callback, this, _1));
            m_pObstacleSub = this->create_subscription<Twist>("/avoidObstacle", 10, std::bind(&FinalController::obstacle_callback, this, _1));
            m_pVelPublisher = this->create_publisher<Twist>("/cmd_vel",10);
            
            //创建循迹定时器
            m_pTimer = this->create_wall_timer(20ms, std::bind(&FinalController::timer_callback, this));
    
        }

    private:
        void pathTrack_callback(Twist::ConstSharedPtr Data)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            m_trackTwist.linear = Data->linear;
            m_trackTwist.angular = Data->angular;
        }
        void obstacle_callback(Twist::ConstSharedPtr Data)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            // 获取到 避障信息，进行避障
            m_obstacleTwist.linear = Data->linear;
            m_obstacleTwist.angular = Data->angular;
        
        }
        void timer_callback()
        {
            Twist m_cmdVel;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);

                if (m_obstacleTwist.linear.x > 250) // 底部Y阈值
                {
                    // 避障
                    RCLCPP_INFO(this->get_logger(), "obstacle : %f", m_obstacleTwist.angular.z);
                    m_cmdVel.linear.x = 0.5;
                    m_cmdVel.angular.z = 1.2 * 150 / m_obstacleTwist.angular.z+1;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "obstacle 0");

                    m_cmdVel = m_trackTwist;
                    m_cmdVel.linear.x = 0.6;
                }
            }

            m_pVelPublisher->publish(m_cmdVel);
        }

    private:
        rclcpp::Subscription<Twist>::SharedPtr m_pTrackSub; // 路径追踪订阅的订阅
        rclcpp::Subscription<Twist>::SharedPtr m_pObstacleSub; // 障碍的订阅
        rclcpp::Publisher<Twist>::SharedPtr m_pVelPublisher;    // 地盘控制发布
        rclcpp::TimerBase::SharedPtr m_pTimer; // 控制定时器
        std::mutex data_mutex_;

        Twist m_trackTwist;     //跟踪的路径
        Twist m_obstacleTwist;  //避障信息
        
        bool m_isObstacle;      // 是否需要避障
        int m_delay = 0;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FinalController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}