#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using ai_msgs::msg::PerceptionTargets;
using geometry_msgs::msg::Twist;

class AvoidObstacle
    : public rclcpp::Node
{
    public:
        AvoidObstacle()
            : Node("AvoidObstacle")
        {
                // 绑定避障消息的内容
            m_pYoloSub = this->create_subscription<PerceptionTargets>("/racing_obstacle_detection", 10, std::bind(&AvoidObstacle::yolo_callback, this, _1));
            m_pObstaclePublisher = this->create_publisher<Twist>("/avoidObstacle",10);

        }

    private: 
        rclcpp::Subscription<PerceptionTargets>::SharedPtr m_pYoloSub; // yolo识别结果的订阅
        rclcpp::Publisher<Twist>::SharedPtr m_pObstaclePublisher;    // 避障信息的发布

        void yolo_callback(PerceptionTargets::ConstSharedPtr targetsData)
        {
            ai_msgs::msg::Roi maxRoi; 
            maxRoi.rect.height = 0;
            maxRoi.rect.width = 0;
            maxRoi.rect.y_offset = 0;
            // 获取yolo所有的信息，查找障碍物相关的信息
            for (auto& target : targetsData->targets) 
            {
                // RCLCPP_INFO(this->get_logger(), "target id: %s",target.type.c_str());   // type 为classname
                if(!(target.type == "obstacle"))
                        continue;
                for (auto& roi : target.rois) {
                    // 记录roi数据  寻找底部y最大的roi
                    if((maxRoi.rect.y_offset+maxRoi.rect.height) <(roi.rect.y_offset+roi.rect.height))
                    // if((maxRoi.rect.height*maxRoi.rect.width) < ( roi.rect.height *roi.rect.width))
                        maxRoi = roi;
                }
            }

            // 判断maxRoi是否满足条件，如果满足则进行计算 ，发布 避障信息
            // RCLCPP_INFO(this->get_logger(), "height : %d",maxRoi.rect.height);
            if((maxRoi.rect.x_offset>50)&&(maxRoi.rect.x_offset<450)&&(maxRoi.rect.height > 170 || maxRoi.rect.width > 120))
            {
                // 计算偏离程度 在左侧为正 右侧为负
                int diff = (maxRoi.rect.x_offset + maxRoi.rect.width/2) - 320;
                
                if((-20 < diff ) && ( diff <0))
                    diff = -20;
                else if(diff >0 && diff < 20)
                    diff = 20;
                
                geometry_msgs::msg::Twist cmd_vel;
                // cmd_vel.linear.x = 0.7;
                // linear.x 发布底部y坐标， z发布偏移程度
                cmd_vel.linear.x = maxRoi.rect.y_offset + maxRoi.rect.height;
                cmd_vel.angular.z = diff;
                m_pObstaclePublisher->publish(cmd_vel); // 发送避障内容
            }
            else
            {
                // 没有障碍也返回 返回0值
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 1;
                m_pObstaclePublisher->publish(cmd_vel); // 发送避障内容
            }
        }
};


int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AvoidObstacle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}