#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iomanip>

using std::placeholders::_1;
using sensor_msgs::msg::Imu;
using nav_msgs::msg::Odometry;

class testNode
    : public rclcpp::Node
{
public:
    testNode()
        : Node("test01")
    {
        // 声明并获取参数
        std::string pkg_path = ament_index_cpp::get_package_share_directory("runtest");
        std::string config_path = pkg_path + "/config/record.txt";

        this->declare_parameter("record_distance", 0.05); // 默认每5cm记录一次

        output_file_ = config_path;
        record_distance_ = this->get_parameter("record_distance").as_double();

        m_pImuSub = this->create_subscription<Odometry>("/odom_combined", 10, std::bind(&testNode::imu_callback, this, _1));
        m_pSub = this->create_subscription<Imu>("/imu/data", 10, std::bind(&testNode::i_callback, this, _1));

        m_recordfile.open(output_file_, std::ios::out);
        if (!m_recordfile.is_open()) 
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件:");
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(), "成功打开文件");
        }

        RCLCPP_INFO(this->get_logger(),"init testNode");
    }

private:

/*  odom的路径记录，写入txt文件，记录路程。
    每10cm记录一个点，目前odom是m为单位,记录也为m
    传入参数： 
    x : odom x ，单位m
    y : odom y , 单位m
*/
void odom_path_record(double x,double y)
{
    static double last_x = 0;    //上一次的距离
    static double last_y = 0;    //上一次的距离

    double distance = std::hypot(x - last_x, y - last_y);

    if(distance >= record_distance_)  // 5cm打一个点
    {
        RCLCPP_INFO(this->get_logger(), "save data");

        m_recordfile << std::fixed << std::setprecision(2);
        m_recordfile << x << "," << y << "\n";
        
        // m_recordfile << std::defaultfloat; // 恢复默认设置
        
        // 刷新缓冲区（可选，确保数据写入磁盘）
        m_recordfile.flush();

        // 更新上一个记录点
        last_x = x;
        last_y = y;
    }
}

void imu_callback(Odometry::ConstSharedPtr odomData)
{
        // 这里是最终的odom的输出
        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = odomData->pose.pose.orientation.x;
        quat_msg.y = odomData->pose.pose.orientation.y;
        quat_msg.z = odomData->pose.pose.orientation.z;
        quat_msg.w = odomData->pose.pose.orientation.w;

        tf2::Quaternion tf_quat;
        tf_quat.setX(quat_msg.x);
        tf_quat.setY(quat_msg.y);
        tf_quat.setZ(quat_msg.z);
        tf_quat.setW(quat_msg.w);

        tf2::Matrix3x3 matrix(tf_quat);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        m_x = odomData->pose.pose.position.x;
        m_y = odomData->pose.pose.position.y;
        m_ekfYaw = yaw*180/M_PI;
        RCLCPP_INFO(this->get_logger(),"x: %.2f, y: %.2f, yaw: %.2f, imuyaw: %.2f",odomData->pose.pose.position.x,
                                                    odomData->pose.pose.position.y,yaw*180/M_PI,
                                                m_imuYaw);
        odom_path_record(m_x,m_y);

}

void i_callback(Imu::ConstSharedPtr imuData)
{
        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = imuData->orientation.x;
        quat_msg.y = imuData->orientation.y;
        quat_msg.z = imuData->orientation.z;
        quat_msg.w = imuData->orientation.w;

        tf2::Quaternion tf_quat;
        tf_quat.setX(quat_msg.x);
        tf_quat.setY(quat_msg.y);
        tf_quat.setZ(quat_msg.z);
        tf_quat.setW(quat_msg.w);

        tf2::Matrix3x3 matrix(tf_quat);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        m_imuYaw = yaw*180/M_PI;
        // RCLCPP_INFO(this->get_logger(),"yaw: %.2f",yaw*180/M_PI);

}

private:
    rclcpp::Subscription<Odometry>::SharedPtr m_pImuSub;    // imu的订阅
    rclcpp::Subscription<Imu>::SharedPtr m_pSub;    // imu的订阅
    double m_x;
    double m_y;
    double m_imuYaw;
    double m_ekfYaw;
    std::ofstream m_recordfile;
    std::string output_file_;
    double record_distance_;
};

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<testNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}