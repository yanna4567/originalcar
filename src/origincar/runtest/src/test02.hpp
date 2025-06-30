#ifndef __TEST02_H__
#define __TEST02_H__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iomanip>

using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using std::placeholders::_1;
using namespace std::chrono_literals;

// 车辆当前状态定义
struct VehicleState {
    double x;
    double y;
    double yaw;
    double v;

    VehicleState(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0)
        : x(x), y(y), yaw(yaw), v(v) {}
};

class PathTracking
{
public:
    PathTracking(std::vector<double> pathx , std::vector<double> pathy);

    double Track(double x, double y, double yaw);

    VehicleState update(double x, double y, double yaw);
    inline int target_ind() { return m_target_ind;}
    double PControl(double target, double current);
    std::pair<int, double> calc_target_index(const std::vector<double>& cx, const std::vector<double>& cy);
    std::pair<double, int> pure_pursuit_control(const std::vector<double>& cx, const std::vector<double>& cy, int pind);
    double calcDistance(double point_x, double point_y); // 计算传入点跟当前点的距离 
private:
    std::vector<double> cx;
    std::vector<double> cy;

    VehicleState m_vehicleState;
    int m_target_ind;       // 预期跟随点

    // 纯追踪算法参数
    const double k = 0.1;             // 前视距离系数，与速度成正比
    const double Lfc = 0.6;           // 最小前视距离，单位：米
    const double Kp = 1.0;            // 速度P控制器比例系数
    const double dt = 0.01;            // 控制周期，单位：秒
    const double L = 0.14;            // 车辆轴距，单位：米
    int max_point_index = 0;          // 最大寻找数量
};


class testNode
    : public rclcpp::Node
{
public:
    testNode();
    ~testNode();
private:
    bool read_path_from_file(const std::string &file_path);
    void imu_callback(Odometry::ConstSharedPtr odomData);
private:
    void timer_callback();
private:
    rclcpp::Subscription<Odometry>::SharedPtr m_pImuSub; // imu的订阅
    rclcpp::Publisher<Twist>::SharedPtr m_pVelPublisher;    // 地盘控制发布

    double m_x;
    double m_y;
    double m_ekfYaw;
    std::vector<double> path_x_; // 新增：存储读取的x坐标
    std::vector<double> path_y_; // 新增：存储读取的y坐标
    PathTracking* m_pPathTracking;
    rclcpp::TimerBase::SharedPtr m_pTimer; // 控制定时器

};

#endif