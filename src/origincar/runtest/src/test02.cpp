#include "test02.hpp"

testNode::testNode()
    : Node("test02")
    , m_pPathTracking(nullptr)
{
    // 声明并获取参数
    std::string pkg_path = ament_index_cpp::get_package_share_directory("runtest");
    std::string config_path = pkg_path + "/config/record.txt";

    m_pImuSub = this->create_subscription<Odometry>("/odom_combined", 10, std::bind(&testNode::imu_callback, this, _1));

    m_pVelPublisher = this->create_publisher<Twist>("/path_track_vel",10);

    read_path_from_file(config_path);

    // 新建跟踪类， 传递数据信息
    m_pPathTracking = new PathTracking(path_x_,path_y_);

    //创建循迹定时器
    m_pTimer = this->create_wall_timer(10ms, std::bind(&testNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "init testNode");
}

testNode::~testNode()
{
    if(m_pPathTracking)
    {
        delete m_pPathTracking;
        m_pPathTracking = nullptr;
    }
}

// 读取路径参数 
bool testNode::read_path_from_file(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", file_path.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "开始读取路径文件: %s", file_path.c_str());

    std::string line;

    int point_count = 0;
    while (std::getline(file, line))
    {
        if (line.empty())
            continue; // 跳过空行

        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;

        // 按逗号分割字符串
        while (std::getline(iss, token, ','))
        {
            tokens.push_back(token);
        }

        // 解析x和y坐标
        if (tokens.size() >= 2)
        {
            try
            {
                double x = std::stod(tokens[0]);
                double y = std::stod(tokens[1]);
                path_x_.push_back(x);
                path_y_.push_back(y);
                point_count++;
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "解析行失败: %s, 错误: %s", line.c_str(), e.what());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "格式不正确的行: %s", line.c_str());
        }
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "成功读取 %d 个路径点", point_count);
    return true;
}

void testNode::imu_callback(Odometry::ConstSharedPtr odomData)
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
    m_ekfYaw = yaw ;//* 180 / M_PI;
    // RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f, yaw: %.2f", odomData->pose.pose.position.x,
    //             odomData->pose.pose.position.y, m_ekfYaw);
}

void testNode::timer_callback()
{
    double di = m_pPathTracking->Track(m_x, m_y, m_ekfYaw);
    int target = m_pPathTracking->target_ind();
    // 创建并发布控制命令
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 1.2;    // 线速度
    if(abs(di) > 0.15)
        cmd_vel.linear.x = 0.8;
    double z= di*8;
    cmd_vel.angular.z = z;        // 弧度
    m_pVelPublisher->publish(cmd_vel);

    // RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f, yaw: %.2f, targeYaw: %.2f,target_ind: %d",m_x,
    //              m_y, m_ekfYaw,di,target);

}

PathTracking::PathTracking(std::vector<double> pathx, std::vector<double> pathy)
{
    cx = pathx;
    cy = pathy;

    max_point_index = pathx.size()-1;

    m_target_ind = -1;

    // m_target_ind = calc_target_index(cx, cy);

}

VehicleState PathTracking::update(double x, double y, double yaw)
{
    double last_x = m_vehicleState.x;
    double last_y = m_vehicleState.y;
    
    double distance = std::hypot(x - last_x, y - last_y); // 距离
    double v = distance / dt;
    
    m_vehicleState.x = x;
    m_vehicleState.y = y;
    m_vehicleState.yaw = yaw;
    m_vehicleState.v = v;

    return m_vehicleState;
    // state.x = state.x + state.v * std::cos(state.yaw) * dt;
    // state.y = state.y + state.v * std::sin(state.yaw) * dt;
    // state.yaw = state.yaw + state.v / L * std::tan(delta) * dt;
    // state.v = state.v + a * dt;
    // return state;
}

double PathTracking::calcDistance(double point_x, double point_y)
{
    double dx = m_vehicleState.x - point_x;
    double dy = m_vehicleState.y - point_y;
    return std::hypot(dx, dy);
}

double PathTracking::PControl(double target, double current)
{
    return Kp * (target - current);
}

/**
 * @brief 计算目标路径点索引
 * 寻找路径上距离车辆当前位置最近的点，并沿路径向前找到适当的前视距离点
 * @param state 当前车辆状态
 * @param cx 路径点X坐标数组
 * @param cy 路径点Y坐标数组
 * @return 目标路径点索引
 */
std::pair<int, double> PathTracking::calc_target_index(const std::vector<double> &cx, const std::vector<double> &cy)
{
    // 当前目标点 可以理解为旧点
    if (m_target_ind == -1)
    {
        std::vector<double> dx(cx.size()), dy(cy.size());
        for (size_t i = 0; i < cx.size(); ++i) {
            dx[i] = m_vehicleState.x - cx[i];
            dy[i] = m_vehicleState.y - cy[i];
        }
        std::vector<double> d(dx.size());
        std::transform(dx.begin(), dx.end(), dy.begin(), d.begin(), [](double dx, double dy)
         { return std::hypot(dx, dy); });
        auto it = std::min_element(d.begin(), d.end());
        m_target_ind = std::distance(d.begin(), it);
    }
    else
    {
        while (true)
        {
            double distanceThisIndex = calcDistance(cx[m_target_ind], cy[m_target_ind]);
            double distanceNextIndex = calcDistance(cx[m_target_ind + 1], cy[m_target_ind + 1]);
            if (distanceThisIndex < distanceNextIndex) {
                break;
            }
            m_target_ind++;
            if (m_target_ind >= static_cast<int>(cx.size()) - 1) {
                break;
            }
        }
    } 

    // 根据前视距离确定目标点v  
    double Lf = k * m_vehicleState.v + Lfc; // 动态前视距离
    // 沿路径向前搜索，直到距离超过前视距离或到达路径终点
    int ind = m_target_ind;
    while (Lf > calcDistance(cx[ind], cy[ind])) {
        if (ind + 1 >= static_cast<int>(cx.size())) {
            break;
        }
        ind++;
    }

    return {ind, Lf};
}

/**
 * @brief 纯追踪控制器
 * 计算达到目标路径点所需的转向角
 * @param state 当前车辆状态
 * @param cx 路径点X坐标数组
 * @param cy 路径点Y坐标数组
 * @param pind 上一时刻的目标点索引，用于防止回退
 * @return 转向角(弧度)和新的目标点索引的pair
 */
std::pair<double, int> PathTracking::pure_pursuit_control(const std::vector<double> &cx, 
                                                            const std::vector<double> &cy,int pind)
{
    // 计算目标点索引
    auto [ind, Lf] = calc_target_index(cx, cy);

    // 防止目标点回退
    if (pind >= ind)
    {
        ind = pind;
    }

    // 确保索引不越界
    if (ind > static_cast<int>(cx.size()))
        ind = static_cast<int>(cx.size()) - 1;

    // 计算目标点相对于车辆坐标系的方位角
    double alpha = std::atan2(cy[ind] - m_vehicleState.y, cx[ind] - m_vehicleState.x) - m_vehicleState.yaw;

    // 处理倒车情况
    if (m_vehicleState.v < 0)
    {
        alpha = M_PI - alpha;
    }

    // 计算前视距离
    // double Lf = k * state.v + Lfc;

    // 计算转向角 (纯追踪算法核心公式)
    double delta = std::atan2(2.0 * L * std::sin(alpha) / Lf, 1.0);

    return std::make_pair(delta, ind);
}

double PathTracking::Track(double x, double y, double yaw)
{
    // int target_speed = 0.4;

    // double ai = PControl(target_speed, state.v);

    // 更新车辆状态  传入 x y yaw
    update(x,y,yaw);

    // 计算转向角控制量
    auto [di, new_target_ind] = pure_pursuit_control(cx, cy, m_target_ind);

    m_target_ind = new_target_ind;

    // time = time + dt;
    return di;
    // // 创建并发布控制命令
    // geometry_msgs::msg::Twist cmd_vel;
    // cmd_vel.linear.x = 0.4;    // 线速度
    // cmd_vel.angular.z = di;        // 角速度
    // cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<testNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}