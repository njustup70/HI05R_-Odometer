#include "chassis_odom.h"
#include <math.h>

// 实例化里程计结构体
ChassisOdom_t robot_odom;

// 常量预计算，节省单片机算力
const float PI_DIV_180 = 0.01745329f;  
const float SQRT_2_DIV_2 = 0.70710678f;
const float SQRT_2 = 1.41421356f;

// 初始化函数（清零）
void Chassis_Odom_Init(void) {
    robot_odom.vx = 0.0f;
    robot_odom.vy = 0.0f;
    robot_odom.vyaw = 0.0f;
    robot_odom.x = 0.0f;
    robot_odom.y = 0.0f;
    robot_odom.yaw = 0.0f;
}

// 核心更新函数
void Chassis_Odom_Update(float delta_angle_1, float delta_angle_2, 
                         float imu_gyr_z, float imu_yaw, float dt_s) 
{
    if (dt_s <= 0.0f) return; // 防止除以0

    // 1. 将轮子角度变化率 (deg/s) 转化为 边缘线速度 (m/s)
    float omega_deg_1 = delta_angle_1 / dt_s;
    float omega_deg_2 = delta_angle_2 / dt_s;
    float v1 = (omega_deg_1 * PI_DIV_180) * WHEEL_RADIUS;
    float v2 = (omega_deg_2 * PI_DIV_180) * WHEEL_RADIUS;

    // 2. 采信 IMU 数据 (角度转弧度)
    robot_odom.vyaw = imu_gyr_z * PI_DIV_180;
    robot_odom.yaw  = imu_yaw * PI_DIV_180;

    // 3. 运动学正解计算 (局部速度)
    robot_odom.vy = SQRT_2_DIV_2 * (v1 + v2);
    robot_odom.vx = ((v1 - v2) + 2.0f * CHASSIS_L * robot_odom.vyaw) / SQRT_2;

    // 4. 航位推算 (更新全局坐标 X, Y)
    // 使用 2D 旋转矩阵将局部坐标系下的速度投影到全局坐标系
    float delta_x = (robot_odom.vx * cosf(robot_odom.yaw) - robot_odom.vy * sinf(robot_odom.yaw)) * dt_s;
    float delta_y = (robot_odom.vx * sinf(robot_odom.yaw) + robot_odom.vy * cosf(robot_odom.yaw)) * dt_s;

    robot_odom.x += delta_x;
    robot_odom.y += delta_y;
}