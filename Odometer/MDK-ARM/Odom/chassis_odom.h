#ifndef __CHASSIS_ODOM_H
#define __CHASSIS_ODOM_H

#include <stdint.h>
#include "bsp_usart.h"
#include "usart.h"
// ========== 底盘机械参数定义 ==========
#define WHEEL_RADIUS   0.0237375f  // 轮子半径 (单位：米)
#define CHASSIS_L      0.150f      // 旋转中心到轮子的力臂长度 (单位：米)

// 定义存放底盘 6 个核心数据的结构体
typedef struct {
    // 速度数据 (Velocity)
    float vx;       // 横向平移速度 (m/s)
    float vy;       // 前向平移速度 (m/s)
    float vyaw;     // 自旋角速度 (rad/s)

    // 姿态/位置数据 (Position/Pose)
    float x;        // 全局 X 坐标 (m)
    float y;        // 全局 Y 坐标 (m)
    float yaw;      // 全局 Yaw 航向角 (rad)
} ChassisOdom_t;


#pragma pack(1)
typedef struct {
    uint16_t head;      // 帧头 0x0A0D (STM32小端模式下，实际发出去是 0x0D, 0x0A)
    float yaw;          // 航向角 (单位: 度)
    float x;            // X坐标 (单位: 毫米)
    float y;            // Y坐标 (单位: 毫米)
    float z_reserved;   // Z坐标保留值 (发 0 即可)
    float vx;           // X方向速度 (其实接收方没用上，随便发或者发真实速度)
    float vy;           // Y方向速度
    uint16_t tail;      // 帧尾 0x0D0A (STM32小端模式下，实际发出去是 0x0A, 0x0D)
} Action_OPS9_Frame_t;
#pragma pack()

// 外部声明，方便其他文件调用
extern ChassisOdom_t robot_odom;

// ========== 函数声明 ==========
void Chassis_Odom_Init(void);

/**
 * @brief 底盘里程计更新核心函数
 * @param delta_angle_1  1号磁编在 dt 时间内的角度变化量 (度)
 * @param delta_angle_2  2号磁编在 dt 时间内的角度变化量 (度)
 * @param imu_gyr_z      IMU读取的 Z 轴角速度 (deg/s)
 * @param imu_yaw        IMU读取的 绝对航向角 (deg)
 * @param dt_s           距离上次更新的时间间隔 (秒)
 */
void Chassis_Odom_Update(float delta_angle_1, float delta_angle_2, 
                         float imu_gyr_z, float imu_yaw, float dt_s);
void Send_Odom_As_OPS9(void);
#endif
