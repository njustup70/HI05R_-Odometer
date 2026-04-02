#include "chassis_odom.h"
#include <math.h>

// 实例化里程计结构体
ChassisOdom_t robot_odom;

// 常量预计算，节省单片机算力
const float PI_DIV_180 = 0.01745329f;  
const float SQRT_2_DIV_2 = 0.70710678f;
const float SQRT_2 = 1.41421356f;
USARTInstance UpdateUart; // 通讯接口
USART_Init_Config_s init_config_update;
    Action_OPS9_Frame_t tx_frame;
// 初始化函数（清零）
void Chassis_Odom_Init(void) {
    robot_odom.vx = 0.0f;
    robot_odom.vy = 0.0f;
    robot_odom.vyaw = 0.0f;
    robot_odom.x = 0.0f;
    robot_odom.y = 0.0f;
    robot_odom.yaw = 0.0f;
	
	  init_config_update.usart_handle = &huart1;
  init_config_update.recv_buff_size = 100;
  USARTRegister(&UpdateUart, &init_config_update);
	
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
uint8_t test_tx_data[2] = {0x01, 0x02};
void Send_Odom_As_OPS9(void)
{


    // 1. 填充帧头帧尾 (严格匹配接收端的 0x0A0D 和 0x0D0A)
    tx_frame.head = 0x0A0D;
    tx_frame.tail = 0x0D0A;

    // 2. 填充核心数据并转换单位
    // 弧度转度 (rad * 180 / PI)
    tx_frame.yaw = robot_odom.yaw * 57.2957795f; 
    // 米转毫米 (m * 1000)
    tx_frame.x   = robot_odom.x * 1000.0f;
    tx_frame.y   = robot_odom.y * 1000.0f;

    // 3. 填充未使用/冗余数据 (为了凑够 28 字节)
    tx_frame.z_reserved = 0.0f;
    tx_frame.vx = robot_odom.vx * 1000.0f; // 顺手把速度也按毫米发过去
    tx_frame.vy = robot_odom.vy * 1000.0f;


    // 4. 调用 HAL 库发送 28 字节
    // 注意超时时间设置短一点，比如 5ms，防止阻塞主循环
    HAL_UART_Transmit(&huart2, (uint8_t *)&tx_frame, sizeof(tx_frame), 5);

USARTSend(&UpdateUart,(uint8_t *)&tx_frame, sizeof(tx_frame),USART_TRANSFER_DMA);
}
