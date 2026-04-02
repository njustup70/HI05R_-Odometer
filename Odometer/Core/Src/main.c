/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_usart.h"
#include "HI05R.h"
#include "DTek_TLE5012B.h"
#include <stdio.h>
#include <math.h>
#include "chassis_odom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
  uint32_t crc = *currectCrc;
  uint32_t j;
  for (j = 0; j < lengthInBytes; ++j)
  {
    uint32_t i;
    uint32_t byte = src[j];
    crc ^= byte << 8;
    for (i = 0; i < 8; ++i)
    {
      uint32_t temp = crc << 1;
      if (crc & 0x8000)
      {
      }
      temp ^= 0x1021;
      crc = temp;
    }
  }
  *currectCrc = crc;
}
// 参数
double angle1 = 0.0;
double angle2 = 0.0;
int16_t rev1 = 0; // 一号磁编圈数 (有符号的 9 位整数，可表示正反转)
int16_t rev2 = 0; // 二号磁编圈数
double angle360_1;
double total_angle_1;
double angle360_2;
double total_angle_2;

uint32_t startTick = 0;
uint32_t tick = 0;
uint32_t stopTick = 0;
float time_ms = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


USARTInstance Hi05RUart;  // HIO5R

USART_Init_Config_s init_config;
imu_hi05r_t odom_imu;
static float last_total_angle_1 = 0.0f;
static float last_total_angle_2 = 0.0f;
static uint8_t is_first_loop = 1; // 用于跳过第一次循环的差分计算

// 用于存放计算出的角速度
float omega_deg_per_sec_1 = 0.0f; // 1号磁编角速度 (度/秒)
float omega_deg_per_sec_2 = 0.0f; // 2号磁编角速度 (度/秒)
float omega_rpm_1 = 0.0f;         // 1号磁编转速 (转/分钟)
float omega_rpm_2 = 0.0f;         // 2号磁编转速 (转/分钟)
void Hi05RCallBack(void *param)
{
  uint8_t *rx_buf = Hi05RUart.recv_buff;

  // 假设你的驱动保证了 rx_buf 指向帧头
  if (rx_buf[0] == 0x5A && rx_buf[1] == 0xA5)
  {
    int16_t payload_len;
    payload_len = rx_buf[2] + (rx_buf[3] << 8);
    if (payload_len != 76)
      return;
    // 5. 传入结构体地址
    HI05R_get(&odom_imu, rx_buf);
  }
}

// 计算计数差值  只是用来计算  不能超过最大测量时间 59秒
uint32_t getDWTCountDx(uint32_t startTick, uint32_t stopTick)
{
  uint32_t tick;
  if (stopTick < startTick)
  {
    tick = (0xffffffff - startTick) + stopTick + 1;
  }
  else
  {
    tick = stopTick - startTick;
  }

  return tick;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // DWT初始化
  CoreDebug->DEMCR |= 1 << 24; // 使能DWT外设
  DWT->CYCCNT = 0;             // 清零CYCCNT
  DWT->CTRL |= 1 << 0;         // 使能计数

  init_config.usart_handle = &huart2;
  init_config.recv_buff_size = 100;
  init_config.module_callback = Hi05RCallBack; // 这里传入的是静态函数,需要注意参数类型
  USARTRegister(&Hi05RUart, &init_config);



  TLE_CS_Disable(TLE_SENSOR_1);
  TLE_CS_Disable(TLE_SENSOR_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    triggerUpdate_Both();

    // 1. 依次读取一号磁编保存在 Update Buffer 里的【角度】和【圈数】
    errorTypes err_a1 = getUpdAngleValue(TLE_SENSOR_1, &angle1);
    errorTypes err_r1 = getUpdNumRevolutions(TLE_SENSOR_1, &rev1);

    errorTypes err_a2 = getUpdAngleValue(TLE_SENSOR_2, &angle2);
    errorTypes err_r2 = getUpdNumRevolutions(TLE_SENSOR_2, &rev2);

    // 2. 精确计算时间差 (dt)
    stopTick = DWT->CYCCNT;
    tick = getDWTCountDx(startTick, stopTick);
    startTick = stopTick; // 更新起始时间戳为下一次做准备

    // 系统主频为 12MHz，将 tick 转换为真实的秒数 dt
    // 12,000,000 ticks = 1 秒
    float dt_s = (float)tick / 12000000.0f;

    // 3. 数据校验与应用计算
    if (err_a1 == NO_ERROR && err_r1 == NO_ERROR &&
        err_a2 == NO_ERROR && err_r2 == NO_ERROR)
    {
      angle360_1 = angle1;
      if (angle360_1 < 0.0f)
        angle360_1 += 360.0f;

      angle360_2 = angle2;
      if (angle360_2 < 0.0f)
        angle360_2 += 360.0f;

      // 计算当前绝对总角度
      total_angle_1 = (rev1 * 360.0f) + angle360_1;
      total_angle_2 = (rev2 * 360.0f) + angle360_2;

      // 4. 计算角速度
      if (!is_first_loop && dt_s > 0.0f)
      {
				//可能不太准，不如交给A板上的工程
        // 角度差 / 时间差 = 角速度 (度/秒)
        omega_deg_per_sec_1 = (total_angle_1 - last_total_angle_1) / dt_s;
        omega_deg_per_sec_2 = (total_angle_2 - last_total_angle_2) / dt_s;

        // 换算为 RPM (Revolutions Per Minute, 转/分钟)
        // 1转 = 360度，1分钟 = 60秒 -> RPM = (度/秒) * 60 / 360 = (度/秒) / 6.0
        omega_rpm_1 = omega_deg_per_sec_1 / 6.0f;
        omega_rpm_2 = omega_deg_per_sec_2 / 6.0f;

        float delta_a1 = total_angle_1 - last_total_angle_1;
        float delta_a2 = total_angle_2 - last_total_angle_2;

        // 调用封装好的里程计更新函数！
        // 假设 IMU 数据 gyr[2] 是 Z 轴角速度，eul[2] 是 Yaw 航向角
        Chassis_Odom_Update(delta_a1, delta_a2,
                            odom_imu.gyr[2], odom_imu.eul[2], dt_s);
				Send_Odom_As_OPS9();
      }

      // 更新历史数据
      last_total_angle_1 = total_angle_1;
      last_total_angle_2 = total_angle_2;
      is_first_loop = 0; // 清除首次运行标志位
    }
    else
    {
      // 发生错误时的处理逻辑（比如保持上一周期的角速度，或置零）
    }

    // 适当控制循环读取的频率 (比如 5ms 读一次，即 200Hz)
    HAL_Delay(5);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
