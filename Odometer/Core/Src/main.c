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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_usart.h"
#include "HI05R.h"
#include "DTek_TLE5012B.h"
#include <stdio.h> 
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
USARTInstance Hi05RUart; // HIO5R
USART_Init_Config_s init_config;
imu_hi05r_t odom_imu;

volatile uint8_t read_encoder_flag = 0; // 定时器触发标志位

// 传感器数据存储
double AngleValue = 0.0;        // 实时角度 (度)
double AngleSpeed = 0.0;        // 实时角速度 (度/秒)
int16_t NumRevolutions = 0;     // 实时圈数

double UpdAngleValue = 0.0;     // 快照角度 (度)
double UpdAngleSpeed = 0.0;     // 快照角速度 (度/秒)
int16_t numRev = 0;             // 快照圈数

double Temperature = 0.0;       // 传感器温度 (摄氏度)
double AngleRange = 0.0;        // 角度量程设置

errorTypes checkError = NO_ERROR; // 全局错误状态码



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
		
		
//    uint16_t calc_crc;
//    calc_crc = 0;

//    /* Calculate 5A A5 and LEN field crc */
//    crc16_update(&calc_crc, rx_buf, 4);
//    /* Calculate payload crc */
//    crc16_update(&calc_crc, rx_buf + 6, payload_len);
//		
//    uint16_t received_crc = (rx_buf[5]); // 假设 CRC 是低字节在前

//    if (received_crc != calc_crc)
//      return;

    // 5. 传入结构体地址
    HI05R_get(&odom_imu, rx_buf);
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1) 
    {
        read_encoder_flag = 1; // 通知主循环去读数据
    }
}
void Read_All_Sensor_Data(void)
{
    // 等待定时器中断立下标志位
    if (read_encoder_flag == 1)
    {
        read_encoder_flag = 0; // 进门立刻清除标志位
        checkError = NO_ERROR; // 确保初始状态为无错误

        /* =========================================================
         * 第一步：读取实时数据
         * ========================================================= */
        if (checkError == NO_ERROR) checkError = getAngleValue(&AngleValue);
        if (checkError == NO_ERROR) checkError = getAngleSpeed(&AngleSpeed);
        if (checkError == NO_ERROR) checkError = getNumRevolutions(&NumRevolutions);

        /* =========================================================
         * 第二步：读取快照数据 (底盘运动学解算的核心数据)
         * ========================================================= */
        if (checkError == NO_ERROR) 
        {
            // 仅当前面没出错时，才发送锁存脉冲
            triggerUpdate(); 
            checkError = getUpdAngleValue(&UpdAngleValue);
        }
        if (checkError == NO_ERROR) checkError = getUpdAngleSpeed(&UpdAngleSpeed);
        if (checkError == NO_ERROR) checkError = getUpdNumRevolutions(&numRev);

        /* =========================================================
         * 第三步：读取系统状态与配置
         * ========================================================= */
        if (checkError == NO_ERROR) checkError = getTemperature(&Temperature);
        if (checkError == NO_ERROR) checkError = getAngleRange(&AngleRange);

        /* =========================================================
         * 结果判定与处理
         * ========================================================= */
        if (checkError == NO_ERROR) 
        {

        }
        else 
        {
            // 错误归零，等待下一个定时器周期重新尝试
            checkError = NO_ERROR; 
        }
    }
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);

  init_config.usart_handle = &huart2;
  init_config.recv_buff_size = 100;
  init_config.module_callback = Hi05RCallBack; // 这里传入的是静态函数,需要注意参数类型
  USARTRegister(&Hi05RUart, &init_config);

  
    SPI_CS_DISABLE;
    checkError = readBlockCRC();
//    printf("Init done!!! ERROR CODE: 0x%02X\r\n", checkError);
    checkError = NO_ERROR;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      Read_All_Sensor_Data();
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
