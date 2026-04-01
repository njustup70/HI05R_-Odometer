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
    errorTypes checkError = NO_ERROR;
    double d = 0.0;
    double AngleSpeed=0.0;
    double AngleValue;
    double UpdAngleSpeed;
    double UpdAngleValue;
      double Temperature;
      double AngleRange;
    int16_t NumRevolutions = 0;
    int16_t numRev = 0;


double angle1 = 0.0;
double angle2 = 0.0;
int16_t rev1 = 0;   // 一号磁编圈数 (有符号的 9 位整数，可表示正反转)
int16_t rev2 = 0;   // 二号磁编圈数
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
double angle360_1 ;
double total_angle_1;
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


  init_config.usart_handle = &huart2;
  init_config.recv_buff_size = 100;
  init_config.module_callback = Hi05RCallBack; // 这里传入的是静态函数,需要注意参数类型
  USARTRegister(&Hi05RUart, &init_config);

  
    TLE_CS_Disable(TLE_SENSOR_1);
    TLE_CS_Disable(TLE_SENSOR_1);
//    checkError = readBlockCRC();
//    printf("Init done!!! ERROR CODE: 0x%02X\r\n", checkError);
    checkError = NO_ERROR;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. 发送硬件同步脉冲：两个磁编在同一瞬间将【角度、速度、圈数】存入各自的 Update Buffer
        triggerUpdate_Both();

        // 2. 依次读取一号磁编保存在 Update Buffer 里的【角度】和【圈数】
        errorTypes err_a1 = getUpdAngleValue(TLE_SENSOR_1, &angle1);
        errorTypes err_r1 = getUpdNumRevolutions(TLE_SENSOR_1, &rev1);

        // 3. 依次读取二号磁编保存在 Update Buffer 里的【角度】和【圈数】
        errorTypes err_a2 = getUpdAngleValue(TLE_SENSOR_2, &angle2);
        errorTypes err_r2 = getUpdNumRevolutions(TLE_SENSOR_2, &rev2);

        // 4. 数据校验与应用计算
        if(err_a1 == NO_ERROR && err_r1 == NO_ERROR && 
           err_a2 == NO_ERROR && err_r2 == NO_ERROR) 
        {
           // 玩法 1：获取单圈 0~360 度的表达
             angle360_1 = angle1;
            if (angle360_1 < 0.0) angle360_1 += 360.0;

            // 玩法 2：获取多圈累加的绝对连续角度（非常适合传给 PID 做位置闭环）
             total_angle_1 = (rev1 * 360.0) + angle1;
        } 
        else 
        {
            // 如果出错（通常是 SPI 干扰导致 CRC 错误），当前循环放弃这帧数据，等下一轮
            // printf("SPI 读取 CRC 错误！\r\n");
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
