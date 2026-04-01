#ifndef _DTEK_TLE5012B_
#define _DTEK_TLE5012B_

#include <stdint.h>
#include "main.h"

/* ==========================================
 * 新增：多传感器 ID 枚举
 * ========================================== */
typedef enum {
    TLE_SENSOR_1 = 0,
    TLE_SENSOR_2
} TLE5012B_SensorID;

// 声明片选与同步控制函数
void TLE_CS_Enable(TLE5012B_SensorID id);
void TLE_CS_Disable(TLE5012B_SensorID id);
void triggerUpdate_Both(void);
void triggerUpdate(TLE5012B_SensorID id);

// Error masks for safety words
#define SYSTEM_ERROR_MASK           0x4000
#define INTERFACE_ERROR_MASK        0x2000
#define INV_ANGLE_ERROR_MASK        0x1000

// Commands for read
#define READ_STA_CMD_NOSAFETY       0x8000
#define READ_STA_CMD                0x8001
#define READ_ACTIV_STA_CMD          0x8011
#define READ_ANGLE_VAL_CMD          0x8021
#define READ_ANGLE_SPD_CMD          0x8031
#define READ_ANGLE_REV_CMD          0x8041
#define READ_TEMP_CMD               0x8051
#define READ_INTMODE_1              0x8061
#define READ_SIL                    0x8071
#define READ_INTMODE_2              0x8081
#define READ_INTMODE_3              0x8091
#define READ_OFFSET_X               0x80A1
#define READ_OFFSET_Y               0x80B1
#define READ_SYNCH                  0x80C1
#define READ_IFAB                   0x80D1
#define READ_INTMODE_4              0x80E1
#define READ_TEMP_COEFF             0x80F1
#define READ_RAW_X_CMD              0x8101
#define READ_RAW_Y_CMD              0x8111

// Commands for updated read
#define READ_UPD_STA_CMD            0x8401
#define READ_UPD_ANGLE_VAL_CMD      0x8421
#define READ_UPD_ANGLE_SPD_CMD      0x8431
#define READ_UPD_ANGLE_REV_CMD      0x8441

#define READ_BLOCK_CRC              0x8088

// Commands for write
#define WRITE_ACTIV_STA             0x0011
#define WRITE_INTMODE_1             0x5061
#define WRITE_SIL                   0x5071
#define WRITE_INTMODE_2             0x5081
#define WRITE_INTMODE_3             0x5091
#define WRITE_OFFSET_X              0x50A1
#define WRITE_OFFSET_Y              0x50B1
#define WRITE_SYNCH                 0x50C1
#define WRITE_IFAB                  0x50D1
#define WRITE_INTMODE_4             0x50E1
#define WRITE_TEMP_COEFF            0x50F1

#define CHECK_CMD_UPDATE            0x0400
#define CRC_POLYNOMIAL              0x1D
#define CRC_SEED                    0xFF
#define CRC_NUM_REGISTERS           8

#define DELETE_BIT_15               0x7FFF
#define CHANGE_UINT_TO_INT_15       32768
#define CHECK_BIT_14                0x4000
#define DELETE_7BITS                0x01FF
#define CHANGE_UNIT_TO_INT_9        512
#define CHECK_BIT_9                 0x0100
#define POW_2_15                    32768.0
#define POW_2_7                     128.0
#define ANGLE_360_VAL               360.0
#define TEMP_OFFSET                 152.0
#define TEMP_DIV                    2.776
#define GET_BIT_14_4                0x7FF0

#define SPEED                       500000
#define DELAYuS                     1
#define DUMMY                       0xFFFF

typedef enum errorTypes {
    NO_ERROR = 0x00,
    SYSTEM_ERROR = 0x01,
    INTERFACE_ACCESS_ERROR = 0x02,
    INVALID_ANGLE_ERROR = 0x03,
    CRC_ERROR = 0xFF
} errorTypes;

/* ==========================================
 * 接口函数：全部新增了 TLE5012B_SensorID 参数
 * ========================================== */
errorTypes readBlockCRC(TLE5012B_SensorID id);
errorTypes getAngleSpeed(TLE5012B_SensorID id, double *angleSpeed);
errorTypes getAngleValue(TLE5012B_SensorID id, double *angleValue);
errorTypes getNumRevolutions(TLE5012B_SensorID id, int16_t *numRev);

// 更新缓冲区读取（我们将主要使用这两个）
errorTypes getUpdAngleSpeed(TLE5012B_SensorID id, double *angleSpeed);
errorTypes getUpdAngleValue(TLE5012B_SensorID id, double *angleValue);
errorTypes getUpdNumRevolutions(TLE5012B_SensorID id, int16_t *numRev);

errorTypes getTemperature(TLE5012B_SensorID id, double *temp);
errorTypes getAngleRange(TLE5012B_SensorID id, double *angleRange);

#endif