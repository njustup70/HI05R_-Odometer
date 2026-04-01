#include "DTek_TLE5012B.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#define USE_SOFT_HIGH_Z

#ifdef USE_SOFT_HIGH_Z
#define GPIO_MODER_INPUT 0x00000000u
#define GPIO_MODER_ANALOG 0x00000003u
#define GPIO_MODER_AF 0x00000002u
#define REGISTER_OFFSET (7U * 2U)
#define SPI_MOSI_HIGH_Z MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, (GPIO_MODER_ANALOG << REGISTER_OFFSET))
#define SPI_MOSI_LOW_Z MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, (GPIO_MODER_AF << REGISTER_OFFSET))
#endif

uint16_t _registers[CRC_NUM_REGISTERS];

/* ==========================================
 * 底层片选与硬件同步控制
 * ========================================== */
void TLE_CS_Enable(TLE5012B_SensorID id)
{
    if (id == TLE_SENSOR_1)
    {
        HAL_GPIO_WritePin(CS_1_GPIO_Port, CS_1_Pin, GPIO_PIN_RESET);
    }
    else if (id == TLE_SENSOR_2)
    {
        HAL_GPIO_WritePin(CS_2_GPIO_Port, CS_2_Pin, GPIO_PIN_RESET);
    }
}

void TLE_CS_Disable(TLE5012B_SensorID id)
{
    if (id == TLE_SENSOR_1)
    {
        HAL_GPIO_WritePin(CS_1_GPIO_Port, CS_1_Pin, GPIO_PIN_SET);
    }
    else if (id == TLE_SENSOR_2)
    {
        HAL_GPIO_WritePin(CS_2_GPIO_Port, CS_2_Pin, GPIO_PIN_SET);
    }
}

/* 同时触发两个传感器的锁存 */
void triggerUpdate_Both(void)
{
    // SCK LOW, MOSI HIGH
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

    // 同时拉低两个 CS
    HAL_GPIO_WritePin(CS_1_GPIO_Port, CS_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_2_GPIO_Port, CS_2_Pin, GPIO_PIN_RESET);

    // 粗略延时 1~2us，替代原来的 HAL_Delay(1) 毫秒级延时
    for (volatile int i = 0; i < 150; i++)
        ;

    // 同时拉高两个 CS
    HAL_GPIO_WritePin(CS_1_GPIO_Port, CS_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_2_GPIO_Port, CS_2_Pin, GPIO_PIN_SET);
}

void triggerUpdate(TLE5012B_SensorID id)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    TLE_CS_Enable(id);
    for (volatile int i = 0; i < 150; i++)
        ;
    TLE_CS_Disable(id);
}

/* ==========================================
 * CRC 校验逻辑
 * ========================================== */
uint8_t _getFirstByte(uint16_t twoByteWord) { return (uint8_t)(twoByteWord >> 8U); }
uint8_t _getSecondByte(uint16_t twoByteWord) { return (uint8_t)twoByteWord; }

uint8_t _crc8(uint8_t *data, uint8_t length)
{
    uint32_t crc = CRC_SEED;
    int16_t i, bit;
    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            if ((crc & 0x80) != 0)
            {
                crc <<= 1;
                crc ^= CRC_POLYNOMIAL;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return (~crc);
}

uint8_t _crcCalc(uint8_t *crcData, uint8_t length) { return _crc8(crcData, length); }

void resetSafety(TLE5012B_SensorID id)
{
    uint16_t u16RegValue = 0;
    triggerUpdate(id);
    TLE_CS_Enable(id);
    u16RegValue = READ_STA_CMD;
    HAL_SPI_Transmit(&hspi1, (uint8_t *)(&u16RegValue), 1, 0xFF);
    u16RegValue = DUMMY;
    HAL_SPI_Transmit(&hspi1, (uint8_t *)(&u16RegValue), 1, 0xFF);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)(&u16RegValue), 1, 0xFF);
    TLE_CS_Disable(id);
}

errorTypes checkSafety(TLE5012B_SensorID id, uint16_t safety, uint16_t command, uint16_t *readreg, uint16_t length)
{
    errorTypes errorCheck;
    if (!((safety)&SYSTEM_ERROR_MASK))
    {
        errorCheck = SYSTEM_ERROR;
    }
    else if (!((safety)&INTERFACE_ERROR_MASK))
    {
        errorCheck = INTERFACE_ACCESS_ERROR;
    }
    else if (!((safety)&INV_ANGLE_ERROR_MASK))
    {
        errorCheck = INVALID_ANGLE_ERROR;
    }
    else
    {
        uint16_t lengthOfTemp = length * 2 + 2;
        uint8_t temp[lengthOfTemp];
        temp[0] = _getFirstByte(command);
        temp[1] = _getSecondByte(command);
        for (uint16_t i = 0; i < length; i++)
        {
            temp[2 + 2 * i] = _getFirstByte(readreg[i]);
            temp[2 + 2 * i + 1] = _getSecondByte(readreg[i]);
        }
        uint8_t crcReceivedFinal = _getSecondByte(safety);
        uint8_t crc = _crcCalc(temp, lengthOfTemp);
        if (crc == crcReceivedFinal)
        {
            errorCheck = NO_ERROR;
        }
        else
        {
            errorCheck = CRC_ERROR;
            resetSafety(id);
        }
    }
    return errorCheck;
}

/* ==========================================
 * 核心通讯与寄存器读取读取
 * ========================================== */
errorTypes readFromSensor(TLE5012B_SensorID id, uint16_t command, uint16_t *data)
{
    uint16_t safety = 0, readreg = 0, u16RegValue = command;
    TLE_CS_Enable(id);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)(&u16RegValue), 1, 0xFF);
#ifdef USE_SOFT_HIGH_Z
    SPI_MOSI_HIGH_Z;
#endif
    HAL_SPI_Receive(&hspi1, (uint8_t *)(&readreg), 1, 0xFF);
    HAL_SPI_Receive(&hspi1, (uint8_t *)(&safety), 1, 0xFF);
#ifdef USE_SOFT_HIGH_Z
    SPI_MOSI_LOW_Z;
#endif
    TLE_CS_Disable(id);

    errorTypes checkError = checkSafety(id, safety, command, &readreg, 1);
    if (checkError != NO_ERROR)
    {
        *data = 0;
        return checkError;
    }
    else
    {
        *data = readreg;
        return NO_ERROR;
    }
}

errorTypes readBlockCRC(TLE5012B_SensorID id)
{
    uint16_t u16RegValue = READ_BLOCK_CRC, safety = 0;
    TLE_CS_Enable(id);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)(&u16RegValue), 1, 0xFFFF);
#ifdef USE_SOFT_HIGH_Z
    SPI_MOSI_HIGH_Z;
#endif
    HAL_SPI_Receive(&hspi1, (uint8_t *)(&_registers), CRC_NUM_REGISTERS, 0xFF);
    HAL_SPI_Receive(&hspi1, (uint8_t *)(&safety), 1, 0xFF);
#ifdef USE_SOFT_HIGH_Z
    SPI_MOSI_LOW_Z;
#endif
    TLE_CS_Disable(id);
    return checkSafety(id, safety, READ_BLOCK_CRC, _registers, CRC_NUM_REGISTERS);
}

errorTypes readAngleValue(TLE5012B_SensorID id, int16_t *data)
{
    uint16_t rawData = 0;
    errorTypes status = readFromSensor(id, READ_ANGLE_VAL_CMD, &rawData);
    if (status != NO_ERROR)
        return status;
    rawData = (rawData & DELETE_BIT_15);
    if (rawData & CHECK_BIT_14)
        rawData = rawData - CHANGE_UINT_TO_INT_15;
    *data = rawData;
    return NO_ERROR;
}

errorTypes readAngleSpeed(TLE5012B_SensorID id, int16_t *data)
{
    uint16_t rawData = 0;
    errorTypes status = readFromSensor(id, READ_ANGLE_SPD_CMD, &rawData);
    if (status != NO_ERROR)
        return status;
    rawData = (rawData & DELETE_BIT_15);
    if (rawData & CHECK_BIT_14)
        rawData = rawData - CHANGE_UINT_TO_INT_15;
    *data = rawData;
    return NO_ERROR;
}

errorTypes readUpdAngleValue(TLE5012B_SensorID id, int16_t *data)
{
    uint16_t rawData = 0;
    errorTypes status = readFromSensor(id, READ_UPD_ANGLE_VAL_CMD, &rawData);
    if (status != NO_ERROR)
    {
        data = 0;
        return status;
    }
    rawData = (rawData & DELETE_BIT_15);
    if (rawData & CHECK_BIT_14)
        rawData = rawData - CHANGE_UINT_TO_INT_15;
    *data = rawData;
    return NO_ERROR;
}

errorTypes readUpdAngleSpeed(TLE5012B_SensorID id, int16_t *data)
{
    uint16_t rawData = 0;
    errorTypes status = readFromSensor(id, READ_UPD_ANGLE_SPD_CMD, &rawData);
    if (status != NO_ERROR)
    {
        *data = 0;
        return status;
    }
    rawData = (rawData & DELETE_BIT_15);
    if (rawData & CHECK_BIT_14)
        rawData = rawData - CHANGE_UINT_TO_INT_15;
    *data = rawData;
    return NO_ERROR;
}

errorTypes readUpdAngleRevolution(TLE5012B_SensorID id, int16_t *data)
{
    uint16_t rawData = 0;
    errorTypes status = readFromSensor(id, READ_UPD_ANGLE_REV_CMD, &rawData);
    if (status != NO_ERROR)
    {
        data = 0;
        return status;
    }
    rawData = (rawData & DELETE_7BITS);
    if (rawData & CHECK_BIT_9)
        rawData = rawData - CHANGE_UNIT_TO_INT_9;
    *data = rawData;
    return NO_ERROR;
}

errorTypes readAngleRevolution(TLE5012B_SensorID id, int16_t *data)
{
    uint16_t rawData = 0;
    errorTypes status = readFromSensor(id, READ_ANGLE_REV_CMD, &rawData);
    if (status != NO_ERROR)
        return status;
    rawData = (rawData & DELETE_7BITS);
    if (rawData & CHECK_BIT_9)
        rawData = rawData - CHANGE_UNIT_TO_INT_9;
    *data = rawData;
    return NO_ERROR;
}

errorTypes readTemp(TLE5012B_SensorID id, int16_t *data)
{
    uint16_t rawData = 0;
    errorTypes status = readFromSensor(id, READ_TEMP_CMD, &rawData);
    if (status != NO_ERROR)
    {
        *data = 0;
        return status;
    }
    rawData = (rawData & DELETE_7BITS);
    if (rawData & CHECK_BIT_9)
        rawData = rawData - CHANGE_UNIT_TO_INT_9;
    *data = rawData;
    return NO_ERROR;
}

errorTypes readIntMode1(TLE5012B_SensorID id, uint16_t *data) { return readFromSensor(id, READ_INTMODE_1, data); }
errorTypes readIntMode2(TLE5012B_SensorID id, uint16_t *data) { return readFromSensor(id, READ_INTMODE_2, data); }

double _calculateAngleSpeed(double angRange, int16_t rawAngleSpeed, uint16_t firMD, uint16_t predictionVal)
{
    double finalAngleSpeed, microsecToSec = 0.000001, firMDVal;
    if (firMD == 1)
        firMDVal = 42.7;
    else if (firMD == 0)
        firMDVal = 21.3;
    else if (firMD == 2)
        firMDVal = 85.3;
    else if (firMD == 3)
        firMDVal = 170.6;
    else
        firMDVal = 0;
    finalAngleSpeed = ((angRange / POW_2_15) * ((double)rawAngleSpeed)) / (((double)predictionVal) * firMDVal * microsecToSec);
    return finalAngleSpeed;
}

/* ==========================================
 * 顶层 API 输出
 * ========================================== */
errorTypes getAngleRange(TLE5012B_SensorID id, double *angleRange)
{
    uint16_t rawData = 0;
    errorTypes checkError = readIntMode2(id, &rawData);
    if (checkError != NO_ERROR)
        return checkError;
    rawData &= GET_BIT_14_4;
    rawData >>= 4;
    *angleRange = ANGLE_360_VAL * (POW_2_7 / (double)(rawData));
    return NO_ERROR;
}

errorTypes getAngleSpeed(TLE5012B_SensorID id, double *finalAngleSpeed)
{
    int16_t rawAngleSpeed = 0;
    double angleRange = 0.0;
    uint16_t firMDVal = 0, intMode2Prediction = 0;
    if (readAngleSpeed(id, &rawAngleSpeed) != NO_ERROR)
        return CRC_ERROR;
    if (getAngleRange(id, &angleRange) != NO_ERROR)
        return CRC_ERROR;
    if (readIntMode1(id, &firMDVal) != NO_ERROR)
        return CRC_ERROR;
    firMDVal >>= 14;
    if (readIntMode2(id, &intMode2Prediction) != NO_ERROR)
        return CRC_ERROR;
    intMode2Prediction = (intMode2Prediction & 0x0004) ? 3 : 2;
    *finalAngleSpeed = _calculateAngleSpeed(angleRange, rawAngleSpeed, firMDVal, intMode2Prediction);
    return NO_ERROR;
}

errorTypes getAngleValue(TLE5012B_SensorID id, double *angleValue)
{
    int16_t rawAnglevalue = 0;
    errorTypes checkError = readAngleValue(id, &rawAnglevalue);
    if (checkError != NO_ERROR)
        return checkError;
    *angleValue = (ANGLE_360_VAL / POW_2_15) * ((double)rawAnglevalue);
    return NO_ERROR;
}

errorTypes getNumRevolutions(TLE5012B_SensorID id, int16_t *numRev) { return readAngleRevolution(id, numRev); }

errorTypes getUpdAngleSpeed(TLE5012B_SensorID id, double *angleSpeed)
{
    int16_t rawAngleSpeed = 0;
    double angleRange = 0.0;
    uint16_t firMDVal = 0, intMode2Prediction = 0;
    if (readUpdAngleSpeed(id, &rawAngleSpeed) != NO_ERROR)
        return CRC_ERROR;
    if (getAngleRange(id, &angleRange) != NO_ERROR)
        return CRC_ERROR;
    if (readIntMode1(id, &firMDVal) != NO_ERROR)
        return CRC_ERROR;
    if (readIntMode2(id, &intMode2Prediction) != NO_ERROR)
        return CRC_ERROR;
    intMode2Prediction = (intMode2Prediction & 0x0004) ? 3 : 2;
    *angleSpeed = _calculateAngleSpeed(angleRange, rawAngleSpeed, firMDVal, intMode2Prediction);
    return NO_ERROR;
}

errorTypes getUpdAngleValue(TLE5012B_SensorID id, double *angleValue)
{
    int16_t rawAnglevalue = 0;
    errorTypes checkError = readUpdAngleValue(id, &rawAnglevalue);
    if (checkError != NO_ERROR)
        return checkError;
    *angleValue = (ANGLE_360_VAL / POW_2_15) * ((double)rawAnglevalue);
    return NO_ERROR;
}

errorTypes getUpdNumRevolutions(TLE5012B_SensorID id, int16_t *numRev) { return readUpdAngleRevolution(id, numRev); }

errorTypes getTemperature(TLE5012B_SensorID id, double *temperature)
{
    int16_t rawTemp = 0;
    errorTypes checkError = readTemp(id, &rawTemp);
    if (checkError != NO_ERROR)
        return checkError;
    *temperature = (rawTemp + TEMP_OFFSET) / (TEMP_DIV);
    return NO_ERROR;
}