#ifndef __HIO5R_H
#define __HIO5R_H

#include "stdio.h"
#include "string.h"
#include "main.h"
/* Common type conversion */
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
#define I2(p) (*((int16_t  *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
static int32_t  I4(uint8_t *p) {int32_t  u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float    r; memcpy(&r,p,4); return r;}
typedef struct
{
    uint8_t     tag;              /* Item tag: 0x91        */
    float       acc[3];           /* Acceleration          */
    float       gyr[3];           /* Angular velocity      */  
    float       mag[3];           /* Magnetic field        */
    float       eul[3];           /* Attitude: Euler angle */
    float       quat[4];          /* Attitude: quaternion  */
    float       pressure;         /* Air pressure          */
    uint32_t    timestamp;        /* Timestamp             */
}imu_hi05r_t;

void HI05R_get(imu_hi05r_t *_imu,uint8_t *data);


#endif