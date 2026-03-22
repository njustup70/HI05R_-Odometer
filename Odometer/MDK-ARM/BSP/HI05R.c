#include "HI05R.h"

void HI05R_get(imu_hi05r_t *_imu,uint8_t *data)
{
int offset = 6; /* Payload start at data[6] */
_imu->tag =             U1(data+offset+0);
_imu->pressure =        R4(data+offset+4);
_imu->timestamp =       U4(data+offset+8);
_imu->acc[0] =          R4(data+offset+12);
_imu->acc[1] =          R4(data+offset+16);
_imu->acc[2] =          R4(data+offset+20);
_imu->gyr[0] =          R4(data+offset+24);
_imu->gyr[1] =          R4(data+offset+28);
_imu->gyr[2] =          R4(data+offset+32);
_imu->mag[0] =          R4(data+offset+36);
_imu->mag[1] =          R4(data+offset+40);
_imu->mag[2] =          R4(data+offset+44);
_imu->eul[0] =          R4(data+offset+48);
_imu->eul[1] =          R4(data+offset+52);
_imu->eul[2] =          R4(data+offset+56);
_imu->quat[0] =         R4(data+offset+60);
_imu->quat[1] =         R4(data+offset+64);
_imu->quat[2] =         R4(data+offset+68);
_imu->quat[3] =         R4(data+offset+72);
}