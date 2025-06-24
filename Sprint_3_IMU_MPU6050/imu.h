#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "miros.h"
#include "tm4c.h"
#include "bsp.h"
#include "logger.h"

void imu_init(void);
void imu_start(void);
void Task_imu(void);
void I2C0_SendByte(uint8_t slave_addr, uint8_t reg_addr);
uint8_t I2C0_ReadByte(uint8_t slave_addr, uint8_t reg_addr);
uint8_t MPU6050_WhoAmI(void);
void MPU6050_Calibrate(void);
void MPU6050_Init(void);
int16_t MPU6050_ReadWord(uint8_t reg_addr);

typedef struct {
		int16_t ax, ay, az;
		int16_t gx, gy, gz;
} Bias;
	
#endif // IMU_H