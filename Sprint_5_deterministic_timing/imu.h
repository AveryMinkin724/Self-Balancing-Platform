#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "miros.h"
#include "tm4c.h"
#include "bsp.h"
#include "logger.h"
#include "PID.h"
#include "bluetooth.h"
#define _USE_MATH_DEFINES  // Must be defined before including math.h
#define M_PI 3.14159265358979323846f
#include <math.h>
#include "TM4C123GH6PM.h" /* the TM4C MCU Peripheral Access Layer (TI) */
void imu_init(void);
void imu_start(void);
void Task_imu(void);
void I2C0_SendByte(uint8_t slave_addr, uint8_t reg_addr);
uint8_t I2C0_ReadByte(uint8_t slave_addr, uint8_t reg_addr);
uint8_t MPU6050_WhoAmI(void);
void MPU6050_Calibrate(void);
void MPU6050_Init(void);
int16_t MPU6050_ReadWord(uint8_t reg_addr);
float Complementary_Filter (float dt);
extern volatile float current_pitch;

//extern float dt; don't need to declare as global, local to imu.c task and passed to PID function

typedef struct {
		int16_t ax, ay, az;
		int16_t gx, gy, gz;
} mpu6050_raw;

typedef struct {
		float ax, ay, az;
		float gx, gy, gz;
} bias;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    uint32_t timestamp;
} imu_sample_t;

extern imu_sample_t imu_data;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} calibrated;

extern calibrated cal_imu_data;

typedef struct {
    float x, y, z;
} trig;

extern trig accel_angle;

extern volatile float accel_angle_y;
extern volatile float gyro_rate_y;

typedef enum {
    SYS_NORMAL = 0,
    SYS_CALIBRATING,
} system_state_t;

extern volatile system_state_t system_state;

#endif // IMU_H