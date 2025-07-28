#include "imu.h"

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75
#define ACCEL_SCALE (1.0f / 16384.0f) // g per LSB
#define GYRO_SCALE  (1.0f / 131.0f)   // °/s per LSB

//#define USE_HARDCODED_BIAS

uint32_t stack_imu[256U];
OSThread imuThread;
Bias imu_bias = {0};  // Global variable, accessible throughout imu.c
static float pitch = 0.0f; // Global filtered angle
float dt = 0.01f;

void imu_start(void) {
		OSThread_start(&imuThread,
                   &Task_imu,
                   stack_imu, sizeof(stack_imu));
}

void imu_init(void) {
		
	
	  /* I2C Initialization */
		// 1. Enable clocks
    SYSCTL_RCGCI2C_R |= 0x01;     // Enable clock to I2C0
    SYSCTL_RCGCGPIO_R |= 0x02;    // Enable clock to GPIOB

    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}; // Wait for GPIOB ready

    // 2. Configure PB2 and PB3 for I2C0
    GPIO_PORTB_AFSEL_R |= 0x0C;   // Enable alternate function on PB2, PB3
    GPIO_PORTB_ODR_R |= 0x08;     // Enable open-drain on PB3 (SDA)
    GPIO_PORTB_DEN_R |= 0x0C;     // Digital enable PB2, PB3
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0x0000FF00) | 0x00003300; // Configure PB2, PB3 as I2C

    // 3. Initialize I2C0 Master
    I2C0_MASTER_MCR_R = 0x10;            // Master mode
    I2C0_MASTER_MTPR_R = 7;              // 100 kHz @ 16 MHz clock (TPR = (16MHz / (20 * 100kHz)) - 1 = 7)

		/* Wake Up IMU */
		MPU6050_Init();
}


void I2C0_SendByte(uint8_t slave_addr, uint8_t reg_addr) {
    I2C0_MASTER_MSA_R = (slave_addr << 1);           // Write mode & address of slave
    I2C0_MASTER_MDR_R = reg_addr;										 // data you want to send goes in here (in our case this woudl be the address on the register on the MPU 6050 that we want to read)
    I2C0_MASTER_MCS_R = 0x03;                        // Start + Run
    while (I2C0_MASTER_MCS_R & 0x01);                // Wait until done
		
		if (I2C0_MASTER_MCS_R & (1 << 1)) {
				// Handle error (could be logging, setting a flag, retry, etc.)
		}
}

uint8_t I2C0_ReadByte(uint8_t slave_addr, uint8_t reg_addr) {
    I2C0_SendByte(slave_addr, reg_addr);     // Set the register pointer
    I2C0_MASTER_MSA_R = (slave_addr << 1) | 1;  // Read mode
    I2C0_MASTER_MCS_R = 0x07;  // Start + Run + stop
    while (I2C0_MASTER_MCS_R & 0x01);  // Wait for high byte
    
		if (I2C0_MASTER_MCS_R & (1 << 1)) {
				// Handle error
        return 0xFF;  // Return invalid data if error
		}
	
		return I2C0_MASTER_MDR_R;
}

void I2C0_WriteByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data) {
    // Send register address
    I2C0_MASTER_MSA_R = (slave_addr << 1);  // write mode
    I2C0_MASTER_MDR_R = reg_addr;
    I2C0_MASTER_MCS_R = 0x03;  // start + run
    while (I2C0_MASTER_MCS_R & 0x01); // wait
		
		if (I2C0_MASTER_MCS_R & (1 << 1)) { // Check error bit
        Logger_log("I2C ERROR: Register address write failed");
        return;
    }
		
    // Send data
    I2C0_MASTER_MDR_R = data;
    I2C0_MASTER_MCS_R = 0x05;  // run + stop
    while (I2C0_MASTER_MCS_R & 0x01);
		
		if (I2C0_MASTER_MCS_R & (1 << 1)) {
        Logger_log("I2C ERROR: Data write failed");
    }
}

int16_t MPU6050_ReadWord(uint8_t reg_addr) {
    I2C0_SendByte(MPU6050_ADDR, reg_addr);     // Set the register pointer
    I2C0_MASTER_MSA_R = (MPU6050_ADDR << 1) | 1;  // Read mode
    I2C0_MASTER_MCS_R = 0x0B;  // Start + Run (no stop)
    while (I2C0_MASTER_MCS_R & 0x01);  // Wait for high byte
    uint8_t high = I2C0_MASTER_MDR_R;

    I2C0_MASTER_MCS_R = 0x05;  // Run + Stop
    while (I2C0_MASTER_MCS_R & 0x01);  // Wait for low byte
    uint8_t low = I2C0_MASTER_MDR_R;

    return (int16_t)((high << 8) | low);  // Combine bytes
}

uint8_t MPU6050_WhoAmI(void) {
		return I2C0_ReadByte(MPU6050_ADDR, WHO_AM_I);
}
void MPU6050_Calibrate(void) {
		int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
		int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
	
		BSP_ledBlueOn();
		for(int i = 0; i < 100; ++i) {
				//Read Gyro & Accel data 
				int16_t ax = MPU6050_ReadWord(0x3B);  // ACCEL_XOUT_H
				int16_t ay = MPU6050_ReadWord(0x3D);
				int16_t az = MPU6050_ReadWord(0x3F);
				int16_t gx = MPU6050_ReadWord(0x43);  // GYRO_XOUT_H
				int16_t gy = MPU6050_ReadWord(0x45);
				int16_t gz = MPU6050_ReadWord(0x47);
				
				ax_sum += ax;
				ay_sum += ay;
				az_sum += az;
				gx_sum += gx;
				gy_sum += gy;
				gz_sum += gz;
				
				char buf[128];
				snprintf(buf, sizeof(buf),
						"ACC[X:%d Y:%d Z:%d] GYRO[X:%d Y:%d Z:%d]\r\n",
						ax, ay, az, gx, gy, gz);
				Logger_log(buf);
				
				BSP_delay(BSP_TICKS_PER_SEC / 2U);  // 5 Hz logging rate
		}
		
		imu_bias.ax = ax_sum / 100;
		imu_bias.ay = ay_sum / 100;
		imu_bias.az = az_sum / 100;
		imu_bias.gx = gx_sum / 100;
		imu_bias.gy = gy_sum / 100;
		imu_bias.gz = gz_sum / 100;
		Logger_log("Calibration Complete\r\n");
		char buf[128];
		snprintf(buf, sizeof(buf),
			"Bias Values: ACC[X:%d Y:%d Z:%d] GYRO[X:%d Y:%d Z:%d]\r\n",
				imu_bias.ax, imu_bias.ay, imu_bias.az, imu_bias.gx, imu_bias.gy, imu_bias.gz);
		Logger_log(buf);
		
		BSP_ledBlueOff();
}

void MPU6050_Init(void) {
    //Write 0x01 to power mgmt reg
		I2C0_WriteByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);  // Wake up & set clock
    while (I2C0_MASTER_MCS_R & 0x01);                 // wait for done
		// Read back
		uint8_t val = I2C0_ReadByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1);
		Logger_log_hex("PWR_MGMT_1", val);
				
}

float Complementary_Filter (void) {
		
		//Read Gyro & Accel data 
		int16_t ax = MPU6050_ReadWord(0x3B);  // ACCEL_XOUT_H
		int16_t ay = MPU6050_ReadWord(0x3D);
		int16_t az = MPU6050_ReadWord(0x3F);
		int16_t gx = MPU6050_ReadWord(0x43);  // GYRO_XOUT_H
		int16_t gy = MPU6050_ReadWord(0x45);
		int16_t gz = MPU6050_ReadWord(0x47);
		
		float ax_g = (ax - imu_bias.ax) * ACCEL_SCALE;
		float ay_g = (ay - imu_bias.ay) * ACCEL_SCALE;
		float az_g = (az - imu_bias.az) * ACCEL_SCALE;

		float gyro_rate = (gx - imu_bias.gx) * GYRO_SCALE;
		float gy_dps = (gy - imu_bias.gy) * GYRO_SCALE;
		float gz_dps = (gz - imu_bias.gz) * GYRO_SCALE;
		
		float accel_angle = atan2f(ax, az) * 180.0f / M_PI;  // degrees
		
		pitch = 0.90f * (pitch + gyro_rate * 0.01f) + 0.1f * accel_angle;
		
		/*
		char buf[128];
		snprintf(buf, sizeof(buf),
				"Pitch: %.2f\r\n", pitch);
		Logger_log(buf);
		*/
		
		return pitch;
		
}


void Task_imu(void) {
		bool calibrated = false;
	
		#ifdef USE_HARDCODED_BIAS
				imu_bias.ax = -104;  // Replace with your averaged log values
				imu_bias.ay = 108;
				imu_bias.az = 16232;
				imu_bias.gx = 120;
				imu_bias.gy = -88;
				imu_bias.gz = 36;
		#else
				MPU6050_Calibrate();
		#endif
		
		while (1) {
				/* Check MPU6050 ID */
				//uint8_t id = MPU6050_WhoAmI();
				//Logger_log_hex("MPU6050 WHO_AM_I", id);
				//BSP_delay(BSP_TICKS_PER_SEC * 10U);// wait some time...
				
				/*
				 if (!calibrated || BSP_buttonPressed()) {
            MPU6050_Calibrate();
            calibrated = true;
        }
				*/
			
				float current_pitch = Complementary_Filter();
				output = PID_update(current_pitch, dt, Kp, Ki, Kd);
				
				BSP_delay(1); // 10 ms : BSP_TICKS_PER_SEC = 100, systick fires every 10 ms, argument of 1 into BSP delay() simply delays 1 tick which is 10 ms, 2 = 20 ms, 3 30ms ... 100 = 1 sec
		}
}