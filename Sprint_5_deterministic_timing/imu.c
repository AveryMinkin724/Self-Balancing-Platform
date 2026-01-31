#include "imu.h"

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75
#define ACCEL_SCALE (1.0f / 16384.0f) // g per LSB
#define GYRO_SCALE  (1.0f / 131.0f)   // °/s per LSB
#define CF_ALPHA 0.95f
#define CF_BETA  (1.0f - CF_ALPHA)

//#define USE_HARDCODED_BIAS

uint32_t stack_imu[512U];
OSThread imuThread;

static float pitch = 0.0f; // Global filtered angle
float dt = 0.001; //fixed 1000 Hz
static uint8_t printCounter = 0;
bias imu_bias = {0};  // Global variable, accessible throughout imu.c
mpu6050_raw raw_imu_data; 
volatile system_state_t system_state = SYS_NORMAL;
volatile float current_pitch = 0.0f;
imu_sample_t imu_data;
calibrated cal_imu_data;
trig accel_angle;
volatile float accel_angle_y;
volatile float gyro_rate_y;
	 
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
    
		system_state = SYS_CALIBRATING;
    BSP_delay(10);   // let motor task see it
	
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    float gx_sq_sum = 0, gy_sq_sum = 0, gz_sq_sum = 0;

    const int samples = 500;
    BSP_ledBlueOn();
    for (int i = 0; i < samples; ++i) {
        
				//Read Raw Gyro 
				raw_imu_data.gx = MPU6050_ReadWord(0x43);  // GYRO_XOUT_H
				raw_imu_data.gy = MPU6050_ReadWord(0x45);
				raw_imu_data.gz = MPU6050_ReadWord(0x47);
				
				//Scale data	
				imu_data.gx = raw_imu_data.gx * GYRO_SCALE;
				imu_data.gy = raw_imu_data.gy * GYRO_SCALE;
				imu_data.gz = raw_imu_data.gz * GYRO_SCALE;
			
        gx_sum += imu_data.gx; gy_sum += imu_data.gy; gz_sum += imu_data.gz;
        
        gx_sq_sum += (float)imu_data.gx * imu_data.gx;
        gy_sq_sum += (float)imu_data.gy * imu_data.gy;
        gz_sq_sum += (float)imu_data.gz * imu_data.gz;
				
				char buf[128];
        snprintf(buf, sizeof(buf),
            "GYRO[X:%f Y:%f Z:%f]\r\n",
            imu_data.gx, imu_data.gy, imu_data.gz);
        Logger_log(buf);
				
        BSP_delay(1);  //  Hz sample rate
    }

    // Compute sample means (bias)
    imu_bias.gx = gx_sum / samples;
    imu_bias.gy = gy_sum / samples;
    imu_bias.gz = gz_sum / samples;

    // Sample standard deviation (divide by n - 1)
    float gx_std = sqrtf((gx_sq_sum - (float)gx_sum * gx_sum / samples) / (samples - 1));
    float gy_std = sqrtf((gy_sq_sum - (float)gy_sum * gy_sum / samples) / (samples - 1));
    float gz_std = sqrtf((gz_sq_sum - (float)gz_sum * gz_sum / samples) / (samples - 1));

    Logger_log("Calibration Complete\r\n");

    char buf[256];

    // CSV output: bias values
    if (++printCounter >= 10) {
				printCounter = 0;
				snprintf(buf, sizeof(buf),
						"%f\t%f\t%f\t%f\t%f\t%f\r\n",
						imu_bias.ax, imu_bias.ay, imu_bias.az,
						imu_bias.gx, imu_bias.gy, imu_bias.gz);
				Logger_log("Bias (Accel X\tY\tZ | Gyro X\tY\tZ):\r\n");
				Logger_log(buf);
		}
    // CSV output: std dev values
    snprintf(buf, sizeof(buf),
				"%.2f\t%.2f\t%.2f\r\n",
				gx_std, gy_std, gz_std);
		Logger_log("StdDev (Gyro X\tY\tZ):\r\n");
		Logger_log(buf);
		
		BSP_ledBlueOff();
    system_state = SYS_NORMAL;
    BSP_ledBlueOff();
}


void MPU6050_Init(void) {
    // Write 0x01 to power mgmt reg
		I2C0_WriteByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);  // Wake up & set clock
    while (I2C0_MASTER_MCS_R & 0x01);                 // wait for done
		// Read back
		uint8_t val = I2C0_ReadByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1);
		Logger_log_hex("PWR_MGMT_1", val);
				
}

float Complementary_Filter (float dt) {
		
		pitch = CF_ALPHA * pitch
					+ CF_ALPHA * gyro_rate_y * dt
					+ CF_BETA  * accel_angle_y;
		
		return pitch;
		
}

void Task_imu(void) {
	
		bool calibrated = false;
	  static uint32_t last_tick = 0;
	
		#ifdef USE_HARDCODED_BIAS
				imu_bias.ax = 0;  // Replace with your averaged log values
				imu_bias.ay = 0;
				imu_bias.az = 0;
				imu_bias.gx = -529; //-535;
				imu_bias.gy = -223; //-185;
				imu_bias.gz = -212; //-232;
		#else
				MPU6050_Calibrate();
		#endif
		
		PID_reset();
		/* Enable Timer0A */
		ControlTimer_Start();   // safe here
	
		while (1) {
					
				//Read Raw Gyro & Accel data 
				raw_imu_data.ax = MPU6050_ReadWord(0x3B);  // ACCEL_XOUT_H
				raw_imu_data.ay = MPU6050_ReadWord(0x3D);
				raw_imu_data.az = MPU6050_ReadWord(0x3F);
				raw_imu_data.gx = MPU6050_ReadWord(0x43);  // GYRO_XOUT_H
				raw_imu_data.gy = MPU6050_ReadWord(0x45);
				raw_imu_data.gz = MPU6050_ReadWord(0x47);
				
				//Scale data
				imu_data.ax = raw_imu_data.ax * ACCEL_SCALE;
				imu_data.ay = raw_imu_data.ay * ACCEL_SCALE;
				imu_data.az = raw_imu_data.az * ACCEL_SCALE;
			
				imu_data.gx = raw_imu_data.gx * GYRO_SCALE;
				imu_data.gy = raw_imu_data.gy * GYRO_SCALE;
				imu_data.gz = raw_imu_data.gz * GYRO_SCALE;
			
				//Account for gyro calibrated bias
				cal_imu_data.gx = (imu_data.gx - imu_bias.gx);
				cal_imu_data.gy = (imu_data.gy - imu_bias.gy);
				cal_imu_data.gz = (imu_data.gz - imu_bias.gz);
				
				//Calc angle based on accelerometer
				accel_angle.y = atan2f(imu_data.ax, imu_data.az) * 180.0f / M_PI;  // degrees
				
				__disable_irq();
				accel_angle_y = accel_angle.y;
				gyro_rate_y = cal_imu_data.gy;
				__enable_irq();
				
				// Only print every 10 samples (10 × 1ms = 10ms)
				if (++printCounter >= 10) {
						printCounter = 0;
						
						__disable_irq();
						float print_pitch = current_pitch;
						float print_output = output;
						__enable_irq();
					
						char buf[128];
						snprintf(buf, sizeof(buf), "%.2f,%.2f\r\n", print_pitch, print_output);
						Logger_log(buf);
						
				}
				
				// ---- LED timeout handling ----
				if (red_led_active && BSP_tickCtr() >= red_led_off_time) {
						BSP_ledRedOff();
						red_led_active = false;
				}
				
				BSP_delay(1); // 1 tick = 1 ms, 
		}
}