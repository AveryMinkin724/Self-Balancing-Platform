#include "imu.h"

#define MPU6050_ADDR 0x68

uint32_t stack_imu[128U];
OSThread imuThread;

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

uint8_t I2C0_ReadByte(uint8_t slave_addr) {
    I2C0_MASTER_MSA_R = (slave_addr << 1) | 0x01;    // Read mode & address of slave
    I2C0_MASTER_MCS_R = 0x07;                        // Repeated start + Run + Stop
    while (I2C0_MASTER_MCS_R & 0x01);                // Wait until done
		
		if (I2C0_MASTER_MCS_R & (1 << 1)) {
				// Handle error
        return 0xFF;  // Return invalid data if error
		}
    
		return I2C0_MASTER_MDR_R;                        // Return data
}

void Task_imu(void) {
		while (1) {
				uint8_t id = MPU6050_WhoAmI();
				Logger_log_hex("MPU6050 WHO_AM_I", id);
				BSP_delay(BSP_TICKS_PER_SEC * 10U);// wait some time...
		}
}

uint8_t MPU6050_WhoAmI(void) {
		I2C0_SendByte(MPU6050_ADDR, 0x75); // WHO_AM_I register address
		return I2C0_ReadByte(MPU6050_ADDR);
}