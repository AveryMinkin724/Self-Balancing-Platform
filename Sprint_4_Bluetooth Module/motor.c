#include "motor.h"

#define DIR_FORWARD 0
#define DIR_REVERSE 1

uint32_t stack_motor[256U];
OSThread motorThread;

void motor_start(void) {
    OSThread_start(&motorThread,
                   &Task_motor,
                   stack_motor, sizeof(stack_motor));
}

void motor_init(void) {
    // === Enable Clocks ===
    SYSCTL_RCGCGPIO_R |= (1U << 0) | (1U << 1) | (1U << 3); 
    // GPIOA (0), GPIOB (1), GPIOD (3)
    SYSCTL->RCGCPWM |= (1U << 0);   // Enable clock for PWM0
    for (volatile int i = 0; i < 1000; i++) {} // Delay

    SYSCTL_RCC_R &= ~(1 << 20);  // Ensure system clock is used (USEPWMDIV=0)

    // === Configure Motor A (Left) ===
    // IN1 = PA3, IN2 = PA4
    GPIO_PORTA_DIR_R |= (1U << 3) | (1U << 4);  // Outputs
    GPIO_PORTA_DEN_R |= (1U << 3) | (1U << 4);  // Digital enable

    // ENA = PB6 (M0PWM0)
    GPIO_PORTB_AFSEL_R |= (1 << 6);
    GPIO_PORTB_PCTL_R &= ~(0xF << 24);
    GPIO_PORTB_PCTL_R |=  (0x4 << 24);   // M0PWM0
    GPIO_PORTB_DEN_R  |=  (1 << 6);
    GPIO_PORTB_DIR_R  |=  (1 << 6);

    // === Configure Motor B (Right) ===
    // IN3 = PD6, IN4 = PD7
    GPIO_PORTD_LOCK_R = 0x4C4F434B;
    GPIO_PORTD_CR_R   |= (1U << 6) | (1U << 7);
    GPIO_PORTD_DIR_R  |= (1U << 6) | (1U << 7);
    GPIO_PORTD_DEN_R  |= (1U << 6) | (1U << 7);

    // ENB = PB7 (M0PWM1)
    GPIO_PORTB_AFSEL_R |= (1 << 7);
    GPIO_PORTB_PCTL_R &= ~(0xF << 28);
    GPIO_PORTB_PCTL_R |=  (0x4 << 28);   // M0PWM1
    GPIO_PORTB_DEN_R  |=  (1 << 7);
    GPIO_PORTB_DIR_R  |=  (1 << 7);

    // === Configure PWM Generator 0 ===
    PWM0->_0_CTL &= ~0x01;             // Disable Generator 0 while configuring
    PWM0->_0_GENA = 0x0000008C;        // Set on LOAD, clear on CMPA down (for M0PWM0)
    PWM0->_0_GENB = 0x0000080C;        // Set on LOAD, clear on CMPB down (for M0PWM1)

    uint32_t pwm_period = 16000;       // 10kHz @ 16 MHz
    PWM0->_0_LOAD = pwm_period - 1;
    PWM0->_0_CMPA = pwm_period / 2;    // 50% duty (ENA)
    PWM0->_0_CMPB = pwm_period / 2;    // 50% duty (ENB)
    PWM0->_0_CTL |= 0x01;              // Enable Generator 0
    PWM0->ENABLE |= (1 << 0) | (1 << 1); // Enable M0PWM0 (PB6) and M0PWM1 (PB7)
}

void pwm_set_duty_cycle(uint8_t percent) {
    if (percent > 100) percent = 100;

    // PB6 = M0PWM0, PB7 = M0PWM1 -> both from PWM0 Generator 0
    uint32_t load = PWM0->_0_LOAD + 1;

    uint32_t cmpA = (load * (100 - percent)) / 100;
    uint32_t cmpB = (load * (100 - percent)) / 100;

    // Update both A and B compare registers in Generator 0
    PWM0->_0_CMPA = cmpA;   // PB6 (M0PWM0)
    PWM0->_0_CMPB = cmpB;   // PB7 (M0PWM1)
}

void set_motor_direction(uint8_t dir) {
    if (dir == DIR_FORWARD) {
        // === Motor A (Left) ===
        GPIO_PORTA_DATA_R |=  (1U << 3);
        GPIO_PORTA_DATA_R &= ~(1U << 4);
        // === Motor B (Right) ===
				GPIO_PORTD_DATA_R &= ~(1U << 6);
        GPIO_PORTD_DATA_R |=  (1U << 7);
    } 
    else if (dir == DIR_REVERSE) {
        // === Motor A (Left) ===
        GPIO_PORTA_DATA_R &= ~(1U << 3);
        GPIO_PORTA_DATA_R |=  (1U << 4);
        // === Motor B (Right) ===
        GPIO_PORTD_DATA_R |=  (1U << 6);
        GPIO_PORTD_DATA_R &= ~(1U << 7);
    }
}

float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void Task_motor(void) {
		while (1) {
				
				if (output > 0) {
						set_motor_direction(DIR_REVERSE);
						pwm_set_duty_cycle(clamp(output, 0, 100)); // Clamp to 0–100%
				} else {
						set_motor_direction(DIR_FORWARD);
						pwm_set_duty_cycle(clamp(-output, 0, 100));
				}
				/*
				char buf[64];
				snprintf(buf, sizeof(buf),
						"Output: %.2f\r\n", output);
				Logger_log(buf);
				*/
				/*		
				set_motor_direction(DIR_FORWARD);
				pwm_set_duty_cycle(60); // 60% speed
				Logger_log("Motor 60% Forward\r\n");
				BSP_delay(BSP_TICKS_PER_SEC * 20U);// wait some time...
				
				pwm_set_duty_cycle(20); // slow down
				Logger_log("Motor 20% Forward\r\n");
				BSP_delay(BSP_TICKS_PER_SEC * 20U);// wait some time...
				
				// change direction
				set_motor_direction(DIR_REVERSE);
				pwm_set_duty_cycle(80); // reverse fast
				Logger_log("Motor 80% Reverse\r\n");
				BSP_delay(BSP_TICKS_PER_SEC * 20U);// wait some time...
				*/
				BSP_delay(1);
		}
}