#include "motor.h"

#define DIR_FORWARD 0
#define DIR_REVERSE 1

uint32_t stack_motor[128U];
OSThread motorThread;

extern volatile float output;

void motor_start(void) {
    OSThread_start(&motorThread,
                   &Task_motor,
                   stack_motor, sizeof(stack_motor));
}

void motor_init(void) {
		SYSCTL_RCGCGPIO_R |= (1U << 0) | (1U << 1); // Enable clock for GPIOA (0) and GPIOB (1)
		for (volatile int i = 0; i < 1000; i++) {}   // Delay for GPIO stabilization
	
		GPIO_PORTA_DIR_R |= (1U << 3) | (1U << 4);    // Set PA3, PA4 as output
		GPIO_PORTA_DEN_R |= (1U << 3) | (1U << 4);    // Digital enable for PA3, PA4

		SYSCTL->RCGCPWM |= (1 << 0);   					// Enable clock to PWM0 module
		for (volatile int i = 0; i < 1000; i++) {}   // Delay loop for peripheral clock to stabilize
		SYSCTL->RCC &= ~(1 << 20);  						// Clear USEPWMDIV bit. Ensure PWM uses system clock (default is OK, but for clarity)
		
		GPIO_PORTB_AFSEL_R |= (1 << 6);         // Enable alternate function on PB6
		GPIO_PORTB_PCTL_R &= ~(0xF << 24);      // Clear PCTL[27:24] for PB6
		GPIO_PORTB_PCTL_R |=  (0x4 << 24);      // Set PB6 to M0PWM0 (function 4)
		GPIO_PORTB_DEN_R  |=  (1 << 6);         // Enable digital function
		GPIO_PORTB_DIR_R  |=  (1 << 6);         // Set PB6 as output
		
		PWM0->_0_CTL &= ~0x01;         	// Disable Generator 0 while configuring
		PWM0-> _0_GENA = 0x0000008C;    // Set on LOAD, clear on CMPA down
		uint32_t pwm_period = 16000; 		// For 10kHz at 16 MHz system clock
		PWM0->_0_LOAD = pwm_period - 1;
		PWM0->_0_CMPA = pwm_period / 2; // 50% duty cycle
		PWM0->_0_CTL |= 0x01;         	// Enable Generator 0
		PWM0->ENABLE |= (1 << 0);     	// Enable output M0PWM0 (PB6)
}

void pwm_set_duty_cycle(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint32_t load = PWM0->_0_LOAD + 1; // +1 since LOAD = period - 1
    uint32_t cmpa = (load * (100 - percent)) / 100;
    PWM0->_0_CMPA = cmpa;
}

void set_motor_direction(uint8_t dir) {
    if (dir == DIR_FORWARD) {
        GPIO_PORTA_DATA_R |=  (1U << 3);
        GPIO_PORTA_DATA_R &= ~(1U << 4);
    } else {
        GPIO_PORTA_DATA_R &= ~(1U << 3);
        GPIO_PORTA_DATA_R |=  (1U << 4);
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
						set_motor_direction(DIR_FORWARD);
						pwm_set_duty_cycle(clamp(output, 0, 100)); // Clamp to 0–100%
				} else {
						set_motor_direction(DIR_REVERSE);
						pwm_set_duty_cycle(clamp(-output, 0, 100));
				}
				/*
				char buf[128];
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