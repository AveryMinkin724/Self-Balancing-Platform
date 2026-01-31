#include <stdint.h> // C99 standard integers
#include "miros.h" 
#include "bsp.h"
#include "uart.h"
#include "logger.h"
#include "motor.h"
#include "imu.h"
#include "bluetooth.h"

uint32_t stack_LEDHeartbeat[128]; //initialize array for stack, this will automatically be assigned to area of RAM
OSThread LEDHeartbeat;
void Task_LEDHeartbeat(void) {		
		while (1) {
				__disable_irq();
				/* Keeps a green LED blinking to indicate system alive */
        BSP_ledGreenOn();
				//Logger_log("Green On\r\n");
        BSP_delay(BSP_TICKS_PER_SEC / 4U);
        BSP_ledGreenOff();
				//Logger_log("Green Off\r\n");
        BSP_delay(BSP_TICKS_PER_SEC * 4U);
				__enable_irq();
    }
}

uint32_t stack_ControlLoop[40];
OSThread ControlLoop;
void Task_ControlLoop(void) {
		while (1) {
				/* Placeholder for balance control logic (PID, etc.) */
		}
}

/* background code: sequential with blocking version */
int main(void) {
		BSP_init();
		OS_init();
    
		/* start UART Logging thread */
    logger_start(); 
	
		/* fabricate Cortex-M ISR stack frame for LEDHeartbeat */
		OSThread_start(&LEDHeartbeat,
									 &Task_LEDHeartbeat,
									 stack_LEDHeartbeat, sizeof(stack_LEDHeartbeat));
		
		/* fabricate Cortex-M ISR stack frame for ControlLoop */
		/*OSThread_start(&ControlLoop,
									 &Task_ControlLoop,
									 stack_ControlLoop, sizeof(stack_ControlLoop));
		*/
		/* start Motor thread */
		motor_start();
		
		/* start IMU thread */
		imu_start();
		
		/* start Bluetooth thread */
		bt_command_start();
				
		/* transfer control to the RTOS to run the threads */
    OS_run();
		
	
	//return 0;
}