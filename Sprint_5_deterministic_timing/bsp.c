/* Board Support Package (BSP) for the EK-TM4C123GXL board */
#include <stdint.h>  /* Standard integers. WG14/N843 C99 Standard */

#include "bsp.h"
#include "miros.h"
#include "qassert.h"
#include "TM4C123GH6PM.h" /* the TM4C MCU Peripheral Access Layer (TI) */
#include "uart.h"
#include "motor.h"
#include "imu.h"
#include "core_cm4.h"

/* on-board LEDs */
#define LED_RED   (1U << 1)
#define LED_BLUE  (1U << 2)
#define LED_GREEN (1U << 3)
#define TEST_PIN  (1U << 4)

static uint32_t volatile l_tickCtr;
char uart_rx_buffer[64];
volatile int rx_index = 0;
volatile bool command_ready = false;
void Timer0A_IRQHandler(void);

void SysTick_Handler(void) {
    GPIOF_AHB->DATA_Bits[TEST_PIN] = TEST_PIN;

    ++l_tickCtr;

    __disable_irq();
    OS_sched();					//this is critical for making RTOS preemptive so tasks can run concurrently
    __enable_irq();

    GPIOF_AHB->DATA_Bits[TEST_PIN] = 0U;
}

void Timer0A_IRQHandler(void) {
    TIMER0_ICR_R = 0x1;   // Clear interrupt
    const float dt = 0.002f;

    current_pitch = Complementary_Filter(dt);
    output = PID_update(current_pitch, dt);
}


void Clock_init(void) {
		/* Force Config Clock 16MHz*/
		SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;      // Use RCC2
		SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;      // Bypass PLL while configuring
		SYSCTL_RCC_R &= ~SYSCTL_RCC_USESYSDIV;     // No system clock divider
		SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;        // Clear XTAL field
		SYSCTL_RCC_R |= SYSCTL_RCC_XTAL_16MHZ;     // Set crystal value to 16MHz
		SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;   // Clear oscillator source
		SYSCTL_RCC2_R |= SYSCTL_RCC2_OSCSRC2_MO;   // Use Main Oscillator
}

void ControlTimer_Init(void) {
    SYSCTL_RCGCTIMER_R |= (1U << 0);   // Enable Timer0
    while (!(SYSCTL_PRTIMER_R & (1U << 0)));

    TIMER0_CTL_R = 0;                  // Disable timer
    TIMER0_CFG_R = 0x0;                // 32-bit timer
    TIMER0_TAMR_R = 0x2;               // Periodic mode

    // 16 MHz / 1000 = 16000 counts
    TIMER0_TAILR_R = 32000 - 1;

    TIMER0_ICR_R = 0x1;                // Clear timeout flag
    TIMER0_IMR_R |= 0x1;               // Enable timeout interrupt

    NVIC_EnableIRQ(TIMER0A_IRQn);
		
		// DO NOT enable timer yet
}

void ControlTimer_Start(void) {
		TIMER0_CTL_R |= 0x1;               // Enable timer
}
void BSP_init(void) {
		
		/* Configure Clock since some part of the startup seems to select clock that is not 16MHz (system_TM4C123GH6PM.c?) */
		Clock_init();
    
		/* For LEDs */
	
		SYSCTL->GPIOHBCTL |= (1U << 5); /* enable AHB for GPIOF */
    SYSCTL->RCGCGPIO  |= (1U << 5); /* enable Run Mode for GPIOF */

    GPIOF_AHB->DIR |= (LED_RED | LED_BLUE | LED_GREEN | TEST_PIN);
    GPIOF_AHB->DEN |= (LED_RED | LED_BLUE | LED_GREEN | TEST_PIN);
		
		/* For UART */
	
		//stop watchdog timer
		SYSCTL_RCGCWD_R &= ~(1U << 0); // Disable WDT0 clock
		SYSCTL_RCGCWD_R &= ~(1U << 1); // Disable WDT1 clock

		//Initialize UART (section 14.4 datasheet)
		uart_init();
	
		/* For Motor Driver */
		motor_init();
		
		/* For IMU / I2C initialization */
		imu_init();
		
		/* For Deterministic Timing of IMU task */
		ControlTimer_Init();   // ADD HERE
		
		/* Enable FPU */
		SCB->CPACR |= (0xF << 20);   // enable CP10 + CP11 (FPU)
		__DSB();
		__ISB();
}

uint32_t BSP_tickCtr(void) {
    uint32_t tickCtr;

    __disable_irq();
    tickCtr = l_tickCtr;
    __enable_irq();

    return tickCtr;
}

void BSP_delay(uint32_t ticks) {
    uint32_t start = BSP_tickCtr();
    while ((BSP_tickCtr() - start) < ticks) {
    }
}

void UART5_IRQHandler(void) {
    BSP_ledRedOn();
		BSP_ledRedOff();
	 
		if (UART5_MIS_R & (UART_MIS_RXMIS | UART_MIS_RTMIS)) {
        char c = (char)(UART5_DR_R & 0xFF);  // Read from UART
        UART5_ICR_R = UART_ICR_RXIC | UART_ICR_RTIC;  // clear both

        if (rx_index < sizeof(uart_rx_buffer) - 1) {
            if (c == '\n' || c == '\r') {
                uart_rx_buffer[rx_index] = '\0';  // Null terminate
                command_ready = true;       // Flag to signal command is ready
                rx_index = 0;               // Reset for next line
            } else {
                uart_rx_buffer[rx_index++] = c;  // Store char in buffer
            }
        } else {
            rx_index = 0; // buffer overflow prevention
        }
    }
		
    QK_ISR_EXIT();  // ?? Call RTOS scheduler after ISR
		//OS_sched();       //should not do this inside IRQ      /* call the scheduler */ \
    //__enable_irq();  //should not do this inside IRQ
}

void HardFault_Handler(void) {
    __BKPT(0);   // stop here and inspect registers
    while (1);
}
void BSP_ledRedOn(void) {
    GPIOF_AHB->DATA_Bits[LED_RED] = LED_RED;
}

void BSP_ledRedOff(void) {
    GPIOF_AHB->DATA_Bits[LED_RED] = 0U;
}

void BSP_ledBlueOn(void) {
    GPIOF_AHB->DATA_Bits[LED_BLUE] = LED_BLUE;
}

void BSP_ledBlueOff(void) {
    GPIOF_AHB->DATA_Bits[LED_BLUE] = 0U;
}

void BSP_ledGreenOn(void) {
    GPIOF_AHB->DATA_Bits[LED_GREEN] = LED_GREEN;
}

void BSP_ledGreenOff(void) {
    GPIOF_AHB->DATA_Bits[LED_GREEN] = 0U;
}

void OS_onStartup(void) {
    SystemCoreClockUpdate();
	
    /* 1 ms RTOS tick */
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    /* Interrupt priorities (lower number = higher priority) */
    NVIC_SetPriority(TIMER0A_IRQn, 0U);   // Control loop (highest)
    NVIC_SetPriority(SysTick_IRQn, 1U);   // RTOS tick
    NVIC_SetPriority(UART5_IRQn, 6U);     // Logging
}

//............................................................................
_Noreturn void Q_onAssert(char const * const module, int const id) {
    (void)module; // unused parameter
    (void)id;     // unused parameter
#ifndef NDEBUG
    // light up all LEDs
    GPIOF_AHB->DATA_Bits[LED_GREEN | LED_RED | LED_BLUE] = 0xFFU;
    // for debugging, hang on in an endless loop...
    for (;;) {
    }
#endif
    NVIC_SystemReset();
}
//............................................................................
_Noreturn void assert_failed(char const * const module, int const id);
_Noreturn void assert_failed(char const * const module, int const id) {
    Q_onAssert(module, id);
}
