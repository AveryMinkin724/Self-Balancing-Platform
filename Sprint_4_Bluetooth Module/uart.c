#include "uart.h"

#define UART_MODE_USB 1
#define UART_MODE_BT  2

#define UART_MODE UART_MODE_USB  // Change this line to switch between USB and BT

/* 115200 Baude rate */
#define USB_BAUD_I 8  // Baud Rate Divisor 8.680555
#define USB_BAUD_F 44 // fractional Part: 0.68055*64 +.5 = 44: (44/64) fraction of lower 6 bits
/* 9600 Baude rate */
#define BT_BAUD_I 104 // 104.16667 usec
#define BT_BAUD_F 11 // 0.16667*64 + 0.5 = 11.1668

extern void UART5_IRQHandler(void);

void diag_vectors(void) {
    uint32_t vtor = SCB->VTOR;   // base of active vector table
    uint32_t idx  = 16 + 61;     // UART5 exception number (IRQn + 16)
    uint32_t *table = (uint32_t *)vtor;
    uint32_t isr_addr = table[idx];

    Logger_log_hex("VTOR", vtor);
    Logger_log_hex("UART5 vector entry", isr_addr);
    Logger_log_hex("&UART5_IRQHandler", (uint32_t)&UART5_IRQHandler);
}


void uart_init(void)
{
		//TODO ASSERT(!Initialized) check if uart_init() has been called yet
	
		// 1. Enable UART Module 5 and GPIO port E clock lines (UART 5 uses gpio port E)
		SYSCTL_RCGCUART_R |= (1U << 5); // UART5  Writting to RCGC1 register also writes to RCGCUART Reg? pg 344
		SYSCTL_RCGCGPIO_R |= (1U << 4); // GPIOE
		
		// 2. Configure GPIOE pins for UART function (AF, digital, etc.)Set bits in GPIO Alternate Function Select reg to mux GPIO pins PE5 & PE4 to UART_5 TX and RX
		GPIO_PORTE_AFSEL_R |= (1U << 4) | (1U << 5);     // Alternate Function for PE4 & PE5
		GPIO_PORTE_DR2R_R |= (1U << 4) | (1U << 5);      // Set 2mA drive strength for PE4, PE5. 4mA & 8mA are other option, chatgpt said to set 2mA // N/A: this should be 2mA by default (4 and 5 bits correct?) 
		GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFF00FFFF) | 0x00110000; // Assign UART signals to appropriate pins (pg688). GPIO Port Control (PCTL) register needs to be set to link the correct function (UART in this case) to the pins//remember to clear 8 bits for pins 4 & 5 with (GPIO_PORTE_PCTL_R & 0xFF00FFFF)
		GPIO_PORTE_DEN_R |= (1U << 4) | (1U << 5);       // Digital enable
		GPIO_PORTE_AMSEL_R &= ~((1U << 4) | (1U << 5));  // Disable analog function	
		
		// 3. Configure UART baud rate, word length, etc.
		UART5_CTL_R &= ~0x01;  // Disable UART before configuring
		UART5_CC_R = 0;        // UART Clock Config (baud clock), chose system clock: 16MHz default
		
		if (UART_MODE == UART_MODE_USB) {
				UART5_IBRD_R = USB_BAUD_I; 
				UART5_FBRD_R = USB_BAUD_F; 
		} else if (UART_MODE == UART_MODE_BT) {
				UART5_IBRD_R = BT_BAUD_I; 
				UART5_FBRD_R = BT_BAUD_F;
		} else {
				UART5_IBRD_R = 0; 
				UART5_FBRD_R = 0;
		}
		
		// 4. Enable UART RX interrupt
		UART5_ICR_R = (1 << 4); // Clear any pending interrupts
		UART5_IM_R |= (UART_IM_RXIM | UART_IM_RTIM); // Enable RX & RX timeout interrupts
		
		// 5. Enable UART5 interrupt in NVIC
    NVIC_EN1_R |= (1 << (61-32));  // UART5 = IRQ#61 ? EN1, bit 29 (78 in vector table?)
		
		//LCRH : data length 8-bit, not parity bit, no FIFO
		UART5_LCRH_R = 0x60;
		// Enable UART, HSE cleared (bit 5), TXE & RXE set
		UART5_CTL_R = 0x301; //|= (1U << 0) | (1U << 8) | (1U << 9); 0b1100000001
		
		
		diag_vectors();

		__enable_irq();
}

void Tiva_UART5_Transmitter_polling(char c)
{
		// Blocking implementation
		while((UART5_FR_R & (1U << 5)) != 0); /* wait until Tx buffer not full */
		UART5_DR_R = c;                  /* before giving it another byte */
}

char Tiva_UART5_Receiver_polling(void)
{
		char data;
		while((UART5_FR_R & (1U << 4)) != 0); /* wait until Rx buffer not empty */
		data = UART5_DR_R;                  /* before giving it another byte */
		return data;
}

void UART5_Transmit_string(char *str)
{
		while (*str) {
				Tiva_UART5_Transmitter_polling(*(str++));
		}
}