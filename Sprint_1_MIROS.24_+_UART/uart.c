#include "uart.h"
#include "tm4c.h"   // changed from t4mc.h to access gpio registers defined in structs rather than the other way

void uart_init(void)
{
		//TODO ASSERT(!Initialized) check if uart_init() has been called yet
	
		//Enable UART Module 5 clock and GPIO port E clock lines (UART 5 uses gpio port E)
		SYSCTL_RCGCUART_R |= (1U << 5); //Writting to RCGC1 register also writes to RCGCUART Reg? pg 344
		SYSCTL_RCGCGPIO_R |= (1U << 4);
		//Set bits in GPIO Alternate Function Select reg to mux GPIO pins PE5 & PE4 to UART_5 TX and RX
		GPIO_PORTE_AFSEL_R |= (1U << 4) | (1U << 5);
		// Set 2mA drive strength for PE4, PE5. 4mA & 8mA are other option, chatgpt said to set 2mA
		GPIO_PORTE_DR2R_R |= (1U << 4) | (1U << 5);  // N/A: this should be 2mA by default (4 and 5 bits correct?)
		//Assign UART signals to appropriate pins (pg688). GPIO Port Control (PCTL) register needs to be set to link the correct function (UART in this case) to the pins
		GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFF00FFFF) | 0x00110000; //remember to clear 8 bits for pins 4 & 5 with (GPIO_PORTE_PCTL_R & 0xFF00FFFF)
		
		GPIO_PORTE_DEN_R |= (1U << 4) | (1U << 5);  // Digital enable
		GPIO_PORTE_AMSEL_R &= ~((1U << 4) | (1U << 5));  // Disable analog function
		
		// Disable UART before configuring
		UART5_CTL_R &= ~0x01;  
		//UART Clock Config (baud clock), chose system clock: 16MHz default
		UART5_CC_R = 0;
		// for 115200, Baud Rate Divisor 8.680555 (notice these are dedicated registers for the buad rates, you can overwrite what is in these so = suffices
		UART5_IBRD_R = 8; //integer part
		UART5_FBRD_R = 44; //fractional Part: 0.68055*64 +.5 = 44: (44/64) fractiono of lower 6 bits
		//LCRH : data length 8-bit, not parity bit, no FIFO
		UART5_LCRH_R = 0x60;
		// Enable UART, HSE cleared (bit 5), TXE & RXE set
		UART5_CTL_R = 0x301; //|= (1U << 0) | (1U << 8) | (1U << 9); 0b1100000001
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