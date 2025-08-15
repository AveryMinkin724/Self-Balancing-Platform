#ifndef UART_H
#define UART_H

#include "tm4c.h"
#include "bsp.h"
#include "TM4C123GH6PM.h"
#include "core_cm4.h"  // or define SCB yourself
#include "logger.h"

void uart_init(void);
void Tiva_UART5_Transmitter_polling(char c);
char Tiva_UART5_Receiver_polling(void);
void UART5_Transmit_string(char *str);

#endif //UART_H
