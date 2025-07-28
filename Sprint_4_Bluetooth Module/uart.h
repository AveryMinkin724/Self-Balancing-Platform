#ifndef UART_H
#define UART_H

#include "tm4c.h"

void uart_init(void);
void Tiva_UART5_Transmitter_polling(char c);
char Tiva_UART5_Receiver_polling(void);
void UART5_Transmit_string(char *str);

#endif //UART_H
