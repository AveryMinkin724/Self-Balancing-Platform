#include "logger.h"
#include "uart.h"
#include "miros.h"
#include <string.h>

#define LOG_BUFFER_SIZE 512

static char logBuffer[LOG_BUFFER_SIZE];
static volatile uint16_t head = 0;
static volatile uint16_t tail = 0;

uint32_t stack_Logger[80];
OSThread LoggerThread;

void Logger_init(void) {
    OSThread_start(&LoggerThread,
                   &Task_Logger,
                   stack_Logger, sizeof(stack_Logger));
}

// Safe enqueue function
void Logger_log(const char *msg) {
    while (*msg) {
        uint16_t next = (head + 1) % LOG_BUFFER_SIZE;
        if (next != tail) {  // not full
            logBuffer[head] = *msg++;
            head = next;
        } else {
            // Buffer full — could drop or block; currently drop
            break;
        }
    }
}

// Logger task (runs forever)
void Task_Logger(void) {
    while (1) {
        if (tail != head) {
            Tiva_UART5_Transmitter_polling(logBuffer[tail]);  // Send 1 char
            tail = (tail + 1) % LOG_BUFFER_SIZE;
        }
    }
}
