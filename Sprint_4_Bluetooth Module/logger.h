#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

void logger_start(void);
void Logger_log(const char *msg);
void Logger_log_int(const char *prefix, int value);
void Logger_log_hex(const char *prefix, uint8_t value);
void Task_Logger(void);

#endif // LOGGER_H
