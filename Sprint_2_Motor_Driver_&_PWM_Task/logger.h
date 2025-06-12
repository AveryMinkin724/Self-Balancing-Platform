#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>

void logger_start(void);
void Logger_log(const char *msg);
void Task_Logger(void);

#endif // LOGGER_H
