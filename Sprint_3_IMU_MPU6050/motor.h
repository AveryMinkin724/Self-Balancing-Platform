#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "uart.h"
#include "miros.h"
#include "TM4C123GH6PM.h"
#include "bsp.h"
#include "logger.h"
#include "PID.h"

void motor_init(void);
void motor_start(void);
void Task_motor(void);
float clamp(float, float, float);

#endif // MOTOR_H