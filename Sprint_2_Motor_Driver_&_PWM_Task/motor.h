#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

void motor_init(void);
void motor_start(void);
void Task_motor(void);

#endif // MOTOR_H