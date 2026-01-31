#ifndef PID_H
#define PID_H

#include "imu.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

extern float error;
extern volatile float output;
extern float Kp, Ki, Kd;
extern volatile float output;  // Declaration for use in other files

float clamp(float, float, float);
float PID_update(float, float);
void PID_reset(void);
	
typedef struct {
    float integral;
    float prev_error;
} PID_t;

extern PID_t pid;

#endif // PID_H