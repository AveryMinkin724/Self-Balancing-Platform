#ifndef PID_H
#define PID_H

#include "imu.h"

extern float error;
extern float integral;
extern float prev_error;
extern volatile float output;
extern float Kp, Ki, Kd;
extern volatile float output;  // Declaration for use in other files


float PID_update(float, float);
	
#endif // PID_H