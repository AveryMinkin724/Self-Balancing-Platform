#include "PID.h"

float error = 0.0f;
volatile float output = 0.0f; // Global definition
float Kp = 11.3f; //9.0f; 
float Ki = 0.45f; //2.0f;
float Kd = 1.10f; //0.3; Deadband = 0.5f
static float derivative_filtered = 0.0f;
float prev_output = 0.0f;
PID_t pid;

float PID_update(float current_pitch, float dt) {
		float target = -82.50f; //desired pitch
		error = target - current_pitch;

    // Normal PID
   
    // Derivative (raw)
    //float derivative_raw = (error - pid.prev_error) / dt;
	
    // Low-pass filter
    const float alpha = 0.90f;  // try 0.85–0.95
    derivative_filtered = alpha * derivative_filtered 
                        + (1.0f - alpha) * gyro_rate_y;

    float output = Kp * error 
                 + Ki * pid.integral 
                 + Kd * derivative_filtered;
		
		// limit change per iteration
    float delta = output - prev_output;
    delta = clamp(delta, -3, 3);
    output = prev_output + delta;
		
		if(delta > -3 && delta < 3 ) {
				pid.integral += error * dt;
		}
		
		prev_output = output;
		pid.prev_error = error;
		/*
		char buf[128];
						snprintf(buf, sizeof(buf), "%.2f\r\n", output);
						Logger_log(buf);
		*/
    return output;
}

void PID_reset(void) {
		pid.integral = 0.0f;
		pid.prev_error = 0.0f;
}