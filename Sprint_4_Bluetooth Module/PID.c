#include "PID.h"

float error = 0.0f;
float integral = 0.0f;
float prev_error = 0.0f;
volatile float output = 0.0f; // Global definition
float Kp = 15.0f; //9.0f; //8.25
float Ki = 0.0f; //3.3f;
float Kd = 0.0f; //0.11f; //0.3
static float derivative_filtered = 0.0f;

float PID_update(float current_pitch, float dt) {
		float target = -82.75f; //desired pitch
		error = target - current_pitch;

    // Normal PID
    integral += error * dt;
    // Derivative (raw)
    float derivative_raw = (error - prev_error) / dt;

    // Low-pass filter
    const float alpha = 0.997f;  // try 0.85–0.95
    derivative_filtered = alpha * derivative_filtered 
                        + (1.0f - alpha) * derivative_raw;

    float output = Kp * error 
                 + Ki * integral 
                 + Kd * derivative_filtered;

    prev_error = error;
		
		/*
		char buf[128];
						snprintf(buf, sizeof(buf), "%.2f\r\n", output);
						Logger_log(buf);
		*/
    return output;
}
