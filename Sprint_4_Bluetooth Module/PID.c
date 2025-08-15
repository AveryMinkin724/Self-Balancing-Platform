#include "PID.h"

float error = 0.0f;
float integral = 0.0f;
float prev_error = 0.0f;
volatile float output = 0.0f; // Global definition
float Kp = 1.0f;
float Ki = 10.0f;
float Kd = 10.0f; 

float PID_update(float current_pitch, float dt) {
		float target = -90.0f; //desired pitch
		error = target - current_pitch;
	
		integral += error * dt;
		float derivative = (error - prev_error) / dt;
				
		output  = (Kp * error) + (Ki * integral) + (Kd * derivative);
		
		prev_error = error;
				
		return output;
}
