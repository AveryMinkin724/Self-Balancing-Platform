#include "PID.h"

float error = 0.0f;
float integral = 0.0f;
float prev_error = 0.0f;
volatile float output = 0.0f;
float Kp = 10.0f;
float Ki = 0.0f;
float Kd = 0.5f;

float PID_update(float current_pitch, float dt, float Kp, float Ki, float Kd) {
		float target = 85.0f; //desired pitch
		error = target - current_pitch;
	
		integral += error * dt;
		float derivative = (error - prev_error) / dt;
		
		char buf[128];
				snprintf(buf, sizeof(buf),
						"dt: %.2f\r\n", dt);
				Logger_log(buf);
				
		float output  = (Kp * error) + (Ki * integral) + (Kd * derivative);
		
		prev_error = error;
		
		char buff[128];
				snprintf(buff, sizeof(buff),
						"output: %.2f\r\n", output);
				Logger_log(buff);
				
		return output;
}
