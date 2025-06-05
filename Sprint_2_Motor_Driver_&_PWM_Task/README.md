# Sprint 2

✅ Sprint 2: Motor Driver and PWM Task
Goal: Control DC motors using PWM signals from RTOS tasks.
Materials/Equipment:


L298N motor driver module or similar (~$8)


2x DC motors (e.g., yellow gear motors) (~$5–10)


Jumper wires, breadboard


Background Knowledge:


PWM generation


Motor driver H-bridge basics


Estimated Time: 8–12 hours (2–3 weeks)


Task Breakdown:


Write a PWM control task


Interface motor driver


Test variable speed output



✅ Sprint 2: Motor Driver and PWM Task — Step-by-Step
🎯 Goal:
Implement PWM control via an RTOS task to drive DC motors through an H-bridge (e.g., L298N).

🧰 Materials Needed
Item	Notes	Required For
L298N Motor Driver Module	Or any similar dual H-bridge	Hardware testing
2x DC motors (6–12V typical)	Yellow gear motors are common	Hardware testing
External power supply (e.g., 9V battery pack or bench PSU)	Needed to drive motors	Hardware testing
Jumper wires & Breadboard	For prototyping connections	Hardware testing
Multimeter	(optional) to test PWM voltage	Hardware debugging

✅ Order these now if you don’t have them.

🧪 Software-Only Tasks (Can Start Now)
1. Define PWM abstraction
Decide how you'll control PWM from a task. If using Tiva C timers:

Use Timer0A or another GPTM in PWM mode.

Choose pins tied to timers capable of PWM output (e.g., PB6, PB7 for T0CCP0/1).

📄 Reference: Tiva TM4C123 datasheet > Pin Muxing & Timer PWM modes.

2. Write PWM Setup Code
Implement a function like:

c
Copy
Edit
void PWM_Init(void);
void PWM_SetDutyCycle(uint8_t dutyPercent);  // 0-100%
Use a 1 kHz base frequency for now (configurable).

3. Create RTOS Task to Control PWM
Example task pseudocode:

c
Copy
Edit
void Task_MotorPWM(void) {
    uint8_t duty = 0;
    int8_t dir = 1;
    while (1) {
        PWM_SetDutyCycle(duty);
        duty += dir * 10;
        if (duty == 100 || duty == 0) dir *= -1;
        OS_delay(BSP_TICKS_PER_SEC / 10); // update every 100 ms
    }
}
4. Update main.c
Create stack space and start the MotorPWM thread.

Make sure it's cooperative with your other tasks or prioritize accordingly.

🔌 Hardware-Dependent Tasks (Do After Parts Arrive)
5. Wire the L298N to the Tiva C & Motors
Typical wiring:

L298N Pin	Connects To
IN1 / IN2	GPIO outputs on Tiva
ENA	PWM output from Tiva
12V / GND	External power supply
OUT1/OUT2	Motor A terminals
5V_EN	Shorted (if powering logic from 12V rail)

🛑 Don’t power the motors from Tiva’s 3.3V rail!

6. Test PWM Output with a Multimeter or LED
Before wiring up the motor, connect the PWM pin to an LED + resistor.

Verify dimming/brighting with changing duty cycle.

7. Test Full Motor Control
Use MotorPWM task to ramp up/down speed.

Add direction toggling via GPIO control of IN1/IN2.

✅ Sprint 2 "Done" Criteria:
PWM output visibly changes duty cycle.

Motor connected to L298N spins and varies speed.

RTOS task cleanly controls speed in time slices.
