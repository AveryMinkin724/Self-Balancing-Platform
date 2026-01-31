#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "miros.h"
#include "PID.h"
#include "bsp.h"

//void bt_command_init(void);
void bt_command_start(void);
void Task_bt_command(void);
extern volatile bool red_led_active;
extern uint32_t red_led_off_time;

#endif // BLUETOOTH_H