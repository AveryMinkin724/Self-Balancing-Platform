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

#endif // BLUETOOTH_H