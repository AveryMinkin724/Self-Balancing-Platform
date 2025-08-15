#include "bluetooth.h"

uint32_t stack_bt_command[128U];
OSThread btCommandThread;

void bt_command_start(void) {
    OSThread_start(&btCommandThread,
                   &Task_bt_command,
                   stack_bt_command, sizeof(stack_bt_command));
}

void Task_bt_command(void) {
		while (1) {
        while (!command_ready) {
            OS_sched();  // ?? Wait/yield until a command is ready
        }

        // Command is ready
        command_ready = false;  // ?? Reset flag

        /*float kp_new, ki_new, kd_new;
        if (strncmp(uart_rx_buffer, "SET", 3) == 0) {
            if (sscanf(uart_rx_buffer, "SET %f %f %f", &kp_new, &ki_new, &kd_new) == 3) {
                Kp = kp_new;
                Ki = ki_new;
                Kd = kd_new;
							
								char buf[128];
								snprintf(buf, sizeof(buf),
									"New Ks: Kp: %.2f Ki: %.2f Kd: %.2f\r\n", Kp, Ki, Kd);
								Logger_log(buf);
            }
        }*/
				char buf[128];
				snprintf(buf, sizeof(buf), "RX: %s\r\n", uart_rx_buffer);
				Logger_log(buf);
    }
}