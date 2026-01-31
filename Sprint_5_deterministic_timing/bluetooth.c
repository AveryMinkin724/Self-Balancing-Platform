#include "bluetooth.h"

uint32_t stack_bt_command[512U];
OSThread btCommandThread;
volatile bool red_led_active = false;
uint32_t red_led_off_time = 0;

void bt_command_start(void) {
    OSThread_start(&btCommandThread,
                   &Task_bt_command,
                   stack_bt_command, sizeof(stack_bt_command));
}

void Task_bt_command(void) {
    while (1) {

        while (!command_ready) {
            OS_sched();   // Yield
        }

        command_ready = false;

        bool gain_changed = false;

        // --- 2-character commands first ---
        if (strncmp(uart_rx_buffer, "PP", 2) == 0) {
            Kp += 1.0f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "pp", 2) == 0) {
            Kp -= 1.0f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "II", 2) == 0) {
            Ki += 1.0f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "ii", 2) == 0) {
            Ki -= 1.0f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "DD", 2) == 0) {
            Kd += 1.0f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "dd", 2) == 0) {
            Kd -= 1.0f;
            gain_changed = true;
        }

        // --- 1-character commands ---
        else if (strncmp(uart_rx_buffer, "P", 1) == 0) {
            Kp += 0.1f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "p", 1) == 0) {
            Kp -= 0.1f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "I", 1) == 0) {
            Ki += 0.1f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "i", 1) == 0) {
            Ki -= 0.1f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "D", 1) == 0) {
            Kd += 0.1f;
            gain_changed = true;
        }
        else if (strncmp(uart_rx_buffer, "d", 1) == 0) {
            Kd -= 0.1f;
            gain_changed = true;
        }

        // --- Feedback ---
        if (gain_changed) {
            BSP_ledRedOn();
            red_led_active = true;
            red_led_off_time = BSP_tickCtr() + (2 * BSP_TICKS_PER_SEC);

            char buf[128];
            snprintf(buf, sizeof(buf),
                     "Kp=%.2f Ki=%.2f Kd=%.2f\r\n", Kp, Ki, Kd);
            Logger_log(buf);
        }
    }
}
