#ifndef SERIAL_UTIL_H
#define SERIAL_UTIL_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"

void serial_util_init();

bool serial_util_readline_nonblocking(char *buf, int buf_len, int *index_ptr, TickType_t timeout_ticks);

#endif