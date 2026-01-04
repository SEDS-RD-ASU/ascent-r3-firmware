#ifndef SERIAL_UTIL_H
#define SERIAL_UTIL_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"

/**
 * @brief Initialize the USB serial interface helpers.
 */
void serial_util_init();

/**
 * @brief Read a line from the serial port without blocking the caller.
 *
 * @param buf Destination buffer for the received characters.
 * @param buf_len Length of the destination buffer.
 * @param index_ptr Pointer to the current index within the buffer (updated in place).
 * @param timeout_ticks Maximum ticks to wait for data before returning.
 * @return true if a full line ending with a newline was read, false otherwise.
 */
bool serial_util_readline_nonblocking(char *buf, int buf_len, int *index_ptr, TickType_t timeout_ticks);

#endif
