#include "serial_util.h"

void serial_util_init() {
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
}

bool serial_util_readline_nonblocking(char *buf, int buf_len, int *index_ptr, TickType_t timeout_ticks)
{
    uint8_t ch;
    int idx = *index_ptr;

    while (idx < buf_len - 1) {
        int len = usb_serial_jtag_read_bytes(&ch, 1, timeout_ticks);

        if (len == 0) {
            *index_ptr = idx;
            return false;  // timeout, incomplete line
        }

        if (ch == '\n' || ch == '\r') {
            buf[idx] = '\0';
            *index_ptr = 0;
            return true;  // full line read
        }

        if (ch != '\r') {
            buf[idx++] = ch;
        }
    }

    buf[buf_len - 1] = '\0';
    *index_ptr = 0;
    return false;  // overflow or bad data
}