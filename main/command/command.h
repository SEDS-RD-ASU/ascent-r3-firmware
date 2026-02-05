#ifndef COMMAND_H
#define COMMAND_H

#include "stdint.h"
#include "esp_err.h"
#include "stdbool.h"
#include "stdatomic.h"

enum ascent_message_classes_t {
    PING,
    PONG,
    ACK,
    NAK,
    REBOOT,
    PYRO1,
    PYRO2,
    PYRO3,
    PYRO4,
    AUX_ON,
    AUX_OFF,
    SLEEP,
    WAKE_UP,
    TXLOCK_ON,
    TXLOCK_OFF,
    DUMP_CONFIG,
    EDIT_CONFIG,
    DUMP_RADIO,
    EDIT_RADIO,
    SIMULATOR_ON,
    VOLTAGE,
    TELEMETRY,
    ERASE_FLASH
};

esp_err_t enable_txlock(void);

esp_err_t process_command(uint8_t msg_class);

bool is_tx_lock();

#endif