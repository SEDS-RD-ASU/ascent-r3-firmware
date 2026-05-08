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
    ERASE_FLASH,
    REQ_TELEMETRY
};

typedef struct {
    uint32_t timestamp;        // 4 bytes
    int32_t latitude;          // 4 bytes
    int32_t longitude;         // 4 bytes
    float altitude_agl;        // 4 bytes
    float vertical_velocity;   // 4 bytes
    float y_acc;               // 4 bytes
    float gyr_y;               // 4 bytes
    uint8_t pyro_state;        // 1 byte
    uint8_t sats;              // 1 byte
    uint8_t flight_state;      // 1 byte
    uint16_t battery_voltage;  // 2 bytes
} ascent_telemetry_t;

esp_err_t enable_txlock(void);
esp_err_t disable_txlock(void);

esp_err_t process_command(uint8_t msg_class);

bool is_tx_lock();

void queueLatestTelemetry(ascent_telemetry_t *payload);

void peekLatestTelemetry(ascent_telemetry_t *payload);

uint8_t next_sequence_id(void);

void initialize_telemetry_queue(void);

#endif