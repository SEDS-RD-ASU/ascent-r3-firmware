#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_config.h"

enum FlightState
{
    FS_ON_PAD = 0,
    FS_BOOSTER,
    FS_COAST_BOOSTER,
    FS_SUSTAINER,
    FS_COAST_SUSTAINER,
    FS_UNDER_DROGUES,
    FS_UNDER_MAINS,
    FS_LANDED,
    FS_PREFLIGHT,
};

// returns if the state changed or not

void flight_config_init(void);

bool flight_update(
    float barometric_agl,
    float barometric_velocity,
    float average_barometric_velocity,
    float raw_vertical_acl
);

uint8_t get_flight_state(void);

const char* get_flight_state_name(void);

void print_flight_config();

#endif