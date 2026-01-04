#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_config.h"

/**
 * @file flight.h
 * @brief Flight state machine interface.
 */

/**
 * @brief Enumerates the possible flight states.
 */
enum FlightState
{
    FS_ON_PAD = 0,       /**< Vehicle is idle on the pad. */
    FS_BOOSTER,          /**< Booster motor is firing. */
    FS_COAST_BOOSTER,    /**< Booster burnout, coasting. */
    FS_SUSTAINER,        /**< Sustainer motor burn. */
    FS_COAST_SUSTAINER,  /**< Sustainer burnout, coasting. */
    FS_UNDER_DROGUES,    /**< Drogue parachutes deployed. */
    FS_UNDER_MAINS,      /**< Main parachutes deployed. */
    FS_LANDED,           /**< Vehicle has landed. */
    FS_PREFLIGHT,        /**< Low-power preflight state. */
};

/**
 * @brief Initialize flight configuration, loading persisted values if present.
 */
void flight_config_init(void);

/**
 * @brief Advance the flight state machine using current sensor readings.
 *
 * @param barometric_agl Altitude above ground level from the barometer.
 * @param barometric_velocity Vertical velocity from the barometer.
 * @param average_barometric_velocity Averaged vertical velocity for filtering.
 * @param raw_vertical_acl Raw vertical acceleration reading.
 * @return true if the flight state changed during this update, false otherwise.
 */
bool flight_update(
    float barometric_agl,
    float barometric_velocity,
    float average_barometric_velocity,
    float raw_vertical_acl
);

/**
 * @brief Get the current flight state code.
 */
uint8_t get_flight_state(void);

/**
 * @brief Get a human-readable name for the current flight state.
 */
const char* get_flight_state_name(void);

/**
 * @brief Print the provided flight configuration values for debugging.
 */
void print_flight_config(flight_config_t cfg);

#endif
