#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include "stdbool.h"
#include "stdatomic.h"
#include "driver_pyro.h"

/**
 * @file flight_config.h
 * @brief Definitions for configurable flight parameters.
 */

//Dont commit w/ IS_SITL defined
// #define IS_SITL


// Note all values are in m or m/s

#define IS_BOOSTER

#define ENGINE_GS (9.81*3)
// #define ENGINE_GS (9.81*0.1)
#define APPO_GS (9.81/2)

#define MAINS_ALT (1000/3.28)

#define PANIC_VEL (-240.0f/3.28)

// ALL UNITS ARE IN METRIC SI UNITS
/**
 * @brief Tunable parameters used by the flight state machine.
 */
typedef struct {
    float lift_off_acceleration_threshold; /**< Minimum acceleration to register lift-off. */
    uint8_t lift_off_tick_count;           /**< Number of consecutive ticks required for lift-off. */
    float apogee_acceleration_threshold;   /**< Acceleration threshold indicating apogee. */
    uint8_t apogee_tick_count;             /**< Number of ticks to confirm apogee detection. */
    float burnout_acceleration_threshold;  /**< Threshold to detect motor burnout. */
    uint8_t burnout_tick_count;            /**< Ticks required to confirm burnout. */
    uint8_t recovery_burnout_counter;      /**< Burnout detections before staging recovery actions. */
    bool arm_at_boot;                      /**< Whether pyro channels arm automatically at boot. */
    pyro_channel_t Appo_Channel;           /**< Pyro channel used for apogee deployment. */
    pyro_channel_t Mains_Channel;          /**< Pyro channel used for main chute deployment. */
    pyro_channel_t Separation_Channel;     /**< Pyro channel used for stage separation. */
    pyro_channel_t Ignition_Channel;       /**< Pyro channel used for ignition. */
    pyro_channel_t Aux_1_Channel;          /**< Auxiliary pyro channel 1. */
    pyro_channel_t Aux_2_Channel;          /**< Auxiliary pyro channel 2. */
    pyro_channel_t Aux_3_Channel;          /**< Auxiliary pyro channel 3. */
    pyro_channel_t Aux_4_Channel;          /**< Auxiliary pyro channel 4. */
    float panic_velocity_threshold;        /**< Descent velocity that triggers a panic condition. */
    float main_deployment_altitude;        /**< AGL altitude to deploy main parachutes. */
    uint8_t main_deployment_tick_count;    /**< Ticks required before deploying mains. */
    float highest_ground_elevation;        /**< Highest observed ground elevation for AGL calculations. */
    float landed_velocity_threshold;       /**< Velocity threshold to mark landing. */
    uint8_t landed_tick_count;             /**< Ticks to confirm landing. */
} flight_config_t;

extern _Atomic flight_config_t flight_config;

// if this is defined then the board will erase and move to the next flash bank
// on any power up with atleast one continuous pyro, and will fake a txlock,
// signal this can be used to allow ascent to fly with out a ground station or
// two way coms to a ground station

#endif
