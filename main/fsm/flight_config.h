#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include "stdbool.h"
#include "stdatomic.h"
#include "driver_pyro.h"

//Dont commit w/ IS_SITL defined
// #define IS_SITL


// Note all values are in m or m/s

#define IS_BOOSTER

#define ENGINE_GS (9.81*3)
// #define ENGINE_GS (9.81*0.1)
#define APPO_GS (9.81/2)

#define MAINS_ALT (1000/3.28)

#define PANIC_VEL (-240.0f/3.28)

// ALL UNITES ARE IN METRIC SI UNITS
typedef struct {
    float lift_off_acceleration_threshold;
    uint8_t lift_off_tick_count;
    float apogee_acceleration_threshold;
    uint8_t apogee_tick_count;
    float burnout_acceleration_threshold;
    uint8_t burnout_tick_count;
    uint8_t recovery_burnout_counter; // This condition will allow us to select the number of stages in multistage rockets
    bool arm_at_boot;
    pyro_channel_t Appo_Channel;
    pyro_channel_t Mains_Channel;
    pyro_channel_t Separation_Channel;
    pyro_channel_t Ignition_Channel;
    pyro_channel_t Aux_1_Channel;
    pyro_channel_t Aux_2_Channel;
    pyro_channel_t Aux_3_Channel;
    pyro_channel_t Aux_4_Channel;
    float panic_velocity_threshold;
    float main_deployment_altitude;
    uint8_t main_deployment_tick_count;
    float highest_ground_elevation;
    float landed_velocity_threshold;
    uint8_t landed_tick_count;
} flight_config_t;

extern _Atomic flight_config_t flight_config;

// if this is defined then the board will erase and move to the next flash bank
// on any power up with atleast one continuous pyro, and will fake a txlock,
// signal this can be used to allow ascent to fly with out a ground station or
// two way coms to a ground station

#endif