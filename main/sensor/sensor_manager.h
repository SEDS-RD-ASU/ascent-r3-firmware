#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "stdatomic.h"

#include "i2c_manager.h"
#include "spi_manager.h"
#include "ascent_r3_hardware_definition.h"

#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"
#include "interface_LSM6DSV320X.h"
#include "esp_timer.h"

typedef struct {
    int64_t timestamp;
    double pressure;
    double temperature;
    double altitude_agl;
    double ground_altitude; 
} barometer_sample_t;

typedef struct {
    float altitude_agl;
    float velocity;
    float average_velocity;
} barometer_velocity_t;

typedef struct {
    int64_t timestamp;
    uint32_t UTCtstamp;
    uint32_t lat;
    uint32_t lon;
    uint32_t altitude_ellipsoid;
    uint32_t altitude_msl;
    uint8_t fixType;
    uint8_t num_sats;
} gps_sample_t;

typedef struct {
    int64_t timestamp;
    float acc_x;
    float acc_y;
    float acc_z;
} acc_sample_t;

typedef struct {
    int64_t timestamp;
    float grav_x;
    float grav_y;
    float grav_z;
} grav_sample_t;

typedef struct {
    int64_t timestamp;
    float gyr_x;
    float gyr_y;
    float gyr_z;
} gyr_sample_t;

typedef struct {
    barometer_sample_t baro;
    TaskHandle_t read_baro_task;
} bmp_context_t;

esp_err_t initialize_sensors(bool simulator);

void poll_sensors(barometer_sample_t *pBaro, acc_sample_t *high_g, acc_sample_t *low_g, grav_sample_t *grav,gyr_sample_t *gyr, gps_sample_t *gps);

void barometer_int_callback(void *args);

void baro_update(barometer_sample_t baro, barometer_velocity_t *baro_vel);

esp_err_t poll_gps(gps_sample_t *gps);

void feed_fake_flight_data(barometer_sample_t baro, barometer_velocity_t baro_vel, acc_sample_t high_g, acc_sample_t low_g, grav_sample_t grav, gyr_sample_t gyr, gps_sample_t gps);

#endif