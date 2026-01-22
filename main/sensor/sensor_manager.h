#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "i2c_manager.h"
#include "spi_manager.h"
#include "ascent_r3_hardware_definition.h"

#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"

typedef struct {
    int64_t timestamp;
    double pressure;
    double temperature;
    double altitude_agl;
    double ground_altitude; 
} barometer_sample_t;

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
    float gyr_x;
    float gyr_y;
    float gyr_z;
} gyr_sample_t;

esp_err_t initialize_sensors(void);

esp_err_t poll_barometer(barometer_sample_t *baro);

esp_err_t poll_gps(gps_sample_t *gps);

void baro_update(barometer_sample_t baro, float *agl, float *vel, float *avg_vel);

#endif