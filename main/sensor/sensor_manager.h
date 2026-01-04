#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"

#include "i2c_manager.h"
#include "spi_manager.h"
#include "ascent_r3_hardware_definition.h"

#include "interface_bmp390l.h"

/**
 * @file sensor_manager.h
 * @brief Initialization routines for ASCENT R3 sensors.
 */

#define R2 // REMOVE IF COMPILING FOR R2

/**
 * @brief Initialize all flight sensors and validate connectivity.
 *
 * @return esp_err_t ESP_OK on success or an error code describing the failure.
 */
esp_err_t initialize_sensors(void);

#endif
