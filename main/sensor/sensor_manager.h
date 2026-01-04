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
#include "interface_sam_m10q.h"

#define R2 // REMOVE IF COMPILING FOR R2

esp_err_t initialize_sensors(void);

#endif