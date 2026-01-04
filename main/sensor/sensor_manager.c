/**
 * @file sensor_manager.c
 * @author Abdul Zia
 * @brief Code responssible for processing sensor data during flight.
 */

#include "sensor_manager.h"

static const char *TAG = "SENSOR MANAGER";

esp_err_t initialize_sensors(void)
{
    esp_err_t ret;
    i2c_port_t BMP390_I2C_PORT = R3_I2C1_PORT;
    i2c_port_t SAM_M10Q_I2C_PORT = R3_I2C0_PORT;

    #ifdef R2
    ESP_LOGW(TAG, "YOU ARE USING R2 PINS FOR SENSORS. REMOVE #define R2 IF THIS IS NOT A R2 BOARD.");
    BMP390_I2C_PORT = R2_I2C0_PORT;
    SAM_M10Q_I2C_PORT = R2_I2C0_PORT;
    #endif
    
    ret = bmp390_flight_init(BMP390_I2C_PORT);
    if(ret != ESP_OK) {ESP_LOGE(TAG, "FAILED TO INITIALIZE BMP390"); return ret;}
    
    ret = GPS_init(SAM_M10Q_I2C_PORT);
    if(ret != ESP_OK) {ESP_LOGE(TAG, "FAILED TO INITIALIZE SAM-M10Q"); return ret;}

    ESP_LOGI(TAG, "SUCCESSFULLY INITIALIZED ALL SENSORS");
    return ESP_OK;
}