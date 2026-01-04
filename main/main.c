/**
 * @file main.c
 * @author Abdul Zia
 * @brief Entry point for ASCENT R3 firmware
 */


//MARK: ESP-IDF
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

//R3 DEVICE INTERFACES
#include "driver_buzzer.h"
#include "beep.h"
#include "i2c_manager.h"
#include "spi_manager.h"
#include "sensor_manager.h"

//MARK: INITIALIZATION CODE
void validate_esp(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}

esp_err_t flight_initialize_devices(void){
    esp_err_t ret = ESP_OK;

    ret = buzzer_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE BUZZER"); return ret;}
    // ascent_beep();

    ret = i2c_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE I2C BUSSES"); return ret;}

    ret = spi_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE SPI BUSSES"); return ret;}

    return ESP_OK;
}

//MARK: ENTRY POINT
void app_main(void)
{   
    int fail = 0;

    validate_esp();

    esp_err_t ret;
    ret = flight_initialize_devices();
    if (ret != ESP_OK) {ESP_LOGE("app_main", "Failed to initialize devices!"); fail++;}

}
