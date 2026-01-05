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
#include "driver/gptimer.h"

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

esp_err_t   flight_initialize_devices(void){
    esp_err_t ret = ESP_OK;

    ret = buzzer_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE BUZZER"); return ret;}
    ascent_beep();

    ret = i2c_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE I2C BUSSES"); return ret;}

    ret = spi_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE SPI BUSSES"); return ret;}

    ret = initialize_sensors();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE SENSORS"); return ret;}

    return ESP_OK;
}

void measure_performance() {
    uint64_t times = 0;
    uint64_t timef = 0;
    double pres = 0;

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_XTAL,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 20 * 1000 * 1000
    };
    // Create a timer instance
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    // Enable the timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // Start the timer
    ESP_ERROR_CHECK(gptimer_start(gptimer));


    barometer_sample_t baro;
    float agl, vel, avg_vel;

    gps_sample_t gps;

    esp_err_t ret;

    const unsigned MEASUREMENTS = 1000;

    gptimer_get_raw_count(gptimer, &times);
    
    for (int retries = 0; retries < MEASUREMENTS; retries++) {
        poll_gps(&gps);
    }

    gptimer_get_raw_count(gptimer, &timef);

    printf("%u iterations took %llu ticks (%llu ticks per measurement)\n",
        MEASUREMENTS, (timef - times), (timef - times)/MEASUREMENTS);

    megolavania();
}

//MARK: PRIMARY TASK
TaskHandle_t primary_task_handle;
int primary_loop_fq = 100;
TickType_t xFrequency_primary;
void primary_task(void *pvParameters)
{
    const TickType_t xFrequency_primary = pdMS_TO_TICKS(1000 / primary_loop_fq);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t cycle = 0;

    barometer_sample_t baro;
    gps_sample_t gps;

    while (1)
    {
        float agl, vel, avg_vel;

        poll_gps(&gps);

        poll_barometer(&baro);
        baro_update(baro, &agl, &vel, &avg_vel);

        cycle = (cycle + 1) % primary_loop_fq;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_primary);
    }
}

//MARK: ENTRY POINT
void app_main(void)
{

    validate_esp();

    esp_err_t ret;
    ret = flight_initialize_devices();
    if (ret != ESP_OK) {
        ESP_LOGE("app_main", "DEVICE INITIALIZATION HAS FAILED!");
        error_beep();
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    // measure_performance();

    xTaskCreatePinnedToCore(primary_task, "primary_task", 8192, NULL, 1, &primary_task_handle, 1);

}
