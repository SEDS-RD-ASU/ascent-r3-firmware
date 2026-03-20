/**
 * @file main.c
 * @brief Entry point for ASCENT R3 firmware
 */


//ESP-IDF
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "stdatomic.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_task_wdt.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/usb_serial_jtag.h"

//R3 DEVICE INTERFACES
#include "ascent_r3_hardware_definition.h"
#include "driver_buzzer.h"
#include "beep.h"
#include "onboard_led.h"
#include "i2c_manager.h"
#include "spi_manager.h"
#include "uart_manager.h"
#include "sensor_manager.h"
#include "mode_manager.h"
#include "flash_interface.h"
#include "nvs_interface.h"
#include "driver_psu.h"
#include "serial_util.h"
#include "ble.h"
#include "command.h"

//FLIGHT STATE MANAGEMENT
#include "flight.h"

//TELEMETRY
#include "goober.h"

// TASKS
#include "tasks/flight_tasks.h"
#include "tasks/util_tasks.h"

// #define DEBUG
// #define SIMULATOR

//GLOBALS
_Atomic barometer_sample_t baro;
_Atomic barometer_velocity_t baro_vel;
_Atomic acc_sample_t low_g_acc;
_Atomic acc_sample_t high_g_acc;
_Atomic gyr_sample_t gyr;
_Atomic gps_sample_t gps;

//MARK: INITIALIZATION CODE
esp_err_t flight_initialize_devices(void)
{
    esp_err_t ret = ESP_OK;
    board_information_t board_info;

    initialize_mode_button();

    ret = buzzer_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE BUZZER"); return ret;}
    ascent_beep(); // beep boop
    
    ret = led_init(PIN_LED);
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE LED"); return ret;}
    led_blue(); // let there be light

    // Check if BOOT is pressed to enter mode selection (2s window)
    if (mode_detect_boot_press_ms(2000)) {
        ESP_LOGI("flight_initialize_devices", "RUN MODE SELECTION STARTED");
        mode_run_selection_window_ms(5000);
    }

    ret = pyro_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE PYRO"); return ret;}

    ret = psu_init_default_with_adc(pyro_get_adc1_handle());
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE PSU"); return ret;}

    ret = nvs_interface_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE NVS"); return ret;}

    ret = i2c_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE I2C BUSSES"); return ret;}

    ret = spi_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE SPI BUSSES"); return ret;}

    ret = uart_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE UART BUSSES"); return ret;}

    

    ret = initialize_sensors(is_simulator_mode());
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE SENSORS"); return ret;}

    serial_util_init();
    
    ret = flash_flight_init();
    if(ret != ESP_OK) {ESP_LOGI("flight_initialize_devices", "FAILED TO INITIALIZE SPI FLASH"); return ret;}

    ble_init(board_serial_number());
    
    print_board_info();

    printf("\n\n");
    ESP_LOGI("flight_initialize_devices", "All devices initialized successfully!");
    printf("\n\n");

    return ESP_OK;
}

//MARK: Pyro Beep
void beep_pyro_cont(void) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            if (pyro_continuity(j+1)) high_beep();
            else low_beep();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}


//MARK: ENTRY POINT
void app_main(void)
{
    validate_esp();
    
    esp_err_t ret;

    ret = esp_task_wdt_deinit();
    if (ret) {
        printf("FAILED TO DEINIT TASK WATCH DOG\n");
        error_beep();
        led_red();
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    ret = flight_initialize_devices();
    if (ret) {
        ESP_LOGE("app_main", "DEVICE INITIALIZATION HAS FAILED!");
        error_beep();
        led_red();
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    beep_pyro_cont();

    flight_config_init();
    print_flight_config();

    flash_print_stats();

    if (is_flash_dump_mode()) {
        try_to_dump_data();
    }

    // measure_performance();
    
    initialize_telemetry_queue();

    if (is_simulator_mode()) { // todo: replace w/ debug harness logic
        // Configure USB SERIAL JTAG
        usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
            .rx_buffer_size = 1024,
            .tx_buffer_size = 1024,
        };
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
        ESP_LOGI("app_main", "USB_SERIAL_JTAG init done");

        xTaskCreatePinnedToCore(simulator_task, "simulator_task", SIMULATOR_TASK_STACK_SIZE, NULL, 10, &simulator_task_handle, 1);
    }

    // SECONDARY CORE TASKS
    xTaskCreatePinnedToCore(fast_sensor_task, "fast_sensor_task", 8192, NULL, 2, &fast_sensor_task_handle, 1);
    xTaskCreatePinnedToCore(slow_sensor_task, "slow_sensor_task", 8192, NULL, 2, &slow_sensor_task_handle, 1);
    xTaskCreatePinnedToCore(flash_task, "flash_task", 4096, NULL, 1, &flash_task_handle, 1);

    // PRIMARY CORE TASKS
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", 8192, NULL, 1, &telemetry_task_handle, 0);
    xTaskCreatePinnedToCore(primary_task, "primary_task", 8192, NULL, 1, &primary_task_handle, 0);

    led_yellow();
    high_beep();high_beep();high_beep(); // success!

}