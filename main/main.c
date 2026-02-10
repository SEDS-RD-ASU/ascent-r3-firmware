/**
 * @file main.c
 * @brief Entry point for ASCENT R3 firmware
 */


//MARK: ESP-IDF
#include <stdio.h>
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

//MARK: R3 DEVICE INTERFACES
#include "ascent_r3_hardware_definition.h"
#include "driver_buzzer.h"
#include "beep.h"
#include "onboard_led.h"
#include "i2c_manager.h"
#include "spi_manager.h"
#include "uart_manager.h"
#include "sensor_manager.h"
#include "flash_interface.h"
#include "nvs_interface.h"
#include "flash_interface.h"
#include "driver_psu.h"
#include "serial_util.h"
#include "ble.h"

//FLIGHT STATE MANAGEMENT
#include "flight.h"

// #define DEBUG
#define SIMULATOR

//GLOBALS
_Atomic barometer_sample_t baro;
_Atomic barometer_velocity_t baro_vel;
_Atomic acc_sample_t low_g_acc;
_Atomic acc_sample_t high_g_acc;
_Atomic gyr_sample_t gyr;
_Atomic gps_sample_t gps;

//MARK: TESTING UTILITIES.
//REMOVE THESE BEFORE MERGING TO FLIGHT BRANCH.
void measure_performance()
{
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

    esp_err_t ret;

    const unsigned MEASUREMENTS = 1000;

    gptimer_get_raw_count(gptimer, &times);
    
    for (int retries = 0; retries < MEASUREMENTS; retries++) {
        // replace w/ function to measure!
    }

    gptimer_get_raw_count(gptimer, &timef);

    printf("%u iterations took %llu ticks (%llu ticks per measurement)\n",
        MEASUREMENTS, (timef - times), (timef - times)/MEASUREMENTS);

    megolavania();
}

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

//MARK: INITIALIZATION CODE
esp_err_t flight_initialize_devices(void)
{
    esp_err_t ret = ESP_OK;
    board_information_t board_info;

    bool is_simulator = false;
    #ifdef SIMULATOR
    is_simulator = true;
    #endif

    ret = buzzer_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE BUZZER"); return ret;}
    ascent_beep(); // beep boop
    
    ret = led_init(PIN_LED);
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE LED"); return ret;}
    led_blue(); // let there be light

    ret = psu_init_default();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE PSU"); return ret;}

    ret = nvs_interface_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE NVS"); return ret;}

    ret = i2c_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE I2C BUSSES"); return ret;}

    ret = spi_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE SPI BUSSES"); return ret;}

    ret = uart_flight_init();
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE UART BUSSES"); return ret;}

    ret = initialize_sensors(is_simulator);
    if(ret != ESP_OK) {ESP_LOGE("flight_initialize_devices", "FAILED TO INITIALIZE SENSORS"); return ret;}

    serial_util_init();

    nvs_retrieve_board_info(&board_info);
    ble_init(board_info.serial_number);
    
    ret = flash_flight_init();
    if(ret != ESP_OK) {ESP_LOGI("flight_initialize_devices", "FAILED TO INITIALIZE SPI FLASH"); return ret;}

    print_board_info();
    led_yellow();
    high_beep();high_beep();high_beep(); // success!

    printf("\n\n");
    ESP_LOGI("flight_initialize_devices", "All devices initialized successfully!");
    printf("\n\n");

    return ESP_OK;
}

//MARK: PRIMARY TASK
// Will be running at 50Hz on the primary core.
TaskHandle_t primary_task_handle;
int primary_loop_fq = 50;
TickType_t xFrequency_primary;
void primary_task(void *pvParameters)
{
    const TickType_t xFrequency_primary = pdMS_TO_TICKS(1000 / primary_loop_fq);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t cycle = 0;

    barometer_sample_t primary_baro;
    barometer_velocity_t primary_baro_vel;
    acc_sample_t primary_low_g_acc;
    acc_sample_t primary_high_g_acc;
    gyr_sample_t primary_gyr;
    gps_sample_t primary_gps;

    while (1)
    {
        uint8_t flight_state = get_flight_state();

        primary_baro = atomic_load(&baro);
        primary_baro_vel = atomic_load(&baro_vel);
        primary_low_g_acc = atomic_load(&low_g_acc);
        primary_high_g_acc = atomic_load(&high_g_acc);
        primary_gyr = atomic_load(&gyr);
        primary_gps = atomic_load(&gps);

        baro_update(primary_baro, &primary_baro_vel);

        #ifdef DEBUG // DO NOT MERGE THIS SECTION TO FLIGHT BRANCH.
        printf( 
            "baro[t=%" PRIi64 "] P=%.2f T=%.2f AGL=%.2f GND=%.2f | vel=%.2f avg=%.2f | "
            "highG[%.3f %.3f %.3f] lowG[%.3f %.3f %.3f] gyr[%.3f %.3f %.3f] | "
            "gps[t=%" PRIi64 " UTC=%" PRIu32 " lat=%" PRIu32 " lon=%" PRIu32 " altE=%" PRIu32 " altMSL=%" PRIu32 " fix=%u sats=%u]\n",
            primary_baro.timestamp,
            primary_baro.pressure,
            primary_baro.temperature,
            primary_baro.altitude_agl,
            primary_baro.ground_altitude,
            primary_baro_vel.velocity,
            primary_baro_vel.average_velocity,
            primary_high_g_acc.acc_x,
            primary_high_g_acc.acc_y,
            primary_high_g_acc.acc_z,
            primary_low_g_acc.acc_x,
            primary_low_g_acc.acc_y,
            primary_low_g_acc.acc_z,
            primary_gyr.gyr_x,
            primary_gyr.gyr_y,
            primary_gyr.gyr_z,
            primary_gps.timestamp,
            primary_gps.UTCtstamp,
            primary_gps.lat,
            primary_gps.lon,
            primary_gps.altitude_ellipsoid,
            primary_gps.altitude_msl,
            primary_gps.fixType,
            primary_gps.num_sats);
        #endif

        if (flight_state > FS_ON_PAD && flight_state != FS_LANDED) // if we are in the air, basically
        {
            printf("I am flying!!!\n");
        } else {
            // Do something while not flying
        }

        flash_packet primary_flash_packet = {
            .n = 0,
            .timestamp = esp_timer_get_time(),
            .bat_voltage = psu_read_battery_voltage(),
            .flight_state = flight_state,
            .pyro_cont = 0, // TODO: REPLACE WITH ACTUAL PYRO LOGIC. FOR DAQ WE DON'T CARE RN.
            
            .pressure = primary_baro.pressure,
            .temperature = primary_baro.temperature,
            .altitude_agl = primary_baro.altitude_agl,
            .ground_altitude = primary_baro.ground_altitude,

            .UTCtstamp = primary_gps.UTCtstamp,
            .lat = primary_gps.lat,
            .lon = primary_gps.lon,
            .altitude_ellipsoid = primary_gps.altitude_ellipsoid,
            .altitude_msl = primary_gps.altitude_msl,
            .fixType = primary_gps.fixType,
            .num_sats = primary_gps.num_sats,

            .acc_x = primary_low_g_acc.acc_x,
            .acc_y = primary_low_g_acc.acc_y,
            .acc_z = primary_low_g_acc.acc_z,

            .hacc_x = primary_high_g_acc.acc_x,
            .hacc_y = primary_high_g_acc.acc_y,
            .hacc_z = primary_high_g_acc.acc_z,

            .gyr_x = primary_gyr.gyr_x,
            .gyr_y = primary_gyr.gyr_y,
            .gyr_z = primary_gyr.gyr_z,
        };

        flight_update(primary_baro.altitude_agl, primary_baro_vel.velocity, primary_baro_vel.average_velocity, primary_high_g_acc.acc_y);

        flash_queue_packet(&primary_flash_packet);

        cycle = (cycle + 1) % primary_loop_fq;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_primary);
    }
}

//MARK: FAST SENSOR TASK
// Will be running as fast as possible on core 1
TaskHandle_t fast_sensor_task_handle;
void fast_sensor_task(void *pvParameters)
{
    barometer_sample_t temp_baro;
    barometer_velocity_t temp_baro_vel;
    acc_sample_t temp_low_g_acc;
    acc_sample_t temp_high_g_acc;
    gyr_sample_t temp_gyr;
    gps_sample_t temp_gps;

    while(1)
    {
        poll_sensors(&temp_baro, &temp_baro_vel, &temp_high_g_acc, &temp_low_g_acc, &temp_gyr, &temp_gps);

        atomic_store(&baro, temp_baro);
        atomic_store(&baro_vel, temp_baro_vel);
        atomic_store(&high_g_acc, temp_high_g_acc);
        atomic_store(&low_g_acc, temp_low_g_acc);
        atomic_store(&gyr, temp_gyr);
        // atomic_store(&gps, temp_gps); MOVED TO SLOW SENSOR TASK
    }
}

//MARK: SLOW SENSOR TASK
// Operates at 20Hz on core 1
TaskHandle_t slow_sensor_task_handle;
int slow_sensor_frequency = 20;
TickType_t xFrequency_slow_sensor;
void slow_sensor_task(void *pvParameters)
{
    const TickType_t xFrequency_slow_sensor = pdMS_TO_TICKS(1000 / slow_sensor_frequency);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t cycle = 0;

    gps_sample_t temp_gps;

    while(1)
    {
        poll_gps(&temp_gps);
        atomic_store(&gps, temp_gps);

        cycle = (cycle + 1) % slow_sensor_frequency;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_slow_sensor);
    }
}

//MARK: FLASH TASK
// Operates as fast as possible on core 1
TaskHandle_t flash_task_handle;
void flash_task(void *pvParameters)
{
    while(1)
    {
        uint8_t flight_state = get_flight_state();

        if (flight_state > FS_ON_PAD && flight_state != FS_LANDED) {
            flash_write_queue(12500);
        }

        taskYIELD();
    }
}


//MARK: SIMULATOR TASK
// Operates only in SITL (Software-in-the-loop) testing mode
#define BUF_SIZE (1024)
#define ECHO_TASK_STACK_SIZE (4096)
TaskHandle_t simulator_task_handle;
void simulator_task(void *pvParameters)
{
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE("usb_serial_jtag echo", "no memory for data");
        return;
    }

    while (1) {

        int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);

        // Write data back to the USB SERIAL JTAG
        if (len) {
            usb_serial_jtag_write_bytes((const char *) data, len, 20 / portTICK_PERIOD_MS);
            data[len] = '\0';
            ESP_LOG_BUFFER_HEXDUMP("Recv str: ", data, len, ESP_LOG_INFO);
        }
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

    flight_config_init();
    print_flight_config();

    flash_print_stats();
    try_to_dump_data();

    // measure_performance();

    // PRIMARY CORE TASKS
    xTaskCreatePinnedToCore(primary_task, "primary_task", 8192, NULL, 1, &primary_task_handle, 0);

    // SECONDARY CORE TASKS
    xTaskCreatePinnedToCore(fast_sensor_task, "fast_sensor_task", 8192, NULL, 2, &fast_sensor_task_handle, 1);
    xTaskCreatePinnedToCore(slow_sensor_task, "slow_sensor_task", 8192, NULL, 2, &slow_sensor_task_handle, 1);
    xTaskCreatePinnedToCore(flash_task, "flash_task", 4096, NULL, 1, &flash_task_handle, 1);

    #ifdef SIMULATOR // todo: replace w/ debug harness logic
    xTaskCreatePinnedToCore(simulator_task, "USB SERIAL JTAG_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, &simulator_task_handle, 1);
    #endif

}