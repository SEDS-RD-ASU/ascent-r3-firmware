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
#include "flash_interface.h"
#include "nvs_interface.h"
#include "flash_interface.h"
#include "driver_psu.h"
#include "serial_util.h"
#include "ble.h"
#include "command.h"

//FLIGHT STATE MANAGEMENT
#include "flight.h"

//TELEMETRY
#include "goober.h"

// #define DEBUG
// #define SIMULATOR

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

    ret = initialize_sensors(is_simulator);
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
    double batt_voltage = 0;

    while (1)
    {
        uint8_t flight_state = get_flight_state();
        uint8_t pyro_arm = calc_pyro_arm();
        pyro_update_state();

        primary_baro = atomic_load(&baro);
        primary_baro_vel = atomic_load(&baro_vel);
        primary_low_g_acc = atomic_load(&low_g_acc);
        primary_high_g_acc = atomic_load(&high_g_acc);
        primary_gyr = atomic_load(&gyr);
        primary_gps = atomic_load(&gps);
        batt_voltage = psu_read_battery_voltage();

        // printf("%f\n", primary_baro.altitude_agl);

        baro_update(primary_baro, &primary_baro_vel);
        atomic_store(&baro_vel, primary_baro_vel);

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

        flash_packet primary_flash_packet = {
            .n = 0,
            .timestamp = esp_timer_get_time(),
            .bat_voltage = batt_voltage,
            .flight_state = flight_state,
            .pyro_cont = 0, // TODO: REPLACE WITH ACTUAL PYRO LOGIC. FOR DAQ WE DON'T CARE RN.
            
            .pressure = primary_baro.pressure,
            .temperature = primary_baro.temperature,
            .altitude_agl = primary_baro.altitude_agl,
            .ground_altitude = primary_baro.ground_altitude,
            .baro_vel = primary_baro_vel.velocity,
            .avg_baro_vel = primary_baro_vel.average_velocity,
            
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

        flash_queue_packet(&primary_flash_packet);

        ascent_telemetry_t latest_telemetry_payload = {
            .timestamp = esp_timer_get_time() / 1000, // convert us to ms
            .latitude = primary_gps.lat,
            .longitude = primary_gps.lon,
            .altitude_agl = primary_baro.altitude_agl,
            .vertical_velocity = primary_baro_vel.velocity,
            .y_acc = primary_low_g_acc.acc_y,
            .gyr_y = primary_gyr.gyr_y,
            .pyro_state = pyro_arm,
            .sats = primary_gps.num_sats,
            .flight_state = flight_state,
            .battery_voltage = (uint16_t)(batt_voltage * 2500),
        };

        queueLatestTelemetry(&latest_telemetry_payload);

        if (flight_state > FS_ON_PAD && flight_state != FS_LANDED) // if we are in the air, basically
        {
            // printf("I am flying!!!\n");
        } else {
            // Do something while not flying
        }

        flight_update(primary_baro.altitude_agl, primary_baro_vel.velocity, primary_baro_vel.average_velocity, primary_low_g_acc.acc_y);
        
        cycle = (cycle + 1) % primary_loop_fq;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_primary);
    }
}


//MARK: FAST SENSOR TASK
// Operates at 100hz on core 1
TaskHandle_t fast_sensor_task_handle;
int fast_sensor_task_frequency = 600;
TickType_t xFrequency_fast_sensor_task;
void fast_sensor_task(void *pvParameters)
{
    barometer_sample_t temp_baro = {0};
    acc_sample_t temp_low_g_acc = {0};
    acc_sample_t temp_high_g_acc = {0};
    gyr_sample_t temp_gyr = {0};
    gps_sample_t temp_gps = {0};

    const TickType_t xFrequency_fast_sensor_task = pdMS_TO_TICKS(1000 / fast_sensor_task_frequency);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t cycle = 0;

    while(1)
    {
        poll_sensors(&temp_baro, &temp_high_g_acc, &temp_low_g_acc, &temp_gyr, &temp_gps);

        atomic_store(&baro, temp_baro);
        atomic_store(&high_g_acc, temp_high_g_acc);
        atomic_store(&low_g_acc, temp_low_g_acc);
        atomic_store(&gyr, temp_gyr);
        // atomic_store(&gps, temp_gps); MOVED TO SLOW SENSOR TASK

        cycle = (cycle + 1) % fast_sensor_task_frequency;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_fast_sensor_task);
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
// Operates at 60hz on core 1
TaskHandle_t flash_task_handle;
int flash_task_frequency = 60;
TickType_t xFrequency_flash_task;
void flash_task(void *pvParameters)
{
    const TickType_t xFrequency_flash_task = pdMS_TO_TICKS(1000 / flash_task_frequency);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t cycle = 0;

    uint8_t flight_state;

    while(1)
    {
        flight_state = get_flight_state();

        if (flight_state >FS_ON_PAD && flight_state != FS_LANDED) {
            flash_write_queue(12500);
        }

        cycle = (cycle + 1) % flash_task_frequency;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_flash_task);
    }
}


//MARK: TELEMETRY TASK
// Operates at 60hz on the primary core
TaskHandle_t telemetry_task_handle;
int telemetry_loop_fq = 60;
TickType_t xFrequency_telemetry;
void telemetry_task(void *pvParameters)
{
    const TickType_t xFrequency_telemetry = pdMS_TO_TICKS(1000 / telemetry_loop_fq);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t cycle = 0;

    ascent_telemetry_t latest_telemetry;

    uint8_t telemetry_buffer[256]; // GOOBER packets cannot be more than 256 bytes
    uint8_t latest_telemetry_buffer_size = 0;
    

    goober_header_t latest_header = {
        .dev_id = board_serial_number(),
        .dev_mode = 0, // will be overwritten
        .seq_id = 0, // will be overwritten
        .msg_cls = 0, // will be overwritten
        .payload_length = 0, // will be overwritten
    };

    while(1) {
        peekLatestTelemetry(&latest_telemetry);

        latest_header.seq_id = next_sequence_id();
        latest_header.msg_cls = TELEMETRY;
        latest_header.payload_length = sizeof(ascent_telemetry_t);

        if(is_tx_lock())
        {
            latest_header.dev_mode = goober_device_mode(GOOBER_MODE_SIMPLEX, true, true, false, false);
        } else {
            latest_header.dev_mode = goober_device_mode(GOOBER_MODE_HALF_DUPLEX, false, true, false, false);
        }

        goober_serialize(latest_header, (uint8_t *)&latest_telemetry, sizeof(latest_telemetry), telemetry_buffer, sizeof(telemetry_buffer), &latest_telemetry_buffer_size);
        
        uart1_transmit((uint8_t *)&telemetry_buffer, latest_telemetry_buffer_size);
        uart1_transmit((uint8_t *)"\n\n\n\n", 4);

        cycle = (cycle + 1) % telemetry_loop_fq;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_telemetry);
    }
}


//MARK: SIMULATOR TASK
// Operates only in SITL (Software-in-the-loop) testing mode
#define SIMULATOR_TASK_STACK_SIZE (4096)
#define SIMULATOR_BUF_SIZE (1024)
#define DELIMITER "\n\n\n\n"
#define DELIMITER_LEN 4
TaskHandle_t simulator_task_handle;
void simulator_task(void *pvParameters)
{
    uint8_t rx_byte;
    uint8_t accum_buf[SIMULATOR_BUF_SIZE];
    int accum_len = 0;
    
    barometer_sample_t temp_baro = {0};
    barometer_velocity_t temp_baro_vel = {0};
    acc_sample_t temp_low_g_acc = {0};
    acc_sample_t temp_high_g_acc = {0};
    gyr_sample_t temp_gyr = {0};
    gps_sample_t temp_gps = {
        .lat = 99,
        .lon = 99
    };

    while (1) {
        int len = usb_serial_jtag_read_bytes(&rx_byte, 1, 20 / portTICK_PERIOD_MS);
        if (len <= 0) continue;

        if (accum_len < SIMULATOR_BUF_SIZE) {
            accum_buf[accum_len++] = rx_byte;
        } else {
            // Buffer overflow — reset
            const char *overflow_msg = "ERR: RX buffer overflow, resetting\n";
            usb_serial_jtag_write_bytes(overflow_msg, strlen(overflow_msg), 20);
            accum_len = 0;
            continue;
        }

        // Check for delimiter at the end of the buffer
        if (accum_len >= DELIMITER_LEN &&
            memcmp(&accum_buf[accum_len - DELIMITER_LEN], DELIMITER, DELIMITER_LEN) == 0)
        {
            int payload_len = accum_len - DELIMITER_LEN;

            if (payload_len == sizeof(flash_packet)) {
                flash_packet *packet = (flash_packet *)accum_buf;
                // Parse all fields from the flash_packet into the temp_* variables
                temp_baro.pressure = packet->pressure;
                temp_baro.temperature = packet->temperature;
                temp_baro.altitude_agl = packet->altitude_agl;
                temp_baro.ground_altitude = packet->ground_altitude;

                temp_low_g_acc.acc_x = packet->acc_x;
                temp_low_g_acc.acc_y = packet->acc_y;
                temp_low_g_acc.acc_z = packet->acc_z;

                temp_high_g_acc.acc_x = packet->hacc_x;
                temp_high_g_acc.acc_y = packet->hacc_y;
                temp_high_g_acc.acc_z = packet->hacc_z;

                temp_gyr.gyr_x = packet->gyr_x;
                temp_gyr.gyr_y = packet->gyr_y;
                temp_gyr.gyr_z = packet->gyr_z;

                temp_gps.UTCtstamp = packet->UTCtstamp;
                temp_gps.lat = packet->lat;
                temp_gps.lon = packet->lon;
                temp_gps.altitude_ellipsoid = packet->altitude_ellipsoid;
                temp_gps.altitude_msl = packet->altitude_msl;
                temp_gps.fixType = packet->fixType;
                temp_gps.num_sats = packet->num_sats;

                feed_fake_flight_data(temp_baro, temp_baro_vel, temp_high_g_acc, temp_low_g_acc, temp_gyr, temp_gps);
            } else {
                char warn_buf[64];
                int warn_len = snprintf(warn_buf, sizeof(warn_buf),
                    "WARN: Bad packet size: got %d, expected %d\nBytes: ",
                    payload_len, (int)sizeof(flash_packet));
                usb_serial_jtag_write_bytes(warn_buf, warn_len, 20);
                for (int i = 0; i < payload_len; i++) {
                    char hex[4];
                    int hex_len = snprintf(hex, sizeof(hex), "%02X ", accum_buf[i]);
                    usb_serial_jtag_write_bytes(hex, hex_len, 20);
                }
                usb_serial_jtag_write_bytes("\n", 1, 20);
            }

            accum_len = 0;
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

    vTaskDelay(pdMS_TO_TICKS(500));

    beep_pyro_cont();

    flight_config_init();
    print_flight_config();

    flash_print_stats();
    try_to_dump_data();

    // measure_performance();
    
    initialize_telemetry_queue();

    #ifdef SIMULATOR // todo: replace w/ debug harness logic
        // Configure USB SERIAL JTAG
        usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
            .rx_buffer_size = 1024,
            .tx_buffer_size = 1024,
        };
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
        ESP_LOGI("app_main", "USB_SERIAL_JTAG init done");

        xTaskCreatePinnedToCore(simulator_task, "simulator_task", SIMULATOR_TASK_STACK_SIZE, NULL, 10, &simulator_task_handle, 1);
    #endif

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