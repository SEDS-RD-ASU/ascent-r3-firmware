#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "uart_manager.h"
#include "sensor_manager.h"
#include "flash_interface.h"
#include "nvs_interface.h"
#include "command.h"
#include "flight.h"
#include "driver_pyro.h"
#include "driver_psu.h"
#include <goober.h>

extern _Atomic barometer_sample_t baro;
extern _Atomic barometer_velocity_t baro_vel;
extern _Atomic acc_sample_t low_g_acc;
extern _Atomic acc_sample_t high_g_acc;
extern _Atomic gyr_sample_t gyr;
extern _Atomic gps_sample_t gps;

//MARK: PRIMARY TASK
// Will be running at 50Hz on the primary core.
TaskHandle_t primary_task_handle;
int primary_loop_fq = 100;
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
int flash_task_frequency = 120;
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

        if (is_tx_lock()) {
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