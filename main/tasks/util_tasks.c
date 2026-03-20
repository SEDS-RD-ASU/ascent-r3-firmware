#include "util_tasks.h"

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
#include "sensor_manager.h"
#include "flash_interface.h"
#include "driver_buzzer.h"
#include "beep.h"

//MARK: PERFORMANCE
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


// MARK: VALIDATION
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


//MARK: SIMULATION
// Operates only in SITL (Software-in-the-loop) testing mode
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