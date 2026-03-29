#include "flash_interface.h"

#include "stdatomic.h"
#include "assert.h"
#include "math.h"
#include "stdlib.h"
#include "driver_w25qxx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ascent_r3_hardware_definition.h"
#include <rom/ets_sys.h>
#include "beep.h"

#include "serial_util.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "nvs_interface.h"

#include "beep.h"
#include "driver_buzzer.h"
#include "onboard_led.h"

static const char *TAG = "FLASH INTERFACE";

#define MAX_SECTORS     16384
#define SECTOR_SIZE     4096
#define FLASH_SIZE      (MAX_SECTORS * SECTOR_SIZE)  // 64 MB
#define STOP_THRESHOLD  0.95f

// NVS key for tracking how many bytes were written last flight (crash recovery)
#define NVS_USED_BYTES_KEY "used_bytes"

static uint32_t n = 0;
static uint32_t addr = 0;

static nvs_handle_t my_handle;

#define RING_BUFFER_SIZE 50
QueueHandle_t flash_packet_queue;

void flash_erase_jingle(void);

uint32_t flash_get_addr() {
    return addr;
}

esp_err_t flash_flight_init(void)
{
    esp_err_t ret;

    ret = w25qxx_init();

    my_handle = nvs_interface_get_handle();

    // Seed "used_bytes" if this is the first boot
    int32_t used_bytes;
    if (nvs_get_i32(my_handle, NVS_USED_BYTES_KEY, &used_bytes) != ESP_OK) {
        printf("'%s' not found in NVS, initializing to 0\n", NVS_USED_BYTES_KEY);
        nvs_set_i32(my_handle, NVS_USED_BYTES_KEY, 0);
    }

    flash_packet_queue = xQueueCreate(RING_BUFFER_SIZE, sizeof(flash_packet));
    assert(flash_packet_queue != NULL);

    if(ret == ESP_OK){
        ESP_LOGI(TAG, "Initialized SPI flash!");
    }

    return ret;
}

bool flash_prepare_for_flight(void) {
    printf("PREPARING FLASH FOR FLIGHT\n");

    // Read how many bytes were written last flight and erase only those sectors
    int32_t used_bytes = 0;
    nvs_get_i32(my_handle, NVS_USED_BYTES_KEY, &used_bytes);

    uint32_t used_sectors = ((uint32_t)used_bytes + SECTOR_SIZE - 1) / SECTOR_SIZE;
    printf("Erasing %lu used sectors (%ld bytes from last flight)\n", used_sectors, used_bytes);

    for (uint32_t i = 0; i < used_sectors; i++) {
        w25qxx_sector_erase(i * SECTOR_SIZE);
        printf("Erase progress: %f\n", (float)(i + 1) / (float)used_sectors);
    }

    // Reset write pointer
    addr = 0;

    // Write 0 to NVS now (safe to stall during pre-flight setup)
    nvs_set_i32(my_handle, NVS_USED_BYTES_KEY, 0);

    printf("Ready to fly\n");
    flash_erase_jingle();
    return true;
}

void flash_dump_to_serial(void) {
    flash_packet fp;

    printf("DUMPING FLASH DATA\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    uint32_t read_addr = 0;
    printf("n, timestamp, bat_voltage, flight_state, pyro_cont, pressure, temperature, altitude_agl, ground_altitude, baro_vel, avg_baro_vel, UTCtstamp, lat, lon, altitude_ellipsoid, altitude_msl, fixType, num_sats, acc_x, acc_y, acc_z, hacc_x, hacc_y, hacc_z, gyr_x, gyr_y, gyr_z\n");

    while (read_addr < FLASH_SIZE) {
        w25qxx_read(read_addr, (uint8_t*)&fp, sizeof(flash_packet));
        read_addr += sizeof(flash_packet);

        // Stop at first all-0xFF page (end of written data)
        bool all = true;
        char* buf = (char*) &fp;
        for (int i = 0; i < (int)sizeof(flash_packet); i++) {
            if (buf[i] != 0xFF) all = false;
        }
        if (all) break;

        printf("%llu,", fp.n);
        printf("%" PRId64 ",", fp.timestamp);
        printf("%f,", fp.bat_voltage);
        printf("%d,", fp.flight_state);
        printf("%d,", fp.pyro_cont);

        printf("%f,", fp.pressure);
        printf("%f,", fp.temperature);
        printf("%f,", fp.altitude_agl);
        printf("%f,", fp.ground_altitude);
        printf("%f,", fp.baro_vel);
        printf("%f,", fp.avg_baro_vel);

        printf("%lu,", fp.UTCtstamp);
        printf("%lu,", fp.lat);
        printf("%lu,", fp.lon);
        printf("%lu,", fp.altitude_ellipsoid);
        printf("%lu,", fp.altitude_msl);
        printf("%d,", fp.fixType);
        printf("%d,", fp.num_sats);

        printf("%f,", fp.acc_x);
        printf("%f,", fp.acc_y);
        printf("%f,", fp.acc_z);

        printf("%f,", fp.hacc_x);
        printf("%f,", fp.hacc_y);
        printf("%f,", fp.hacc_z);

        printf("%f,", fp.gyr_x);
        printf("%f,", fp.gyr_y);
        printf("%f\n", fp.gyr_z);
    }

    flash_erase_jingle();
    printf("FINISHED DUMPING DATA\n");
}

void flash_write_packet(flash_packet *packet) {
    static const uint32_t stop_addr = (uint32_t)(FLASH_SIZE * STOP_THRESHOLD);
    static bool led_triggered = false;

    // 95% full — stop writing, save used bytes to NVS exactly once
    if (addr >= stop_addr) {
        if (!led_triggered) {
            nvs_set_i32(my_handle, NVS_USED_BYTES_KEY, (int32_t)addr);
            led_red();
            led_triggered = true;
        }
        printf("FLASH 95%% FULL, PACKET DROPPED\n");
        return;
    }

    // Hard limit — shouldn't be reachable given 95% check above
    if (addr >= FLASH_SIZE) {
        if (!led_triggered) {
            nvs_set_i32(my_handle, NVS_USED_BYTES_KEY, (int32_t)addr);
            led_red();
            led_triggered = true;
        }
        printf("FLASH FULL, PACKET LOST\n");
        return;
    }

    w25qxx_write(addr, (uint8_t*) packet, sizeof(flash_packet));
    addr += sizeof(flash_packet);
}

void flash_queue_packet(flash_packet *packet) {
    packet->n = n++;
    if (xQueueSendToBack(flash_packet_queue, packet, 0) != pdTRUE) {
        flash_packet oldItem;
        xQueueReceive(flash_packet_queue, &oldItem, 0);
        xQueueSendToBack(flash_packet_queue, packet, 1);
    }
}

void flash_write_queue(int64_t max_time) {
    int64_t start = esp_timer_get_time();
    flash_packet packet;

    static UBaseType_t max_count = 0;
    UBaseType_t count = uxQueueMessagesWaiting(flash_packet_queue);
    max_count = count > max_count ? count : max_count;

    while ((esp_timer_get_time() - start) < max_time) {
        if (xQueueReceive(flash_packet_queue, &packet, 0) == pdTRUE) {
            flash_write_packet(&packet);
        } else {
            break;
        }
    }
}

void flash_print_stats() {
    int32_t used_bytes = 0;
    nvs_get_i32(my_handle, NVS_USED_BYTES_KEY, &used_bytes);
    printf("Flash used last flight: %ld / %d bytes (%.1f%%)\n",
           used_bytes, FLASH_SIZE,
           (float)used_bytes / (float)FLASH_SIZE * 100.0f);
    printf("\n");
}

void flash_blank_slate() {
    printf("CHIP ERASE — please wait, this can take several minutes. DO NOT POWER OFF.\n");
    w25qxx_chip_erase();
    nvs_set_i32(my_handle, NVS_USED_BYTES_KEY, 0);
    printf("Done\n");
    flash_erase_jingle();
}

void try_to_dump_data() {
    printf("You have 5 seconds to enter \"DUMP\", \"ERASE\", or \"NUCLEAR\"...\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    char buf[512];
    int i = 0;
    while (serial_util_readline_nonblocking(buf, 512, &i, 1000/portTICK_PERIOD_MS)) {
        if (strcmp("DUMP", buf) == 0) {
            while (true) {
                flash_dump_to_serial();
                for (int j = 0; j < 3; j++) {
                    flash_erase_jingle();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
        } else if (strcmp("ERASE", buf) == 0) {
            flash_blank_slate();
            printf("System will now restart...\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        } else if (strcmp("NUCLEAR", buf) == 0) {
            int time_spent = 0;
            printf("WARNING: This will erase all NVS data including flight configs! Type \"YES\" to confirm (Timeout: 10s):\n");
            while (time_spent < 10) {
                if (serial_util_readline_nonblocking(buf, 512, &i, 1000/portTICK_PERIOD_MS)) {
                    if (strcmp("YES", buf) == 0) {
                        printf("Erasing NVS...\n");
                        nvs_flash_erase();
                        printf("Done. System will now restart...\n");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        esp_restart();
                    } else {
                        printf("Aborting NUCLEAR operation.\n");
                    }
                    break;
                }
                time_spent++;
            }
            if (time_spent >= 10) {
                printf("NUCLEAR operation timed out.\n");
            }
        }
    }
    usb_serial_jtag_driver_uninstall();
}

void flash_erase_jingle(void) {
    note(NOTE_E, 8, 120);
    note(NOTE_G, 8, 120);
    note(NOTE_C, 7, 200);
    note(NOTE_D, 7, 120);
    note(NOTE_B, 6, 250);
    note(NOTE_E, 7, 400);
}
