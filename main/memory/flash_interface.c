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

static const char *TAG = "FLASH INTERFACE";

#define MAX_SECTORS 16384
#define SECTOR_SIZE 4096

#define BANKS 4
#define BANK_SIZE (MAX_SECTORS * SECTOR_SIZE / BANKS)
#define SECTORS_IN_BANK (MAX_SECTORS / BANKS)

static uint32_t n = 0;
static uint32_t addr = 0;
static int32_t current_bank = -1;

static nvs_handle_t my_handle;

static const char* bank_keys[BANKS] = {
    "bank_0",
    "bank_1",
    "bank_2",
    "bank_3",
};

#define RING_BUFFER_SIZE 50
QueueHandle_t flash_packet_queue;

void flash_erase_jingle(void);
static esp_err_t flash_erase_bank(int bank, int64_t max_time, int32_t *resume);

uint32_t flash_get_addr() {
    return addr;
}

esp_err_t flash_flight_init(void)
{
    esp_err_t ret;

    ret = w25qxx_init();

    my_handle = nvs_interface_get_handle();

    if (nvs_find_key(my_handle, "bank", NULL) != ESP_OK) {
        printf("Can't find 'bank' key nvs flash, rebuilding\n");
        nvs_set_i32(my_handle, "bank", 0);
        for (int i = 0; i < BANKS; i++) {
            nvs_set_i32(my_handle, bank_keys[i], BANK_SIZE);
        }
    }

    if (nvs_get_i32(my_handle, "bank", &current_bank) != ESP_OK) {
        printf("Can't find 'bank' key nvs flash part 2\n");
        // now we don't know what bank to use please fail
        for (int i = 0; i < 5; i++) {
            error_beep();
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        esp_restart();
    }

    flash_packet_queue = xQueueCreate(RING_BUFFER_SIZE, sizeof(flash_packet));
    assert(flash_packet_queue != NULL);


    if(ret == ESP_OK){
        ESP_LOGI(TAG, "Initialized SPI flash!");
    }

    return ret;
}

bool flash_erase_next_bank_no_advance(int64_t max_time, int32_t* resume) {
    int32_t bank;

    if (nvs_get_i32(my_handle, "bank", &bank) != ESP_OK) {
        // now we don't know what bank to use please fail
        printf("Why do we have no bank?\n");
        return false;
    }

    int32_t next_bank = (bank + 1) % BANKS;

    if (flash_erase_bank(next_bank, max_time, resume) != ESP_OK) {
        return false;
    }

    return true;
}

bool flash_prepare_for_flight(void) {
    printf("DOING CHIP ERASE\n");

    if (nvs_get_i32(my_handle, "bank", &current_bank) != ESP_OK) {
        // now we don't know what bank to use please fail
        printf("Why do we have no bank?\n");
        for (int i = 0; i < 5; i++) {
            error_beep();
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        esp_restart();
    }

    // erase next bank
    int32_t resume = -1;
    if (!flash_erase_next_bank_no_advance(0, &resume)) {
        printf("Erasing next bank failed\n");
        for (int i = 0; i < 5; i++) {
            error_beep();
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        esp_restart();
    }

    // actually advance the bank
    current_bank = (current_bank + 1) % BANKS;
    addr = current_bank*BANK_SIZE;

    nvs_set_i32(my_handle, "bank", current_bank);
    printf("Ready to fly using bank: %ld\n", current_bank);

	flash_erase_jingle();
    return true;
}

// max_time == 0 means that the bank must be fully erased and that the function will block to ensure that
static esp_err_t flash_erase_bank(int bank, int64_t max_time, int32_t *resume) {
    int64_t start = esp_timer_get_time();

    printf("Erasing bank: %d\n", bank);
    int32_t used_bytes;
    nvs_get_i32(my_handle, bank_keys[bank], &used_bytes);
    int32_t used = (used_bytes + SECTOR_SIZE - 1)/SECTOR_SIZE;

    static uint32_t i;
    if (max_time == 0) i = 0;
    else {
        if (*resume == -1) i = 0;
        else i = *resume;
    }

    uint32_t base = bank*SECTORS_IN_BANK;

    bool should_stop = ((esp_timer_get_time() - start) >= max_time);
    if (max_time == 0) should_stop = false;
    while (!should_stop && i < used) {
        w25qxx_sector_erase((base+i)*SECTOR_SIZE);
        printf("%f\n", (float) i / (float) used);

        i++;
    }

    if(i >= used) {
        nvs_set_i32(my_handle, bank_keys[bank], 0);
        *resume = -1;
        return ESP_OK;
    } else {
        *resume = i;
        return ESP_ERR_NOT_FINISHED;
    }
}


void flash_dump_to_serial(int bank) {
    flash_packet fp;

    printf("DUMPING DATA FROM BANK: %d\n", bank);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    addr = BANK_SIZE*bank;
    printf("n, timestamp, bat_voltage, flight_state, pyro_cont, pressure, temperature, altitude_agl, ground_altitude, baro_vel, UTCtstamp, lat, lon, altitude_ellipsoid, altitude_msl, fixType, num_sats, acc_x, acc_y, acc_z, hacc_x, hacc_y, hacc_z, gyr_x, gyr_y, gyr_z\n");
    while (addr < MAX_SECTORS*SECTOR_SIZE && addr < BANK_SIZE*bank + BANK_SIZE) {
        w25qxx_read(addr, (uint8_t*)&fp, sizeof(flash_packet));
        addr += sizeof(flash_packet);

        bool all = true;
        char* buf = (char*) &fp;
        for (int i = 0; i < sizeof(flash_packet); i++) {
            if (buf[i] != 0xFF) all=false;
        }
        if (all) break;

        printf("%llu,", fp.n);
        printf("%"PRId64",", fp.timestamp);
        printf("%f,", fp.bat_voltage);
        printf("%d,", fp.flight_state);
        printf("%d,", fp.pyro_cont);

        printf("%f,", fp.pressure);
        printf("%f,", fp.temperature);
        printf("%f,", fp.altitude_agl);
        printf("%f,", fp.ground_altitude);
        printf("%f,", fp.baro_vel);

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
        printf("%f", fp.gyr_z);

        printf("\n");
    }

    flash_erase_jingle();

    printf("FINISHED DUMPING DATA\n");
}

void flash_write_packet(flash_packet *packet) {
    if (addr >= current_bank*BANK_SIZE + BANK_SIZE) {
        printf("BANK OVER RUN, FLASH PACKET LOST\n");
    }

    w25qxx_write(addr, (uint8_t*) packet, sizeof(flash_packet));
    addr += sizeof(flash_packet);

    nvs_set_i32(my_handle, bank_keys[current_bank], addr-(current_bank*BANK_SIZE));
}

void flash_queue_packet(flash_packet *packet) {
    packet->n = n++;
    if (xQueueSendToBack(flash_packet_queue, packet, 0) != pdTRUE) {
        // vTaskDelay(pdMS_TO_TICKS(1)); 
        // printf("QUEUE OVER RUN, FLASH PACKET LOST\n");

        flash_packet oldItem;
        xQueueReceive(flash_packet_queue, &oldItem, 0);
        
        if (xQueueSendToBack(flash_packet_queue, packet, 1) != pdTRUE) {
            // vTaskDelay(pdMS_TO_TICKS(1)); 
            printf("QUEUE OVER RUN, FLASH PACKET LOST\n");
        }
        // printf("QUEUE OVER RUN, FLASH PACKET LOST\n");
    }
}

void flash_write_queue(int64_t max_time) {
    int64_t start = esp_timer_get_time();
    flash_packet packet;

    static UBaseType_t max_count = 0;
    UBaseType_t count = uxQueueMessagesWaiting(flash_packet_queue);
    max_count = count > max_count ? count : max_count;
    // printf("ITEMS IN QUEUE: %u, %u\n", count, max_count);

    while ((esp_timer_get_time() - start) < max_time) {
        if (xQueueReceive(flash_packet_queue, &packet, 0) == pdTRUE) {
            flash_write_packet(&packet);
        } else {
            break; // Queue is empty
        }
    }
}

int32_t flash_get_last_used_bank() {
    return current_bank;
}

void flash_print_stats() {
    for (int i = 0; i < BANKS; i++) {
        int32_t used;
        nvs_get_i32(my_handle, bank_keys[i], &used);
        printf("Bank %d, %ld/%d, %f\n", i, used, BANK_SIZE, (float) used / (float) BANK_SIZE);
    }
    printf("\n");
}

void flash_blank_slate() {
    printf("CHIP ERASE, please be patient lol, this will take 5-10min lol, DO NOT POWER OFF....\n");
    w25qxx_chip_erase();
    nvs_set_i32(my_handle, "bank", 0);
    for (int i = 0; i < BANKS; i++) {
        nvs_set_i32(my_handle, bank_keys[i], 0);
    }
    printf("Done\n");
    flash_erase_jingle();
}

void try_to_dump_data() {
    printf("You have 5 seconds to enter \"DUMP\" to enter data dumping mode\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    char buf[512];
    int i = 0;
    while (serial_util_readline_nonblocking(buf, 512, &i, 1000/portTICK_PERIOD_MS)) {
        if (strcmp("DUMP", buf) == 0) {
            while (true) {
                printf("Enter the bank to dump (last bank used: %ld):\n", flash_get_last_used_bank());
                while (serial_util_readline_nonblocking(buf, 512, &i, 1000/portTICK_PERIOD_MS)) {
                    int bank = atoi(buf);
                    flash_dump_to_serial(bank);
                    for (int j = 0; j < 3; j++) {
                        flash_erase_jingle();
                        vTaskDelay(pdMS_TO_TICKS(500));
                    }
                }
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
