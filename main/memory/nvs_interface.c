#include "nvs_interface.h"

#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_psram.h"
#include "esp_task_wdt.h"
#include <inttypes.h>
#include <rom/ets_sys.h>
#include "flight_config.h"

static nvs_handle_t my_handle;

// #define PROVISION

esp_err_t nvs_interface_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // nvs_flash_erase();
        // nvs_flash_init();

        // here the nvs flash has been over run for some reason and needs to be
        // totally erased
        // so we don't lose bank data this throws an error and
        // requires a manual recompile to call the above functions after all
        // data from banks has been saved and the whole external flash chip has
        // been erased.
        // use the above two commented out lines...

        printf("Failed initalize nvs flash\n");
    }

    if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK) {
        printf("Failed open nvs storage\n");
    }

    return err;
}

nvs_handle_t nvs_interface_get_handle(void)
{
    return my_handle;
}

esp_err_t nvs_retreive_flight_config(flight_config_t *flight_config){
    nvs_handle_t my_handle = nvs_interface_get_handle();
    size_t length = sizeof(flight_config_t);
    esp_err_t err = nvs_get_blob(my_handle, "flight_config", flight_config, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("flight_config not found\n");
        return ESP_ERR_NVS_NOT_FOUND;
    } else if (err != ESP_OK || length != sizeof(flight_config_t)) {
        printf("Failed to load flight_config (err=%d, len=%u)\n", (int)err, (unsigned)length);
        return err != ESP_OK ? err : ESP_ERR_NVS_INVALID_LENGTH;
    }
    return ESP_OK;
}

esp_err_t nvs_set_flight_config(flight_config_t *flight_config){
    printf("Setting flight_config\n");
    return nvs_set_blob(my_handle, "flight_config", flight_config, sizeof(flight_config_t));
}

esp_err_t nvs_retrieve_board_info(board_information_t *board_info){
    nvs_handle_t my_handle = nvs_interface_get_handle();
    size_t length = sizeof(board_information_t);
    esp_err_t err = nvs_get_blob(my_handle, "board_info", board_info, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("board_info not found\n");
        return ESP_ERR_NVS_NOT_FOUND;
    } else if (err != ESP_OK || length != sizeof(board_information_t)) {
        printf("Failed to load board_info (err=%d, len=%u)\n", (int)err, (unsigned)length);
        return err != ESP_OK ? err : ESP_ERR_NVS_INVALID_LENGTH;
    }
    return ESP_OK;
}

esp_err_t nvs_set_board_info(board_information_t *board_info){
    printf("Setting board_info\n");
    return nvs_set_blob(my_handle, "board_info", board_info, sizeof(board_information_t));
}

void print_board_info(){
    board_information_t board_info;

    #ifdef PROVISION
    board_information_t provisioned_info = {
        .model = 3,
        .hw_rev = 1,
        .fw_rev = 1,
        .serial_number = 2,
        .passed_hw_validation = 99,
        .manufacture_day = 19,
        .manufacture_month = 1,
        .manufacture_year = 26
    };
    nvs_set_board_info(&provisioned_info);
    #endif

    esp_err_t err = nvs_retrieve_board_info(&board_info);

    if (err != ESP_OK) {
        printf("Failed to retrieve board_info for printing\n");
        return;
    }
    
    printf("\n===== ASCENT R%d =====\n", board_info.model);
    printf("Serial Number: %d\n", board_info.serial_number);
    printf("Hardware Version: %d\n", board_info.hw_rev);
    printf("Firmware Version: %d\n", board_info.fw_rev);
    printf("Passed HW Validation: %d\n", board_info.passed_hw_validation);
    printf("Manufactured on %d/%d/%d\n", board_info.manufacture_month, board_info.manufacture_day, board_info.manufacture_year);
    printf("=========================\n\n");
}

uint8_t board_serial_number(void)
{
    static uint8_t serial_num = 0;
    
    if(serial_num)
    {
        return serial_num;
    }

    board_information_t board_info;
    nvs_retrieve_board_info(&board_info);
    serial_num = board_info.serial_number;

    return serial_num;
}