#ifndef NVS_INTERFACE
#define NVS_INTERFACE

#include "nvs_flash.h"
#include "nvs.h"

#include "flight_config.h"

typedef struct {
    uint8_t model;
    uint8_t hw_rev;
    uint8_t fw_rev;
    uint8_t serial_number;
    uint8_t passed_hw_validation;
    uint8_t manufacture_month;
    uint8_t manufacture_day;
    uint8_t manufacture_year;
} board_information_t;

esp_err_t nvs_interface_init(void);

nvs_handle_t nvs_interface_get_handle(void);

esp_err_t nvs_retreive_flight_config(flight_config_t *flight_config);
esp_err_t nvs_set_flight_config(flight_config_t *flight_config);

esp_err_t nvs_retrieve_board_info(board_information_t *board_info);
esp_err_t nvs_set_board_info(board_information_t *board_info);
void print_board_info();

#endif