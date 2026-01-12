#ifndef NVS_INTERFACE
#define NVS_INTERFACE

#include "nvs_flash.h"
#include "nvs.h"

#include "flight_config.h"

typedef struct lora_config lora_config_t; // forward declaration

esp_err_t nvs_interface_init(void);

nvs_handle_t nvs_interface_get_handle(void);
void nvs_retreive_matrices(float (*acc_correction_matrix)[3], float (*gyr_correction_matrix)[3], float (*mag_correction_matrix)[3], float (*high_g_correction_matrix)[3], float acc_bias_vector[3], float gyr_bias_vector[3], float mag_bias_vector[3], float high_g_bias_vector[3]);
void nvs_retreive_uuid(uint8_t *uuid);
esp_err_t nvs_retreive_flight_config(flight_config_t *flight_config);
esp_err_t nvs_set_flight_config(flight_config_t *flight_config);
esp_err_t nvs_retreive_lora_config(lora_config_t *lora_config);
esp_err_t nvs_set_lora_config(lora_config_t *lora_config);
#endif