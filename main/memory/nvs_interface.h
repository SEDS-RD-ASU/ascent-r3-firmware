#ifndef NVS_INTERFACE
#define NVS_INTERFACE

#include "nvs_flash.h"
#include "nvs.h"

#include "flight_config.h"

/**
 * @brief Initialize access to the non-volatile storage partition.
 */
void nvs_interface_init(void);

typedef struct lora_config lora_config_t; // forward declaration

/**
 * @brief Retrieve the global NVS handle used by the firmware.
 *
 * @return nvs_handle_t Handle to the opened NVS namespace.
 */
nvs_handle_t nvs_interface_get_handle(void);

/**
 * @brief Retrieve IMU calibration matrices and bias vectors from NVS.
 */
void nvs_retreive_matrices(float (*acc_correction_matrix)[3], float (*gyr_correction_matrix)[3], float (*mag_correction_matrix)[3], float (*high_g_correction_matrix)[3], float acc_bias_vector[3], float gyr_bias_vector[3], float mag_bias_vector[3], float high_g_bias_vector[3]);

/**
 * @brief Retrieve the stored UUID from NVS.
 *
 * @param uuid Output buffer to receive the UUID bytes.
 */
void nvs_retreive_uuid(uint8_t *uuid);

/**
 * @brief Load the persisted flight configuration from NVS.
 *
 * @param flight_config Output pointer to the configuration to populate.
 * @return esp_err_t ESP_OK on success or an error code from the NVS APIs.
 */
esp_err_t nvs_retreive_flight_config(flight_config_t *flight_config);

/**
 * @brief Persist the provided flight configuration to NVS.
 *
 * @param flight_config Configuration to store.
 * @return esp_err_t ESP_OK on success or an error code from the NVS APIs.
 */
esp_err_t nvs_set_flight_config(flight_config_t *flight_config);

/**
 * @brief Load the stored LoRa configuration.
 *
 * @param lora_config Output pointer to populate with stored values.
 * @return esp_err_t ESP_OK on success or an error code from the NVS APIs.
 */
esp_err_t nvs_retreive_lora_config(lora_config_t *lora_config);

/**
 * @brief Persist the provided LoRa configuration to NVS.
 *
 * @param lora_config Configuration to store.
 * @return esp_err_t ESP_OK on success or an error code from the NVS APIs.
 */
esp_err_t nvs_set_lora_config(lora_config_t *lora_config);
#endif
