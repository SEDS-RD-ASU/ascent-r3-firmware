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

void nvs_interface_init(void)
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

        esp_restart();
    }

    if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK) {
        printf("Failed open nvs storage\n");
    }
}

nvs_handle_t nvs_interface_get_handle(void)
{
    return my_handle;
}

void nvs_retreive_matrices(float (*acc_correction_matrix)[3], float (*gyr_correction_matrix)[3], float (*mag_correction_matrix)[3], float (*high_g_correction_matrix)[3], float acc_bias_vector[3], float gyr_bias_vector[3], float mag_bias_vector[3], float high_g_bias_vector[3]) {
    my_handle = nvs_interface_get_handle();

    esp_err_t err;

    // matrices: 3x3 (9 floats)
    size_t mat_size = sizeof(float) * 9;
    size_t vec_size = sizeof(float) * 3;

    // acc matrix
    size_t length = mat_size;
    err = nvs_get_blob(my_handle, "acc_mat", (float*)acc_correction_matrix, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("acc_mat not found; using defaults from C file\n");
    } else if (err != ESP_OK || length != mat_size) {
        printf("Failed to load acc_correction_matrix (err=%d, len=%u)\n", (int)err, (unsigned)length);
        esp_restart();
    }

    // gyr matrix
    length = mat_size;
    err = nvs_get_blob(my_handle, "gyr_mat", (float*)gyr_correction_matrix, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("gyr_mat not found; using defaults from C file\n");
    } else if (err != ESP_OK || length != mat_size) {
        printf("Failed to load gyr_correction_matrix (err=%d, len=%u)\n", (int)err, (unsigned)length);
        esp_restart();
    }

    // mag matrix
    length = mat_size;
    err = nvs_get_blob(my_handle, "mag_mat", (float*)mag_correction_matrix, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("mag_mat not found; using defaults from C file\n");
    } else if (err != ESP_OK || length != mat_size) {
        printf("Failed to load mag_correction_matrix (err=%d, len=%u)\n", (int)err, (unsigned)length);
        esp_restart();
    }

    // high-g matrix
    length = mat_size;
    err = nvs_get_blob(my_handle, "high_g_mat", (float*)high_g_correction_matrix, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("high_g_mat not found; using defaults from C file\n");
    } else if (err != ESP_OK || length != mat_size) {
        printf("Failed to load high_g_correction_matrix (err=%d, len=%u)\n", (int)err, (unsigned)length);
        esp_restart();
    }

    // vectors: 3 floats
    length = vec_size;
    err = nvs_get_blob(my_handle, "acc_vec", acc_bias_vector, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("acc_vec not found; using defaults from C file\n");
    } else if (err != ESP_OK || length != vec_size) {
        printf("Failed to load acc_bias_vector (err=%d, len=%u)\n", (int)err, (unsigned)length);
        esp_restart();
    }

    length = vec_size;
    err = nvs_get_blob(my_handle, "gyr_vec", gyr_bias_vector, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("gyr_vec not found; using defaults from C file\n");
    } else if (err != ESP_OK || length != vec_size) {
        printf("Failed to load gyr_bias_vector (err=%d, len=%u)\n", (int)err, (unsigned)length);
        esp_restart();
    }

    length = vec_size;
    err = nvs_get_blob(my_handle, "mag_vec", mag_bias_vector, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("mag_vec not found; using defaults from C file\n");
    } else if (err != ESP_OK || length != vec_size) {
        printf("Failed to load mag_bias_vector (err=%d, len=%u)\n", (int)err, (unsigned)length);
        esp_restart();
    }

    length = vec_size;
    err = nvs_get_blob(my_handle, "high_g_vec", high_g_bias_vector, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("high_g_vec not found; using defaults from C file\n");
    } else if (err != ESP_OK || length != vec_size) {
        printf("Failed to load high_g_bias_vector (err=%d, len=%u)\n", (int)err, (unsigned)length);
    }
}

void nvs_retreive_uuid(uint8_t *uuid){
    nvs_handle_t my_handle = nvs_interface_get_handle();
    size_t length = 16;
    esp_err_t err = nvs_get_blob(my_handle, "uuid", uuid, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("uuid not found; using fallback uuid\n");
    } else if (err != ESP_OK || length != 16) {
        printf("Failed to load uuid (err=%d, len=%u)\n", (int)err, (unsigned)length);
    }
}

esp_err_t nvs_retreive_flight_config(flight_config_t *flight_config){
    nvs_handle_t my_handle = nvs_interface_get_handle();
    size_t length = sizeof(flight_config_t);
    esp_err_t err = nvs_get_blob(my_handle, "flight_config", flight_config, &length);
    printf("retreived flight_config from nvs\n");
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

// esp_err_t nvs_retreive_lora_config(lora_config_t *lora_config){
//     nvs_handle_t my_handle = nvs_interface_get_handle();
//     size_t length = sizeof(lora_config_t);
//     esp_err_t err = nvs_get_blob(my_handle, "lora_config", lora_config, &length);
//     printf("retreived lora_config from nvs\n");
//     if (err == ESP_ERR_NVS_NOT_FOUND) {
//         printf("lora_config not found\n");
//         return ESP_ERR_NVS_NOT_FOUND;
//     }
//     else if (err != ESP_OK || length != sizeof(lora_config_t)) {
//         printf("Failed to load lora_config (err=%d, len=%u)\n", (int)err, (unsigned)length);
//         return err != ESP_OK ? err : ESP_ERR_NVS_INVALID_LENGTH;
//     }
//     return ESP_OK;
// }

// esp_err_t nvs_set_lora_config(lora_config_t *lora_config){
//     printf("Setting lora_config\n");
//     return nvs_set_blob(my_handle, "lora_config", lora_config, sizeof(lora_config_t));
// }