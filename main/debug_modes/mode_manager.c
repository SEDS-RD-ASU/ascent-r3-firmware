#include "mode_manager.h"

#include <driver/gpio.h>
#include "esp_timer.h"
#include "esp_log.h"

#include <ascent_r3_hardware_definition.h>
#include "beep.h"

#define MODE_BTN_PIN GPIO_NUM_0

// Current mode bitmask
// b0 = debug mode
// b1 = simulator mode
// b2 = flash dump mode
uint8_t current_mode_mask = 0b000;
int last_press = 0;

// MARK: BUTTON HANDLING
esp_err_t initialize_mode_button(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODE_BTN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&io_conf);
}

bool mode_detect_boot_press_ms(uint32_t ms) {
    int prev = gpio_get_level(MODE_BTN_PIN);
    uint32_t elapsed = 0;
    const uint32_t step_ms = 20; // check every 20ms

    while (elapsed < ms) {
        int now = gpio_get_level(MODE_BTN_PIN);

        if (prev == 1 && now == 0) { // Detect falling edge
            return true;
        }

        prev = now;
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        elapsed += step_ms;
    }
    return false;
}

// MARK: MODE SELECTION
void cycle_mode(void)
{
    // Debounce: ignore presses within 100ms
    if (esp_timer_get_time() - last_press < 100000) {
        return;
    }
    last_press = esp_timer_get_time();

    switch (current_mode_mask)
    {
        case 0b000:                     // Default (flight) mode
            current_mode_mask = 0b001;  // Debug mode
            low_beep();low_beep();
            ESP_LOGI("mode_manager", "Switched to DEBUG MODE");
            break;
        case 0b001:                     // Debug mode
            current_mode_mask = 0b010;  // Simulator mode
            low_beep();low_beep();low_beep();
            ESP_LOGI("mode_manager", "Switched to SIMULATOR MODE");
            break;
        case 0b010:                     // Simulator mode
            current_mode_mask = 0b011;  // Sim + Debug mode
            low_beep();low_beep();low_beep();low_beep();
            ESP_LOGI("mode_manager", "Switched to SIM + DEBUG MODE");
            break;
        case 0b011:                     // Sim + Debug mode
            current_mode_mask = 0b100;  // Flash dump mode
            low_beep();low_beep();low_beep();low_beep();low_beep();
            ESP_LOGI("mode_manager", "Switched to FLASH DUMP MODE");
            break;
        case 0b100:                     // Flash dump mode
            current_mode_mask = 0b000;  // Default (flight) mode
            low_beep();
            ESP_LOGI("mode_manager", "Switched to FLIGHT MODE");
            break;
        default:
            current_mode_mask = 0b000;  // Reset to default (flight) mode on invalid state
            low_beep();
            ESP_LOGW("mode_manager", "Invalid mode state detected, resetting to FLIGHT MODE");
            break;
    }
}

void mode_run_selection_window_ms(uint32_t ms) {
    uint32_t elapsed = 0;
    const uint32_t step_ms = 20; // check every 20ms
    int prev = gpio_get_level(MODE_BTN_PIN);

    low_beep();high_beep();

    ESP_LOGI("mode_manager", "Entering mode selection window.");

    while (elapsed < ms) {
        int now = gpio_get_level(MODE_BTN_PIN);

        if (prev == 1 && now == 0) { // Detect falling edge
            cycle_mode();
            elapsed = 0; // reset timer on press
        }

        prev = now;
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        elapsed += step_ms;
    }

    ESP_LOGI("mode_manager", "Exiting mode selection window. Final mode: %s%s%s",
             is_debug_mode() ? "DEBUG " : "",
             is_simulator_mode() ? "SIMULATOR " : "",
             is_flash_dump_mode() ? "FLASH DUMP " : "");
    mcdonalds(); // signal end of mode selection
    // should probably change this to a uinque pattern in the future
}


// MARK: MODE QUERIES
bool is_debug_mode(void)
{
    return (current_mode_mask & 0b001) != 0;
}

bool is_simulator_mode(void)
{
    return (current_mode_mask & 0b010) != 0;
}

bool is_flash_dump_mode(void)
{
    return (current_mode_mask & 0b100) != 0;
}