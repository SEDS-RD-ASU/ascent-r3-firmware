#include "freertos/FreeRTOS.h"

esp_err_t initialize_mode_button(void);
bool mode_detect_boot_press_ms(uint32_t ms);

void mode_run_selection_window_ms(uint32_t ms);

bool is_debug_mode(void);
bool is_simulator_mode(void);
bool is_flash_dump_mode(void);