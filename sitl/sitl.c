#include "sitl.h"

#include "stdint.h"
#include "stdbool.h"
#include "esp_timer.h"

#include "lora_interface.h"

#define BUF_SIZE (2048)
#include "string.h"

#include "serial_util.h"

int64_t start_time;

double time;

float alt;
float vert_accl;

char buf[BUF_SIZE];

void sitl_init() {
    // lora_sitl_fake_tx_lock();

    start_time = esp_timer_get_time();

    time = 0;
    
    alt = 0.0;
    vert_accl = 0.0;
}

void pull_from_uart() {
    static int i = 0;
    if (serial_util_readline_nonblocking(buf, BUF_SIZE, &i, 10/portTICK_PERIOD_MS)) {
        sscanf(buf, "%f, %f", &alt, &vert_accl);
    }
    // printf("%f, %f\n", alt, vert_accl);
}

void sitl_update() {
    pull_from_uart();
}

float get_current_vertical_accl() {
    return vert_accl;
}

double get_current_baro_alt() {
    return alt;
}