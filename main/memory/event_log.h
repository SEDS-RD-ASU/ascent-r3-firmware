#ifndef EVENT_LOG_H
#define EVENT_LOG_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    EVENT_SEVERITY_INFO = 0,
    EVENT_SEVERITY_WARNING,
    EVENT_SEVERITY_ERROR,
    EVENT_SEVERITY_CRITICAL,
} event_severity_t;

typedef enum {
    EVENT_TYPE_STATE_TRANSITION = 1,
    EVENT_PYRO_ACTIVATED,
    EVENT_TYPE_BURNOUT,
    EVENT_TYPE_MAIN_DEPLOYMENT,
    EVENT_TYPE_LANDED,
} event_type_t;

typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence_number;
    event_type_t event_type;
    uint8_t event_severity;
    uint8_t previous_event;
    uint8_t new_event;
} event_record_t;

bool event_log_init(void);
bool enqueue_event(const event_record_t *event);
bool dequeue_radio_event(event_record_t *event);
bool dequeue_flash_event(event_record_t *event);
#endif 