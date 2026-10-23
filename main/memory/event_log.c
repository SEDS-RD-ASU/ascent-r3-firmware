#include "event_log.h"

static QueueHandle_t event_radio_queue;
static QueueHandle_t event_flash_queue;
#define EVENT_QUEUE_SIZE 20

bool event_log_init(void){
    //create the rtos queue
    event_radio_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(event_record_t));
    event_flash_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(event_record_t));
    return event_radio_queue != NULL && event_flash_queue != NULL;
}
bool enqueue_event(const event_record_t *event){
    //add event to the rtos queue
    bool radio_sent = xQueueSend(event_radio_queue, event, 0) == pdPASS;
    bool flash_sent = xQueueSend(event_flash_queue, event, 0) == pdPASS;
    return radio_sent && flash_sent; //sends the copy of the event to both the flash queue and the radio queue, returns true if both were successful
}
bool dequeue_radio_event(event_record_t *event){
    //remove event from rtos queue
    return xQueueReceive(event_radio_queue, event, 0) == pdPASS;
}
bool dequeue_flash_event(event_record_t *event){
    //remove event from rtos queue
    return xQueueReceive(event_flash_queue, event, 0) == pdPASS;
}