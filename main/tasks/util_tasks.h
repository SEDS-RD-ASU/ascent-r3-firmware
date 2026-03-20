#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SIMULATOR_TASK_STACK_SIZE 4096\

void simulator_task(void *pvParameters);
extern TaskHandle_t simulator_task_handle;


void measure_performance(void);

void validate_esp(void);