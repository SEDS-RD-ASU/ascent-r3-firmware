#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t primary_task_handle;

void primary_task(void *pvParameters);
extern TaskHandle_t fast_sensor_task_handle;

void fast_sensor_task(void *pvParameters);
extern TaskHandle_t slow_sensor_task_handle;

void slow_sensor_task(void *pvParameters);
extern TaskHandle_t flash_task_handle;

void flash_task(void *pvParameters);
extern TaskHandle_t telemetry_task_handle;

void telemetry_task(void *pvParameters);
extern TaskHandle_t simulator_task_handle;