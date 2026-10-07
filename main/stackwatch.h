#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Create a task and remember it, so the stack space each task has never used can be logged. Only for tasks that run until the dongle
// restarts (a task that deletes itself must not be registered).
BaseType_t stackwatch_create(TaskFunction_t fn, const char *name, uint32_t stack_bytes, void *arg, UBaseType_t priority);

// One log line: for every registered task, the least free stack it has ever had, in bytes
void stackwatch_log(void);
