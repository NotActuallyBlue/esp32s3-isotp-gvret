#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "stackwatch.h"

#define MAX_WATCHED 32

typedef struct {
    char        name[24];
    TaskHandle_t handle;
} watched_t;

static watched_t        watched[MAX_WATCHED];
static int              watched_count;
static portMUX_TYPE     lock = portMUX_INITIALIZER_UNLOCKED;

BaseType_t stackwatch_create(TaskFunction_t fn, const char *name, uint32_t stack_bytes, void *arg, UBaseType_t priority)
{
    TaskHandle_t handle = NULL;
    BaseType_t ok = xTaskCreate(fn, name, stack_bytes, arg, priority, &handle);
    if (ok == pdPASS && handle) {
        taskENTER_CRITICAL(&lock);
        if (watched_count < MAX_WATCHED) {
            strlcpy(watched[watched_count].name, name, sizeof(watched[0].name));
            watched[watched_count].handle = handle;
            watched_count++;
        }
        taskEXIT_CRITICAL(&lock);
    }
    return ok;
}

void stackwatch_log(void)
{
    // Several short lines: the flash log keeps lines short
    char line[200];
    int n = 0;
    for (int i = 0; i < watched_count; i++) {
        if (n == 0) n = snprintf(line, sizeof(line), "Stack never used (bytes):");
        n += snprintf(line + n, sizeof(line) - n, " %s=%u", watched[i].name, (unsigned)uxTaskGetStackHighWaterMark(watched[i].handle));
        if (n > 140 || i == watched_count - 1) {
            ESP_LOGI("Stacks", "%s", line);
            n = 0;
        }
    }
}
