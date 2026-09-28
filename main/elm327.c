#include "elm327.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "ble_server.h"

#define TAG "ELM327"

#define ELM_RX_BUF_SIZE     128
#define ELM_TX_BUF_SIZE     256

static bool echo_on = true;
static bool headers_on = false;
static bool linefeeds_on = true;

static char rx_line_buf[ELM_RX_BUF_SIZE];
static size_t rx_line_pos = 0;

static QueueHandle_t elm_cmd_queue = NULL;

static void elm_send_string(const char *str)
{
    if (!str || strlen(str) == 0) return;
    ble_send(0, 0, 0, (const void *)str, strlen(str));
}

static void elm_send_prompt(void)
{
    if (linefeeds_on) {
        elm_send_string("\r\n>");
    } else {
        elm_send_string("\r>");
    }
}

static void str_clean(char *str)
{
    char *src = str, *dst = str;
    while (*src) {
        if (!isspace((unsigned char)*src)) {
            *dst++ = toupper((unsigned char)*src);
        }
        src++;
    }
    *dst = '\0';
}

static void handle_at_command(const char *cmd)
{
    char resp[64] = {0};

    if (strcmp(cmd, "ATZ") == 0 || strcmp(cmd, "ATWS") == 0) {
        snprintf(resp, sizeof(resp), "\r\nELM327 v1.5\r\n");
    } else if (strcmp(cmd, "ATE0") == 0) {
        echo_on = false;
        snprintf(resp, sizeof(resp), "OK\r\n");
    } else if (strcmp(cmd, "ATE1") == 0) {
        echo_on = true;
        snprintf(resp, sizeof(resp), "OK\r\n");
    } else if (strcmp(cmd, "ATH0") == 0) {
        headers_on = false;
        snprintf(resp, sizeof(resp), "OK\r\n");
    } else if (strcmp(cmd, "ATH1") == 0) {
        headers_on = true;
        snprintf(resp, sizeof(resp), "OK\r\n");
    } else if (strcmp(cmd, "ATL0") == 0) {
        linefeeds_on = false;
        snprintf(resp, sizeof(resp), "OK\r\n");
    } else if (strcmp(cmd, "ATL1") == 0) {
        linefeeds_on = true;
        snprintf(resp, sizeof(resp), "OK\r\n");
    } else if (strncmp(cmd, "ATSP", 4) == 0) {
        snprintf(resp, sizeof(resp), "OK\r\n");
    } else if (strcmp(cmd, "ATDP") == 0) {
        snprintf(resp, sizeof(resp), "ISO 15765-4 (CAN 11/500)\r\n");
    } else if (strcmp(cmd, "ATRV") == 0) {
        snprintf(resp, sizeof(resp), "12.6V\r\n");
    } else if (strcmp(cmd, "AT@1") == 0) {
        snprintf(resp, sizeof(resp), "OBDII-BLE-DONGLE\r\n");
    } else if (strncmp(cmd, "AT", 2) == 0) {
        snprintf(resp, sizeof(resp), "OK\r\n");
    } else {
        snprintf(resp, sizeof(resp), "?\r\n");
    }

    elm_send_string(resp);
    elm_send_prompt();
}

static void handle_obd_hex(const char *cmd)
{
    size_t len = strlen(cmd);
    if (len % 2 != 0 || len > 14) {
        elm_send_string("?\r\n");
        elm_send_prompt();
        return;
    }

    uint8_t payload[8] = {0};
    uint8_t req_bytes = len / 2;

    payload[0] = req_bytes;

    for (int i = 0; i < req_bytes; i++) {
        char byte_str[3] = { cmd[i * 2], cmd[i * 2 + 1], '\0' };
        payload[1 + i] = (uint8_t)strtol(byte_str, NULL, 16);
    }

    for (int i = 1 + req_bytes; i < 8; i++) {
        payload[i] = 0x55;
    }

    twai_message_t tx_msg = {
        .identifier = 0x7DF,
        .flags = TWAI_MSG_FLAG_NONE,
        .data_length_code = 8,
    };
    memcpy(tx_msg.data, payload, 8);

    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(50)) != ESP_OK) {
        ESP_LOGW(TAG, "TWAI tx failed");
        elm_send_string("CAN ERROR\r\n");
        elm_send_prompt();
        return;
    }

    twai_message_t rx_msg;
    bool got_response = false;
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(200)) {
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(20)) == ESP_OK) {
            if (rx_msg.identifier >= 0x7E8 && rx_msg.identifier <= 0x7EF) {
                if (rx_msg.data_length_code > 1) {
                    char resp_line[64] = {0};
                    int pos = 0;

                    if (headers_on) {
                        pos += snprintf(resp_line + pos, sizeof(resp_line) - pos, "%03lX ", (unsigned long)rx_msg.identifier);
                    }

                    uint8_t data_len = rx_msg.data[0] & 0x0F;
                    if (data_len == 0 || data_len > 7) data_len = 7;

                    for (int i = 1; i <= data_len; i++) {
                        pos += snprintf(resp_line + pos, sizeof(resp_line) - pos, "%02X ", rx_msg.data[i]);
                    }
                    snprintf(resp_line + pos, sizeof(resp_line) - pos, "\r\n");

                    elm_send_string(resp_line);
                    got_response = true;
                    break;
                }
            }
        }
    }

    if (!got_response) {
        elm_send_string("NO DATA\r\n");
    }

    elm_send_prompt();
}

static void elm327_task(void *pvParameters)
{
    char line[ELM_RX_BUF_SIZE];

    while (1) {
        if (xQueueReceive(elm_cmd_queue, line, portMAX_DELAY) == pdTRUE) {
            str_clean(line);

            if (strlen(line) == 0) {
                elm_send_prompt();
                continue;
            }

            if (strncmp(line, "AT", 2) == 0) {
                handle_at_command(line);
            } else {
                handle_obd_hex(line);
            }
        }
    }
}

void elm327_init(void)
{
    rx_line_pos = 0;
    if (elm_cmd_queue == NULL) {
        elm_cmd_queue = xQueueCreate(8, ELM_RX_BUF_SIZE);
    }
}

void elm327_start(void)
{
    xTaskCreate(elm327_task, "elm327_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "ELM327 processing task started");
}

void elm327_rx_byte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (rx_line_pos > 0) {
            rx_line_buf[rx_line_pos] = '\0';
            xQueueSend(elm_cmd_queue, rx_line_buf, 0);
            rx_line_pos = 0;
        }
    } else {
        if (rx_line_pos < (ELM_RX_BUF_SIZE - 1)) {
            rx_line_buf[rx_line_pos++] = (char)byte;
        }
    }
}

void elm327_rx_data(const void *data, size_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        elm327_rx_byte(bytes[i]);
    }
}