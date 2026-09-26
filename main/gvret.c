#include "gvret.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "display.h"

#define TAG "GVRET"

// Protocol command bytes
#define PROTO_BUILD_CAN_FRAME   0   // 0x00
#define PROTO_TIME_SYNC         1   // 0x01
#define PROTO_DIG_INPUTS        2   // 0x02
#define PROTO_ANA_INPUTS        3   // 0x03
#define PROTO_SET_DIG_OUT       4   // 0x04
#define PROTO_SET_CONFIG        5   // 0x05
#define PROTO_GET_CANBUS_PARAMS 6   // 0x06
#define PROTO_GET_DEV_INFO      7   // 0x07
#define PROTO_SET_BAUD_0        8   // 0x08
#define PROTO_GET_EXT_BUSES     9   // 0x09
#define PROTO_SET_EXT_BUSES     10  // 0x0A
#define PROTO_GET_NUM_BUSES     12  // 0x0C
#define PROTO_GET_NUM_BUSES_EXT 13  // 0x0D

// Forward frames from vehicle/CAN bus to USB (SavvyCAN)
static void gvret_send_frame(const twai_message_t *frame)
{
    uint8_t buffer[16];
    uint32_t now = (uint32_t)(esp_timer_get_time());

    buffer[0] = 0xF1;
    buffer[1] = PROTO_BUILD_CAN_FRAME;
    buffer[2] = (uint8_t)(now & 0xFF);
    buffer[3] = (uint8_t)((now >> 8) & 0xFF);
    buffer[4] = (uint8_t)((now >> 16) & 0xFF);
    buffer[5] = (uint8_t)((now >> 24) & 0xFF);

    uint32_t id = frame->identifier;
    if (frame->extd) {
        id |= (1UL << 31);
    }
    buffer[6] = (uint8_t)(id & 0xFF);
    buffer[7] = (uint8_t)((id >> 8) & 0xFF);
    buffer[8] = (uint8_t)((id >> 16) & 0xFF);
    buffer[9] = (uint8_t)((id >> 24) & 0xFF);

    buffer[10] = frame->data_length_code & 0x0F;
    memcpy(&buffer[11], frame->data, frame->data_length_code);

    usb_serial_jtag_write_bytes(buffer, 11 + frame->data_length_code, pdMS_TO_TICKS(10));
}

// Background task: Listen for incoming CAN frames and pump to USB
static void gvret_rx_can_task(void *pvParameters)
{
    twai_message_t rx_frame;
    while (1) {
        if (twai_receive(&rx_frame, pdMS_TO_TICKS(50)) == ESP_OK) {
            gvret_send_frame(&rx_frame);
        }
    }
}

// Background task: Handle SavvyCAN USB commands, keepalives, and transmit requests
static void gvret_comm_task(void *pvParameters)
{
    uint8_t rx_buf[128];

    while (1) {
        int len = usb_serial_jtag_read_bytes(rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(20));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t b = rx_buf[i];

                if (b == 0xE7) {
                    continue;
                }

                if (b == 0xF1 && (i + 1 < len)) {
                    uint8_t cmd = rx_buf[++i];

                    switch (cmd) {
                        case PROTO_BUILD_CAN_FRAME: { // 0x00: Transmit frame from SavvyCAN to vehicle
                            // Payload structure from SavvyCAN:
                            // [i+1..i+4] ID (uint32_t, little-endian, MSB bit 31 set if extended)
                            // [i+5] Bus number + flags
                            // [i+6] DLC (length)
                            // [i+7..] Data payload
                            if (i + 6 < len) {
                                twai_message_t tx_frame = {0};
                                uint32_t id = (uint32_t)rx_buf[i + 1] |
                                              ((uint32_t)rx_buf[i + 2] << 8) |
                                              ((uint32_t)rx_buf[i + 3] << 16) |
                                              ((uint32_t)rx_buf[i + 4] << 24);

                                if (id & (1UL << 31)) {
                                    tx_frame.extd = 1;
                                    tx_frame.identifier = id & ~(1UL << 31);
                                } else {
                                    tx_frame.extd = 0;
                                    tx_frame.identifier = id & 0x7FF;
                                }

                                uint8_t dlc = rx_buf[i + 6] & 0x0F;
                                if (dlc > 8) dlc = 8;
                                tx_frame.data_length_code = dlc;

                                if (i + 6 + dlc < len) {
                                    memcpy(tx_frame.data, &rx_buf[i + 7], dlc);
                                    twai_transmit(&tx_frame, pdMS_TO_TICKS(10));
                                    i += (6 + dlc); // Advance pointer past payload
                                }
                            }
                            break;
                        }

                        case PROTO_GET_NUM_BUSES: { // 0x0C: Number of standard buses
                            const uint8_t resp[] = {0xF1, PROTO_GET_NUM_BUSES, 1};
                            usb_serial_jtag_write_bytes(resp, sizeof(resp), pdMS_TO_TICKS(20));
                            break;
                        }

                        case PROTO_GET_NUM_BUSES_EXT: { // 0x0D: Number of extended buses
                            const uint8_t resp[] = {0xF1, PROTO_GET_NUM_BUSES_EXT, 0};
                            usb_serial_jtag_write_bytes(resp, sizeof(resp), pdMS_TO_TICKS(20));
                            break;
                        }

                        case PROTO_GET_DEV_INFO: { // 0x07: Device Info
                            const uint8_t resp[] = {0xF1, PROTO_GET_DEV_INFO, 0x20, 0x01, 0x00, 0x00};
                            usb_serial_jtag_write_bytes(resp, sizeof(resp), pdMS_TO_TICKS(20));
                            break;
                        }

                        case PROTO_GET_CANBUS_PARAMS: { // 0x06: Standard bus params
                            const uint8_t resp[] = {
                                0xF1, PROTO_GET_CANBUS_PARAMS,
                                0x01,                   // Bus 0 active, listen-only off
                                0x20, 0xA1, 0x07, 0x00  // 500,000 baud (little-endian uint32)
                            };
                            usb_serial_jtag_write_bytes(resp, sizeof(resp), pdMS_TO_TICKS(20));
                            break;
                        }

                        case PROTO_GET_EXT_BUSES: { // 0x09: Keepalive ping / ext bus descriptor
                            const uint8_t resp[] = {
                                0xF1, PROTO_GET_EXT_BUSES, 
                                0x01, // Bus 0 exists
                                0x00, // Standard CAN
                                0x00, 
                                0x00
                            };
                            usb_serial_jtag_write_bytes(resp, sizeof(resp), pdMS_TO_TICKS(20));
                            break;
                        }

                        case PROTO_TIME_SYNC: { // 0x01: Microsecond clock synchronization
                            uint32_t now = (uint32_t)esp_timer_get_time();
                            uint8_t resp[6];
                            resp[0] = 0xF1;
                            resp[1] = PROTO_TIME_SYNC;
                            resp[2] = (uint8_t)(now & 0xFF);
                            resp[3] = (uint8_t)((now >> 8) & 0xFF);
                            resp[4] = (uint8_t)((now >> 16) & 0xFF);
                            resp[5] = (uint8_t)((now >> 24) & 0xFF);
                            usb_serial_jtag_write_bytes(resp, sizeof(resp), pdMS_TO_TICKS(20));
                            break;
                        }

                        default:
                            break;
                    }
                }
            }
        }
    }
}

void gvret_start(void)
{
    display_set_status("SAVVYCAN", "USB READY", COLOR_GREEN);

    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 2048,
    };
    usb_serial_jtag_driver_install(&usb_cfg);

    xTaskCreate(gvret_rx_can_task, "gvret_can_rx", 4096, NULL, 5, NULL);
    xTaskCreate(gvret_comm_task,   "gvret_comm",   4096, NULL, 5, NULL);
}