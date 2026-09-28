#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "driver/uart.h"

// ==========================================
// Dongle Operational Modes
// ==========================================
typedef enum {
    OP_MODE_SIMOS_BLE = 0,
    OP_MODE_SAVVYCAN_GVRET,
    OP_MODE_ELM327,
    OP_MODE_COUNT,
} dongle_mode_t;

// Global synchronization primitives
extern SemaphoreHandle_t                sync_task_sem;

// Type definitions & macros
typedef int16_t                         bool16;
#define tMUTEX(x)                       xSemaphoreTake(x, portMAX_DELAY)
#define rMUTEX(x)                       xSemaphoreGive(x)

// Settings / NVS keys
#define BRG_SETTING_ISOTP_STMIN         1
#define BRG_SETTING_PERSIST_DELAY       3
#define BRG_SETTING_PERSIST_Q_DELAY     4
#define BRG_SETTING_BLE_SEND_DELAY      5
#define BRG_SETTING_BLE_MULTI_DELAY     6
#define BRG_SETTING_PASSWORD            7
#define BRG_SETTING_GAP                 8

// FreeRTOS Task Priorities & Stacks
#define TASK_STACK_SIZE                 3072
#define TWAI_TASK_PRIO                  3 // Rapid CAN processing for ISO15765-2
#define ISOTP_TSK_PRIO                  2 // ISO-TP message pump
#define MAIN_TSK_PRIO                   1 // Coordinated delivery with BLE stack
#define PERSIST_TSK_PRIO                0
#define HANDLER_TSK_PRIO                0
#define UART_TSK_PRIO                   1

// ==========================================
// Adafruit Feather ESP32-S3 TFT Pinout
// ==========================================
// CAN Transceiver Interface (Adafruit CAN Pal)
#define CAN_TX_PORT                     5  // Feather GPIO 5 -> CAN Pal TX
#define CAN_RX_PORT                     4  // Feather GPIO 4 -> CAN Pal RX

// Hardware User Button
#define BOOT_BUTTON_PIN                 0  // Feather Onboard BOOT Button (SW)

// Queue and Buffer Capacities
#define ISOTP_QUEUE_SIZE                64
#define UART_QUEUE_SIZE                 96
#define ISOTP_BUFFER_SIZE               4096
#define ISOTP_BUFFER_SIZE_SMALL         512

// Communication Timeouts (in ms / ticks)
#define TIMEOUT_SHORT                   50
#define TIMEOUT_NORMAL                  100
#define TIMEOUT_LONG                    1000
#define TIMEOUT_CANCONNECTION           2
#define TIMEOUT_FIRSTBOOT               30
#define TIMEOUT_UARTCONNECTION          120
#define TIMEOUT_UARTPACKET              1

// Security & BLE Configuration
//#define PASSWORD_CHECK 
#define MAX_PASSWORD_LENGTH             64
#define PASSWORD_KEY                    "Password"
#define PASSWORD_DEFAULT                "BLE2"
#define BLE_GAP_KEY                     "GAP"

// TWAI / CAN Bus Controller Settings
#define CAN_INTERNAL_BUFFER_SIZE        1024
#define CAN_CLK_IO                      TWAI_IO_UNUSED
#define CAN_BUS_IO                      TWAI_IO_UNUSED
#define CAN_MODE                        TWAI_MODE_NORMAL
#define CAN_ALERTS                      (TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED)
#define CAN_FLAGS                       ESP_INTR_FLAG_LEVEL1
#define CAN_CLK_DIVIDER                 0
#define CAN_TIMING                      TWAI_TIMING_CONFIG_500KBITS()
#define CAN_FILTER                      TWAI_FILTER_CONFIG_ACCEPT_ALL()

// USB / UART Serial Settings (SavvyCAN / Debug)
#define UART_TXD                        UART_PIN_NO_CHANGE
#define UART_RXD                        UART_PIN_NO_CHANGE
#define UART_RTS                        UART_PIN_NO_CHANGE
#define UART_CTS                        UART_PIN_NO_CHANGE
#define UART_PORT_NUM                   UART_NUM_0
#define UART_BAUD_RATE                  250000
#define UART_BUFFER_SIZE                8192
#define UART_INTERNAL_BUFFER_SIZE       2048
//#define UART_ECHO

// Persistence & Buffer Timing
#define PERSIST_COUNT                   2
#define PERSIST_MAX_MESSAGE             64
#define PERSIST_DEFAULT_MESSAGE_DELAY   20
#define PERSIST_DEFAULT_QUEUE_DELAY     10

#endif // CONSTANTS_H