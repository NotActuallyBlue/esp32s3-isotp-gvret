#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "constants.h"

// BLE header ID
#define BLE_HEADER_ID                    0xF1
#define BLE_PARTIAL_ID                   0xF2

#define MAX_GAP_LENGTH                   14
#define DEFAULT_GAP_NAME                 "BLE_TO_ISOTP20"

// BLE command flags
#define BLE_COMMAND_FLAG_PER_ENABLE       1
#define BLE_COMMAND_FLAG_PER_CLEAR        2
#define BLE_COMMAND_FLAG_PER_ADD          4
#define BLE_COMMAND_FLAG_SPLIT_PK         8
#define BLE_COMMAND_FLAG_SETTINGS_GET    64
#define BLE_COMMAND_FLAG_SETTINGS        128

// BLE send queue & stack size
#define BLE_QUEUE_SIZE                   64
#define BLE_STACK_SIZE                   4096   // Increased from 2048 to prevent stack canary panics
#define BLE_TASK_PRIORITY                1

// BLE max congestion
#define BLE_CONGESTION_MAX               5000

#define SPP_DATA_MAX_LEN                 (512)
#define SPP_CMD_MAX_LEN                  (20)
#define SPP_STATUS_MAX_LEN               (20)

enum {
    SPP_IDX_SVC,
    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,
    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,
    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,
    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,
    SPP_IDX_NB,
};

typedef struct send_message {
    int32_t msg_length;
    uint8_t* buffer;
    uint16_t rxID;
    uint16_t txID;
    uint8_t  flags;
} send_message_t;

typedef struct ble_header {
    uint8_t   hdID;
    uint8_t   cmdFlags;
    uint16_t  rxID;
    uint16_t  txID;
    uint16_t  cmdSize;
} ble_header_t;

typedef struct {
    void (*data_received)(const void* src, size_t size);
    void (*notifications_subscribed)(void);
    void (*notifications_unsubscribed)(void);
} ble_server_callbacks;

void        ble_server_init(void);
void        ble_server_deinit(void);
void        ble_server_start(ble_server_callbacks callbacks);
void        ble_server_stop(void);
void        ble_send(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size);
bool16      ble_connected(void);
uint16_t    ble_queue_spaces(void);
uint16_t    ble_queue_waiting(void);
void        ble_set_delay_send(uint16_t delay);
void        ble_set_delay_multi(uint16_t delay);
uint16_t    ble_get_delay_send(void);
uint16_t    ble_get_delay_multi(void);
bool16      ble_set_gap_name(char* name, bool16 set);
bool16      ble_get_gap_name(char* name);
bool16      ble_allow_connection(void);
void        ble_stop_advertising(void);
void        ble_start_advertising(void);

#endif // BLE_SERVER_H