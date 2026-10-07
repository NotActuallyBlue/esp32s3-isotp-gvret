#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

// The serial-cable transport for Simos Tools (a wired UART instead of Bluetooth). Nothing is connected to those pins on this
// board, and the transport costs about 20 KB of memory, so it is left out. Define SIMOS_UART_TRANSPORT to build it back in.
// #define SIMOS_UART_TRANSPORT

void uart_data_received(const void* src, size_t size);     // implemented by the bridge

#ifdef SIMOS_UART_TRANSPORT
void uart_init();
void uart_deinit();
void uart_start_task();
void uart_stop_task();
void uart_send(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size);
void uart_buffer_clear();
#else
static inline void uart_init(void) {}
static inline void uart_deinit(void) {}
static inline void uart_start_task(void) {}
static inline void uart_stop_task(void) {}
static inline void uart_send(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size) { (void)txID; (void)rxID; (void)flags; (void)src; (void)size; }
static inline void uart_buffer_clear(void) {}
#endif

#endif
