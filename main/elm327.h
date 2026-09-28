#ifndef ELM327_H
#define ELM327_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

void elm327_init(void);
void elm327_start(void);
void elm327_rx_byte(uint8_t byte);
void elm327_rx_data(const void *data, size_t len);

#endif // ELM327_H