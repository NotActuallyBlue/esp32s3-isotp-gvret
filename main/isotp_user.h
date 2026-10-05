#ifndef __ISOTP_USER_H__
#define __ISOTP_USER_H__

/* user implemented, print debug message */
void isotp_user_debug(const char* message, ...);

/* user implemented, send can message */
int  isotp_user_send_can(const uint32_t arbitration_id,
                         const uint8_t* data, const uint16_t size);

/* user implemented, get microsecond */
uint64_t isotp_user_get_us(void);

/* user implemented, called for every flow control frame accepted while sending (for diagnostics) */
void isotp_user_flow_control(uint32_t arbitration_id, uint8_t block_size,
                             uint32_t receiver_st_min_us, uint16_t override_us, uint32_t used_st_min_us);

#endif // __ISOTP_H__

