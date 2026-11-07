#ifndef PROTOCOL_MAVLINK_MAVLINK_H_
#define PROTOCOL_MAVLINK_MAVLINK_H_

#include "main.h"

typedef struct {
	UART_HandleTypeDef *huart;
	uint8_t   rx_byte;
	uint8_t   frame[280];
	uint16_t  frame_idx;
	uint16_t  packet_len;
	volatile uint16_t rc_channel[16];
	volatile uint32_t last_packet_time;
} mavlink_t;

HAL_StatusTypeDef mavlink_init(mavlink_t *dev, UART_HandleTypeDef *huart);
void              mavlink_uart_rx_callback(mavlink_t *dev);
void              mavlink_uart_error_callback(mavlink_t *dev);
uint8_t           mavlink_is_connected(mavlink_t *dev, uint32_t timeout_ms);

extern volatile float    rc_mavlink_hz;
extern volatile uint32_t count_rc_mavlink;

#endif /* PROTOCOL_MAVLINK_MAVLINK_H_ */
