#include "protocol/mavlink/mavlink.h"

HAL_StatusTypeDef mavlink_init(mavlink_t *dev, UART_HandleTypeDef *huart)
{
	*dev = (mavlink_t){ .huart = huart, .packet_len = 12 };
	return HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}
volatile float    rc_mavlink_hz = 0.0f;
volatile uint32_t count_rc_mavlink = 0;

void mavlink_uart_rx_callback(mavlink_t *dev)
{

	if (dev->frame_idx == 0) {
		if (dev->rx_byte == 0xFD) { // MAVLink v2 STX
			dev->frame[dev->frame_idx++] = dev->rx_byte;
			dev->packet_len = 12;
		}
	} else {
		dev->frame[dev->frame_idx++] = dev->rx_byte;
		if (dev->frame_idx == 2) dev->packet_len = 12 + dev->rx_byte;            // Khoa do dai: 10 hdr + LEN + 2 crc
		if (dev->frame_idx == 3 && (dev->rx_byte & 0x01)) dev->packet_len += 13; // Co chu ky

		if (dev->frame_idx >= dev->packet_len || dev->frame_idx >= sizeof(dev->frame)) {
			if (dev->frame[7] == 70) { // RC_CHANNELS_OVERRIDE (TX16S)
				count_rc_mavlink++;
				dev->last_packet_time = HAL_GetTick();
				for (int i = 0; i < 8; i++) {
					dev->rc_channel[i]     = (uint16_t)(dev->frame[10 + i * 2] | (dev->frame[11 + i * 2] << 8));
					dev->rc_channel[8 + i] = (uint16_t)(dev->frame[28 + i * 2] | (dev->frame[29 + i * 2] << 8));
				}
			}
			dev->frame_idx = 0;
		}
	}

	HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

void mavlink_uart_error_callback(mavlink_t *dev)
{
	__HAL_UART_CLEAR_FLAG(dev->huart, UART_CLEAR_OREF | UART_CLEAR_FEF);
	dev->frame_idx = 0;
	HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

uint8_t mavlink_is_connected(mavlink_t *dev, uint32_t timeout_ms)
{
	return (dev->last_packet_time && (HAL_GetTick() - dev->last_packet_time <= timeout_ms)) ? 1 : 0;
}
