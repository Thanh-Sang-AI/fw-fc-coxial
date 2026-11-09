#include "drivers/MTF-01P/mtf01p.h"
#include <string.h>

HAL_StatusTypeDef mtf01p_init(mtf01p_t *dev, UART_HandleTypeDef *huart)
{
    if (dev == NULL || huart == NULL) return HAL_ERROR;
    memset((void *)dev, 0, sizeof(mtf01p_t));
    dev->huart = huart;
    return HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

void mtf01p_uart_rx_callback(mtf01p_t *dev)
{
    if (dev == NULL) return;

    if (dev->idx == 0) {
        if (dev->rx_byte == 0xFE) { // MAVLink v1 STX
            dev->buf[dev->idx++] = dev->rx_byte;
        }
    } else {
        dev->buf[dev->idx++] = dev->rx_byte;
        if (dev->idx == 2) {
            dev->payload_len = dev->rx_byte;
        }

        // Frame MAVLink v1: 6 header + payload_len + 2 crc = payload_len + 8
        if (dev->idx >= (dev->payload_len + 8) || dev->idx >= sizeof(dev->buf)) {
            dev->last_msg_id = dev->buf[5]; // Message ID o byte 5

            if (dev->last_msg_id == 132 && dev->payload_len >= 10) { // DISTANCE_SENSOR
                dev->raw_distance_cm = (uint16_t)(dev->buf[14] | (dev->buf[15] << 8));
                dev->last_update_ms = HAL_GetTick();
                dev->frame_count++;
            } else if (dev->last_msg_id == 100 && dev->payload_len >= 26) { // OPTICAL_FLOW
                dev->raw_flow_x   = (int16_t)(dev->buf[26] | (dev->buf[27] << 8));
                dev->raw_flow_y   = (int16_t)(dev->buf[28] | (dev->buf[29] << 8));
                dev->raw_quality  = dev->buf[31];
                dev->last_update_ms = HAL_GetTick();
                dev->frame_count++;
            } else {
                dev->last_update_ms = HAL_GetTick();
                dev->frame_count++;
            }

            dev->idx = 0;
        }
    }

    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

void mtf01p_uart_error_callback(mtf01p_t *dev)
{
    if (dev == NULL || dev->huart == NULL) return;
    __HAL_UART_CLEAR_FLAG(dev->huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    dev->idx = 0;
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}
