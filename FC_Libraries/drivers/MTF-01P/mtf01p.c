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
            } else if (dev->last_msg_id == 106 && dev->payload_len >= 44) { // OPTICAL_FLOW_RAD
                uint32_t dt_us = (uint32_t)dev->buf[14] | ((uint32_t)dev->buf[15] << 8) |
                                 ((uint32_t)dev->buf[16] << 16) | ((uint32_t)dev->buf[17] << 24);
                float int_x = 0.0f, int_y = 0.0f;
                memcpy(&int_x, &dev->buf[18], sizeof(float));
                memcpy(&int_y, &dev->buf[22], sizeof(float));
                dev->raw_quality = dev->buf[49];

                float dt_s = (dt_us > 0 && dt_us < 1000000) ? ((float)dt_us * 1e-6f) : 0.01f;
                dev->raw_flow_x = int_x / dt_s;
                dev->raw_flow_y = int_y / dt_s;

                float dist_m = 0.0f;
                memcpy(&dist_m, &dev->buf[42], sizeof(float));
                if (dist_m > 0.01f && dev->raw_distance_cm == 0) {
                    dev->raw_distance_cm = (uint16_t)(dist_m * 100.0f);
                }

                dev->last_update_ms = HAL_GetTick();
                dev->frame_count++;
            } else if (dev->last_msg_id == 100 && dev->payload_len >= 26) { // OPTICAL_FLOW
                int16_t fx = (int16_t)(dev->buf[26] | (dev->buf[27] << 8));
                int16_t fy = (int16_t)(dev->buf[28] | (dev->buf[29] << 8));
                dev->raw_flow_x   = (float)fx * 0.01f;
                dev->raw_flow_y   = (float)fy * 0.01f;
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
