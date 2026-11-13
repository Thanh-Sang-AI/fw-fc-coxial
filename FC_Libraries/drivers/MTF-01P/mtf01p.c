#include "drivers/MTF-01P/mtf01p.h"
#include <string.h>

HAL_StatusTypeDef mtf01p_init(mtf01p_t *dev, UART_HandleTypeDef *huart)
{
    if (dev == NULL || huart == NULL) return HAL_ERROR;
    memset((void *)dev, 0, sizeof(mtf01p_t));
    dev->huart = huart;
    return HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

// Ngat UART: Chi day byte vao ring buffer trung gian (sieu nhe)
void mtf01p_uart_rx_callback(mtf01p_t *dev)
{
    if (dev == NULL) return;

    uint16_t next_head = (dev->rb_head + 1) & (MTF01P_RING_BUF_SIZE - 1);
    if (next_head != dev->rb_tail) {
        dev->rb_buf[dev->rb_head] = dev->rx_byte;
        dev->rb_head = next_head;
    }

    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

void mtf01p_uart_error_callback(mtf01p_t *dev)
{
    if (dev == NULL || dev->huart == NULL) return;
    __HAL_UART_CLEAR_FLAG(dev->huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

// Ham doc va giai ma du lieu tu buffer trung gian (chay trong Task)
bool mtf01p_update(mtf01p_t *dev)
{
    if (dev == NULL) return false;
    bool new_frame = false;

    while (dev->rb_tail != dev->rb_head) {
        uint8_t b = dev->rb_buf[dev->rb_tail];
        dev->rb_tail = (dev->rb_tail + 1) & (MTF01P_RING_BUF_SIZE - 1);

        if (dev->parse_idx == 0) {
            if (b == 0xFE) { // MAVLink v1 STX
                dev->parse_buf[dev->parse_idx++] = b;
            }
        } else {
            dev->parse_buf[dev->parse_idx++] = b;
            if (dev->parse_idx == 2) {
                dev->payload_len = b;
            }

            // Frame MAVLink v1: 6 header + payload_len + 2 crc = payload_len + 8
            if (dev->parse_idx >= (dev->payload_len + 8) || dev->parse_idx >= sizeof(dev->parse_buf)) {
                dev->last_msg_id = dev->parse_buf[5];

                if (dev->last_msg_id == 132 && dev->payload_len >= 10) { // DISTANCE_SENSOR
                    dev->raw_distance_cm = (uint16_t)(dev->parse_buf[14] | (dev->parse_buf[15] << 8));
                    dev->last_update_ms = HAL_GetTick();
                    dev->frame_count++;
                    new_frame = true;
                } else if (dev->last_msg_id == 106 && dev->payload_len >= 44) { // OPTICAL_FLOW_RAD
                    uint32_t dt_us = (uint32_t)dev->parse_buf[14] | ((uint32_t)dev->parse_buf[15] << 8) |
                                     ((uint32_t)dev->parse_buf[16] << 16) | ((uint32_t)dev->parse_buf[17] << 24);
                    float int_x = 0.0f, int_y = 0.0f;
                    memcpy(&int_x, &dev->parse_buf[18], sizeof(float));
                    memcpy(&int_y, &dev->parse_buf[22], sizeof(float));
                    dev->raw_quality = dev->parse_buf[49];

                    float dt_s = (dt_us > 0 && dt_us < 1000000) ? ((float)dt_us * 1e-6f) : 0.01f;
                    dev->raw_flow_x = int_x / dt_s;
                    dev->raw_flow_y = int_y / dt_s;

                    float dist_m = 0.0f;
                    memcpy(&dist_m, &dev->parse_buf[42], sizeof(float));
                    if (dist_m > 0.01f && dev->raw_distance_cm == 0) {
                        dev->raw_distance_cm = (uint16_t)(dist_m * 100.0f);
                    }

                    dev->last_update_ms = HAL_GetTick();
                    dev->frame_count++;
                    new_frame = true;
                } else if (dev->last_msg_id == 100 && dev->payload_len >= 26) { // OPTICAL_FLOW
                    int16_t fx = (int16_t)(dev->parse_buf[26] | (dev->parse_buf[27] << 8));
                    int16_t fy = (int16_t)(dev->parse_buf[28] | (dev->parse_buf[29] << 8));
                    dev->raw_flow_x   = (float)fx * 0.01f;
                    dev->raw_flow_y   = (float)fy * 0.01f;
                    dev->raw_quality  = dev->parse_buf[31];
                    dev->last_update_ms = HAL_GetTick();
                    dev->frame_count++;
                    new_frame = true;
                }

                dev->parse_idx = 0;
            }
        }
    }
    dev->has_new_data = new_frame;
    return new_frame;
}
