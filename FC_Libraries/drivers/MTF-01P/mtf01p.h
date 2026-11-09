#ifndef DRIVERS_MTF01P_H_
#define DRIVERS_MTF01P_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
   Cấu trúc dữ liệu cảm biến MTF-01P (Optical Flow + ToF LiDAR)
   Giao tiếp qua MAVLink v1 (115200 baud)
   ========================================================================= */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t   rx_byte;
    uint8_t   buf[64];
    uint8_t   idx;
    uint8_t   payload_len;

    // Du lieu raw doc truc tiep tu cam bien
    volatile uint8_t  last_msg_id;     /* 132: DISTANCE_SENSOR, 106: OPTICAL_FLOW_RAD, 100: OPTICAL_FLOW */
    volatile uint16_t raw_distance_cm; /* Khoang cach ToF toi mat dat (cm) */
    volatile float    raw_flow_x;      /* Toc do dong quang truc X (rad/s) */
    volatile float    raw_flow_y;      /* Toc do dong quang truc Y (rad/s) */
    volatile uint8_t  raw_quality;     /* Chat luong anh quang hoc (0-255) */
    volatile uint32_t last_update_ms;  /* Thoi diem nhan goi tin gan nhat */
    volatile uint32_t frame_count;     /* Tong so frame hop le nhan duoc */
} mtf01p_t;

/* =========================================================================
   API Driver MTF-01P
   ========================================================================= */
HAL_StatusTypeDef mtf01p_init(mtf01p_t *dev, UART_HandleTypeDef *huart);
void              mtf01p_uart_rx_callback(mtf01p_t *dev);
void              mtf01p_uart_error_callback(mtf01p_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_MTF01P_H_ */
