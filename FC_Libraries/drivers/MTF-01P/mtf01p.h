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

    // Dữ liệu raw đọc trực tiếp từ cảm biến
    volatile uint8_t  last_msg_id;     /* 132: DISTANCE_SENSOR, 100: OPTICAL_FLOW, 0: HEARTBEAT */
    volatile uint16_t raw_distance_cm; /* Khoảng cách ToF tới mặt đất (cm) */
    volatile int16_t  raw_flow_x;      /* Vận tốc dòng quang trục X (raw) */
    volatile int16_t  raw_flow_y;      /* Vận tốc dòng quang trục Y (raw) */
    volatile uint8_t  raw_quality;     /* Chất lượng ảnh quang học (0-255) */
    volatile uint32_t last_update_ms;  /* Thời điểm nhận gói tin gần nhất */
    volatile uint32_t frame_count;     /* Tổng số frame hợp lệ nhận được */
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
