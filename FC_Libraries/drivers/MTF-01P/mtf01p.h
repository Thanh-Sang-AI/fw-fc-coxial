#ifndef DRIVERS_MTF01P_H_
#define DRIVERS_MTF01P_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MTF01P_RING_BUF_SIZE 256

/* =========================================================================
   Cấu trúc dữ liệu cảm biến MTF-01P (Optical Flow + ToF LiDAR)
   Giao tiếp qua MAVLink v1 (115200 baud)
   ========================================================================= */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t   rx_byte;

    // Ring buffer trung gian cho ngat UART (Lock-free SPSC)
    volatile uint8_t  rb_buf[MTF01P_RING_BUF_SIZE];
    volatile uint16_t rb_head; // Ghi vao (trong ngat)
    volatile uint16_t rb_tail; // Doc ra (trong task)

    // Buffer parse frame noi bo trong task
    uint8_t   parse_buf[64];
    uint8_t   parse_idx;
    uint8_t   payload_len;

    // Du lieu raw doc truc tiep tu cam bien
    volatile uint8_t  last_msg_id;     /* 132: DISTANCE_SENSOR, 106: OPTICAL_FLOW_RAD, 100: OPTICAL_FLOW */
    volatile uint16_t raw_distance_cm; /* Khoang cach ToF toi mat dat (cm) */
    volatile float    raw_flow_x;      /* Toc do dong quang truc X (rad/s) */
    volatile float    raw_flow_y;      /* Toc do dong quang truc Y (rad/s) */
    volatile uint8_t  raw_quality;     /* Chat luong anh quang hoc (0-255) */
    volatile uint32_t last_update_ms;  /* Thoi diem nhan goi tin gan nhat */
    volatile uint32_t frame_count;     /* Tong so frame hop le nhan duoc */
    volatile bool     has_new_data;    /* Co bao vua co frame moi duoc giai ma */
} mtf01p_t;

/* =========================================================================
   API Driver MTF-01P
   ========================================================================= */
HAL_StatusTypeDef mtf01p_init(mtf01p_t *dev, UART_HandleTypeDef *huart);
void              mtf01p_uart_rx_callback(mtf01p_t *dev);
void              mtf01p_uart_error_callback(mtf01p_t *dev);
bool              mtf01p_update(mtf01p_t *dev); // Goi trong task de doc va giai ma data

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_MTF01P_H_ */
