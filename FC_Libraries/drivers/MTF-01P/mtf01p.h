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
    volatile uint8_t  last_msg_id;     /* 132: DISTANCE_SENSOR, 100: OPTICAL_FLOW, 0: HEARTBEAT */
    volatile uint16_t raw_distance_cm; /* Khoang cach ToF toi mat dat (cm) */
    volatile int16_t  raw_flow_x;      /* Van toc dong quang truc X (raw) */
    volatile int16_t  raw_flow_y;      /* Van toc dong quang truc Y (raw) */
    volatile uint8_t  raw_quality;     /* Chat luong anh quang hoc (0-255) */
    volatile uint32_t last_update_ms;  /* Thoi diem nhan goi tin gan nhat */
    volatile uint32_t frame_count;     /* Tong so frame hop le nhan duoc */

    // Du lieu uoc luong trang thai sau loc va Kalman (100 Hz)
    volatile float    z;               /* Do cao Z uoc luong (m) */
    volatile float    vz;              /* Van toc leo Vz uoc luong (m/s) */
    volatile float    x;               /* Vi tri ngang X (m) */
    volatile float    y;               /* Vi tri ngang Y (m) */
    volatile float    vx;              /* Van toc ngang Vx (m/s) */
    volatile float    vy;              /* Van toc ngang Vy (m/s) */
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
