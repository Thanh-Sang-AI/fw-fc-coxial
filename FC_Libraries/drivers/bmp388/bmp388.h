#ifndef BMP388_H
#define BMP388_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
   Cấu hình I2C (Người dùng chỉnh khi đổi jumper SDO phần cứng)
   ========================================================================= */
#define BMP388_I2C_ADDR_PRIM        0x76  /* SDO noi GND (dia chi 7-bit) */
#define BMP388_I2C_ADDR_SEC         0x77  /* SDO noi VDD (dia chi 7-bit) */

/* =========================================================================
   Cấu trúc dữ liệu cảm biến
   ========================================================================= */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;           /* Dia chi 8-bit (addr << 1) */
    uint8_t chip_id;

    float temperature_c;        /* Nhiet do (degC) */
    float pressure_pa;          /* Ap suat thuc te (Pa) */
    float altitude_m;           /* Do cao tinh tu ap suat (m) */
    float ground_altitude_m;    /* Do cao moc mat dat luc khoi dong */
} BMP388_t;

/* =========================================================================
   API Driver
   ========================================================================= */
/**
 * @brief  Khoi tao BMP388 tren I2C (tu do dia chi 0x76 / 0x77):
 *         - Chế độ: Forced Mode (theo chuan INAV)
 *         - Oversampling: Pressure 8x, Temp 1x (OSR = 0x03)
 *         - IIR filter: 0 / Bypass (CONFIG = 0x00)
 */
bool BMP388_Init(BMP388_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Doc 6 byte du lieu, tinh bu nhiet/ap suat va quy doi ra do cao (m)
 */
bool BMP388_ReadData(BMP388_t *dev);

/**
 * @brief  Dat do cao moc mat dat (0m)
 */
void BMP388_SetGroundAltitude(BMP388_t *dev, float ground_alt_m);

#ifdef __cplusplus
}
#endif

#endif /* BMP388_H */
