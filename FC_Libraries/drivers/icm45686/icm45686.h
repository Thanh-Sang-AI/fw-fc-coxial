#ifndef DRIVERS_ICM45686_ICM45686_H_
#define DRIVERS_ICM45686_ICM45686_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ICM45686_WHO_AM_I_VALUE   0xE9

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef      *cs_port;
	uint16_t           cs_pin;

	uint8_t  who_am_i;
	int16_t  accel_raw[3];
	int16_t  gyro_raw[3];
	int16_t  temp_raw;

	float    accel_g[3];    // g
	float    gyro_dps[3];   // deg/s
	float    temp_c;        // degC
} icm45686_t;

uint8_t           icm45686_read_reg(icm45686_t *dev, uint8_t reg);
HAL_StatusTypeDef icm45686_write_reg(icm45686_t *dev, uint8_t reg, uint8_t value);
HAL_StatusTypeDef icm45686_init(icm45686_t *dev);
HAL_StatusTypeDef icm45686_read(icm45686_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_ICM45686_ICM45686_H_ */
