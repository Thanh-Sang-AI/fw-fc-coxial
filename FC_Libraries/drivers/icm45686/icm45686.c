#include <string.h>
#include "icm45686.h"
#include "cmsis_os.h"

/* Bank 0 registers */
#define REG_ACCEL_DATA_X1_UI     0x00   // 0x00..0x0D: accel XYZ, gyro XYZ, temp
#define REG_PWR_MGMT0            0x10
#define REG_INT1_STATUS0         0x19
#define REG_ACCEL_CONFIG0        0x1B
#define REG_GYRO_CONFIG0         0x1C
#define REG_WHO_AM_I             0x72
#define REG_MISC2                0x7F

#define SPI_READ_BIT             0x80
#define MISC2_SOFT_RST           0x02
#define PWR_MGMT0_ACC_GYR_LN     0x0F   // GYRO_MODE = 11, ACCEL_MODE = 11 (low noise)

/* bit [6:4] full scale, bit [3:0] ODR. 16 g = 1, 2000 dps = 1, 800 Hz = 6 */
#define ACCEL_CONFIG0_16G_800HZ     ((1u << 4) | 6u)
#define GYRO_CONFIG0_2000DPS_800HZ  ((1u << 4) | 6u)

#define ACCEL_LSB_PER_G          2048.0f
#define GYRO_LSB_PER_DPS         16.4f
#define TEMP_LSB_PER_C           128.0f
#define TEMP_OFFSET_C            25.0f

#define SPI_TIMEOUT_MS           10

static void cs_select(icm45686_t *dev)   { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static void cs_deselect(icm45686_t *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

/* Read: first byte = address | 0x80, the chip answers on the following bytes */
static HAL_StatusTypeDef read_regs(icm45686_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	uint8_t tx[16] = { 0 };
	uint8_t rx[16] = { 0 };
	HAL_StatusTypeDef st;

	if (len > sizeof(tx) - 1) return HAL_ERROR;

	tx[0] = reg | SPI_READ_BIT;
	cs_select(dev);
	st = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len + 1, SPI_TIMEOUT_MS);
	cs_deselect(dev);

	memcpy(buf, &rx[1], len);
	return st;
}

uint8_t icm45686_read_reg(icm45686_t *dev, uint8_t reg)
{
	uint8_t value = 0;
	read_regs(dev, reg, &value, 1);
	return value;
}

HAL_StatusTypeDef icm45686_write_reg(icm45686_t *dev, uint8_t reg, uint8_t value)
{
	uint8_t tx[2] = { reg & 0x7F, value };
	HAL_StatusTypeDef st;

	cs_select(dev);
	st = HAL_SPI_Transmit(dev->hspi, tx, 2, SPI_TIMEOUT_MS);
	cs_deselect(dev);
	return st;
}

HAL_StatusTypeDef icm45686_init(icm45686_t *dev)
{
	cs_deselect(dev);

	/* 1. Is the chip there? */
	dev->who_am_i = icm45686_read_reg(dev, REG_WHO_AM_I);
	if (dev->who_am_i != ICM45686_WHO_AM_I_VALUE) return HAL_ERROR;

	/* 2. Soft reset, wait >= 1 ms, then read INT1_STATUS0 to clear RESET_DONE */
	icm45686_write_reg(dev, REG_MISC2, MISC2_SOFT_RST);
	HAL_Delay(2);
	if (icm45686_read_reg(dev, REG_WHO_AM_I) != ICM45686_WHO_AM_I_VALUE) return HAL_ERROR;
	(void)icm45686_read_reg(dev, REG_INT1_STATUS0);

	/* 3. Range + rate, then turn accel and gyro on */
	icm45686_write_reg(dev, REG_ACCEL_CONFIG0, ACCEL_CONFIG0_16G_800HZ);
	icm45686_write_reg(dev, REG_GYRO_CONFIG0, GYRO_CONFIG0_2000DPS_800HZ);
	icm45686_write_reg(dev, REG_PWR_MGMT0, PWR_MGMT0_ACC_GYR_LN);
	HAL_Delay(30);  // gyro start-up time

	/* 4. Read back to make sure the writes landed */
	if (icm45686_read_reg(dev, REG_PWR_MGMT0) != PWR_MGMT0_ACC_GYR_LN) return HAL_ERROR;
	if (icm45686_read_reg(dev, REG_ACCEL_CONFIG0) != ACCEL_CONFIG0_16G_800HZ) return HAL_ERROR;

	return HAL_OK;
}

HAL_StatusTypeDef icm45686_read(icm45686_t *dev)
{
	uint8_t b[14];
	HAL_StatusTypeDef st = read_regs(dev, REG_ACCEL_DATA_X1_UI, b, sizeof(b));
	if (st != HAL_OK) return st;

	/* Chip output is Big-Endian: high byte (X1) at lower address, low byte (X0) at higher address */
	for (int i = 0; i < 3; i++) {
		dev->accel_raw[i] = (int16_t)(((int16_t)b[2 * i] << 8) | b[2 * i + 1]);
		dev->gyro_raw[i]  = (int16_t)(((int16_t)b[2 * i + 6] << 8) | b[2 * i + 7]);
		dev->accel_g[i]   = dev->accel_raw[i] / ACCEL_LSB_PER_G;
		dev->gyro_dps[i]  = dev->gyro_raw[i] / GYRO_LSB_PER_DPS;
	}
	dev->temp_raw = (int16_t)(((int16_t)b[12] << 8) | b[13]);
	dev->temp_c   = dev->temp_raw / TEMP_LSB_PER_C + TEMP_OFFSET_C;

	return HAL_OK;
}

HAL_StatusTypeDef icm45686_calibrate_gyro(icm45686_t *dev, float bias[3])
{
	if (dev == NULL || dev->hspi == NULL || bias == NULL) return HAL_ERROR;

	float sum[3] = {0.0f, 0.0f, 0.0f};
	const int samples = 500;

	for (int i = 0; i < samples; i++) {
		if (icm45686_read(dev) != HAL_OK) return HAL_ERROR;
		sum[0] +=  dev->gyro_dps[0];
		sum[1] +=  dev->gyro_dps[1];
		sum[2] += -dev->gyro_dps[2]; // Dao chieu truc Z do chip han o mat duoi PCB
		vTaskDelay(2);
	}

	bias[0] = sum[0] / (float)samples;
	bias[1] = sum[1] / (float)samples;
	bias[2] = sum[2] / (float)samples;
	return HAL_OK;
}
