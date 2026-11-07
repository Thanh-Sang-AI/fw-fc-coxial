#include "bmp388.h"
#include "cmsis_os.h"
#include <string.h>
#include <math.h>

/* =========================================================================
   Thanh ghi phần cứng BMP388 & Chip ID (Cố định theo datasheet Bosch)
   ========================================================================= */
#define BMP388_CHIP_ID_BMP388       0x50
#define BMP388_CHIP_ID_BMP390       0x60

#define BMP388_REG_CHIP_ID          0x00
#define BMP388_REG_ERR              0x02
#define BMP388_REG_STATUS           0x03
#define BMP388_REG_DATA_0           0x04  /* Press XLSB */
#define BMP388_REG_PWR_CTRL         0x1B
#define BMP388_REG_OSR              0x1C
#define BMP388_REG_ODR              0x1D
#define BMP388_REG_CONFIG           0x1F
#define BMP388_REG_CALIB_DATA       0x31  /* 21 bytes tu 0x31 den 0x45 */
#define BMP388_REG_CMD              0x7E


typedef struct {
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
} bmp388_quantized_calib_t;

static bmp388_quantized_calib_t s_cal;

/* Giải mã 21 bytes Calibration từ NVM */
static void bmp388ParseCalibration(const uint8_t *d)
{
    uint16_t raw_t1 = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    uint16_t raw_t2 = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
    int8_t   raw_t3 = (int8_t)d[4];

    int16_t  raw_p1 = (int16_t)((uint16_t)d[5] | ((uint16_t)d[6] << 8));
    int16_t  raw_p2 = (int16_t)((uint16_t)d[7] | ((uint16_t)d[8] << 8));
    int8_t   raw_p3 = (int8_t)d[9];
    int8_t   raw_p4 = (int8_t)d[10];
    uint16_t raw_p5 = (uint16_t)d[11] | ((uint16_t)d[12] << 8);
    uint16_t raw_p6 = (uint16_t)d[13] | ((uint16_t)d[14] << 8);
    int8_t   raw_p7 = (int8_t)d[15];
    int8_t   raw_p8 = (int8_t)d[16];
    int16_t  raw_p9 = (int16_t)((uint16_t)d[17] | ((uint16_t)d[18] << 8));
    int8_t   raw_p10 = (int8_t)d[19];
    int8_t   raw_p11 = (int8_t)d[20];

    s_cal.par_t1 = (double)raw_t1 * 256.0;
    s_cal.par_t2 = (double)raw_t2 / 1073741824.0;
    s_cal.par_t3 = (double)raw_t3 / 281474976710656.0;

    s_cal.par_p1 = ((double)raw_p1 - 16384.0) / 1048576.0;
    s_cal.par_p2 = ((double)raw_p2 - 16384.0) / 536870912.0;
    s_cal.par_p3 = (double)raw_p3 / 4294967296.0;
    s_cal.par_p4 = (double)raw_p4 / 137438953472.0;
    s_cal.par_p5 = (double)raw_p5 * 8.0;
    s_cal.par_p6 = (double)raw_p6 / 64.0;
    s_cal.par_p7 = (double)raw_p7 / 256.0;
    s_cal.par_p8 = (double)raw_p8 / 32768.0;
    s_cal.par_p9 = (double)raw_p9 / 281474976710656.0;
    s_cal.par_p10 = (double)raw_p10 / 281474976710656.0;
    s_cal.par_p11 = (double)raw_p11 / 36893488147419103232.0;
    s_cal.t_lin = 0.0;
}

static float bmp388CompensateTemperature(uint32_t uncomp_temp)
{
    double p1 = (double)uncomp_temp - s_cal.par_t1;
    double p2 = p1 * s_cal.par_t2;
    s_cal.t_lin = p2 + (p1 * p1) * s_cal.par_t3;
    return (float)s_cal.t_lin;
}

static float bmp388CompensatePressure(uint32_t uncomp_press)
{
    double t = s_cal.t_lin;
    double t2 = t * t;
    double t3 = t2 * t;

    double p1 = s_cal.par_p6 * t;
	double p2 = s_cal.par_p7 * t2;
	double p3 = s_cal.par_p8 * t3;
	double out1 = s_cal.par_p5 + p1 + p2 + p3;

	p1 = s_cal.par_p2 * t;
	p2 = s_cal.par_p3 * t2;
	p3 = s_cal.par_p4 * t3;
	double out2 = (double)uncomp_press * (s_cal.par_p1 + p1 + p2 + p3);

	p1 = (double)uncomp_press * (double)uncomp_press;
	p2 = s_cal.par_p9 + s_cal.par_p10 * t;
	p3 = p1 * p2;
	double p4 = p3 + (p1 * (double)uncomp_press) * s_cal.par_p11;

	double comp_press = out1 + out2 + p4;
	return (float)comp_press; /* Pascals (Pa) */
}

/* =========================================================================
   API Driver BMP388: Normal Mode 50Hz, Hardware IIR Coeff 3, OSR Press 8x
   ========================================================================= */
bool BMP388_Init(BMP388_t *dev, I2C_HandleTypeDef *hi2c)
{
	if (dev == NULL || hi2c == NULL) return false;
	dev->hi2c = hi2c;
	dev->ground_altitude_m = 0.0f;
	dev->altitude_m = 0.0f;
	dev->pressure_pa = 101325.0f;
	dev->temperature_c = 25.0f;

	/* 1. Dò địa chỉ I2C: 0x76 trước rồi 0x77 */
	uint8_t addrs[2] = { (BMP388_I2C_ADDR_PRIM << 1), (BMP388_I2C_ADDR_SEC << 1) };
	bool found = false;
	uint8_t id = 0;

	for (int i = 0; i < 2; i++) {
		if (HAL_I2C_IsDeviceReady(dev->hi2c, addrs[i], 2, 50) == HAL_OK) {
			if (HAL_I2C_Mem_Read(dev->hi2c, addrs[i], BMP388_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &id, 1, 50) == HAL_OK) {
				if (id == BMP388_CHIP_ID_BMP388 || id == BMP388_CHIP_ID_BMP390) {
					dev->i2c_addr = addrs[i];
					dev->chip_id = id;
					found = true;
					break;
				}
			}
		}
	}

	if (!found) return false;

	/* 2. Soft reset đưa chip về trạng thái sạch */
	uint8_t cmd_reset = 0xB6;
	HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, BMP388_REG_CMD, I2C_MEMADD_SIZE_8BIT, &cmd_reset, 1, 50);
	HAL_Delay(10);

	/* 3. Đọc 21 bytes hệ số hiệu chuẩn NVM */
	uint8_t calib_buf[21];
	if (HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, BMP388_REG_CALIB_DATA,
						 I2C_MEMADD_SIZE_8BIT, calib_buf, 21, 100) != HAL_OK) {
		return false;
	}
	bmp388ParseCalibration(calib_buf);

	/* 4. Cấu hình Oversampling: Pressure 8x (0x03), Temp 1x (0x00) -> OSR = 0x03 */
	uint8_t osr = 0x03;
	HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, BMP388_REG_OSR, I2C_MEMADD_SIZE_8BIT, &osr, 1, 50);

	/* 5. Cấu hình ODR: 50 Hz (chu kỳ 20ms) -> BMP388_ODR_50HZ = 0x02 */
	uint8_t odr = 0x02;
	HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, BMP388_REG_ODR, I2C_MEMADD_SIZE_8BIT, &odr, 1, 50);

	/* 6. Bật bộ lọc IIR phần cứng (Coeff 3 để triệt tiêu gió cánh quạt)
		  Thanh ghi CONFIG (0x1F): bit [3:1] = 0b010 (Filter coefficient 3) -> 0x04 */
	uint8_t cfg = 0x08;
	HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, BMP388_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &cfg, 1, 50);

	/* 7. Kích hoạt Normal Mode (tự động lấy mẫu liên tục ở 50Hz, không cần trigger thủ công)
		  PWR_CTRL (0x1B): press_en=1 (bit 0), temp_en=1 (bit 1), mode=11 (Normal Mode, bit 5:4) -> 0x33 */
	uint8_t pwr = 0x33;
	HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, BMP388_REG_PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &pwr, 1, 50);

	HAL_Delay(25); // Chờ mẫu đầu tiên hoàn tất chuyển đổi
	return true;
}

bool BMP388_ReadData(BMP388_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL) return false;

	/* Đọc đồng thời 6 byte dữ liệu thô (Pressure 3 bytes, Temp 3 bytes) từ shadow registers */
	uint8_t buf[6];
	if (HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, BMP388_REG_DATA_0, I2C_MEMADD_SIZE_8BIT, buf, 6, 20) != HAL_OK) {
		return false;
	}

	uint32_t raw_press = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
	uint32_t raw_temp  = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);

	// Kiểm tra tràn dữ liệu cảm biến
	if (raw_press == 0 || raw_press == 0x800000 || raw_temp == 0 || raw_temp == 0x800000) {
		return false;
	}

	// Bù nhiệt độ và áp suất
	dev->temperature_c = bmp388CompensateTemperature(raw_temp);
	dev->pressure_pa = bmp388CompensatePressure(raw_press);

	if (dev->pressure_pa < 30000.0f || dev->pressure_pa > 125000.0f) {
		return false;
	}

	// Quy đổi độ cao tuyệt đối (Hypsometric Formula)
	float p_ratio = dev->pressure_pa / 101325.0f;
	float abs_alt = 44330.0f * (1.0f - powf(p_ratio, 0.19029495f));
	dev->altitude_m = abs_alt - dev->ground_altitude_m;

	return true;
}

void BMP388_SetGroundAltitude(BMP388_t *dev, float ground_alt_m)
{
	if (dev == NULL) return;
	dev->ground_altitude_m = ground_alt_m;
}

bool BMP388_CalibrateGround(BMP388_t *dev, uint16_t samples)
{
	if (dev == NULL || samples == 0) return false;

	dev->ground_altitude_m = 0.0f;
	float sum_alt = 0.0f;
	uint16_t valid_cnt = 0;

	for (uint16_t i = 0; i < samples; i++) {
		if (BMP388_ReadData(dev)) {
			sum_alt += dev->altitude_m;
			valid_cnt++;
		}
		vTaskDelay(20);
	}

	if (valid_cnt > 0) {
		dev->ground_altitude_m = sum_alt / (float)valid_cnt;
		dev->altitude_m = 0.0f;
		return true;
	}

	return false;
}
