#include "Drivers/bmp390.h"

static inline void BMP390_Select(BMP390_Handle_t *handle) {
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);
}

static inline void BMP390_Deselect(BMP390_Handle_t *handle) {
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);
}

static void BMP390_LoadCalibrationData(BMP390_Handle_t *handle, const uint8_t *raw_data);
static float BMP390_CompensateTemperature(BMP390_Handle_t *handle, uint32_t uncomp_temp);
static float BMP390_CompensatePressure(BMP390_Handle_t *handle, uint32_t uncomp_pressure);

int BMP390_Init(BMP390_Handle_t *handle) {
	handle->last_altitude = NAN;
	handle->last_pressure = NAN;
	handle->last_temperature = NAN;

	BMP390_Deselect(handle);
	HAL_Delay(10);

	uint8_t chip_id = BMP390_ReadReg(handle, BMP390_REG_CHIP_ID);
	if (chip_id != BMP390_CHIP_ID) {
		return 0;
	}

	BMP390_WriteReg(handle, BMP390_REG_CMD, BMP390_SOFT_RESET);
	HAL_Delay(50);

    uint8_t pwr_ctrl = (BMP390_ENABLE_PRESSURE | BMP390_ENABLE_TEMP | BMP390_ENABLE_SENSOR);
    BMP390_WriteReg(handle, BMP390_REG_PWR_CTRL, pwr_ctrl);

    uint8_t osr_val = ((BMP390_OSR_x2 << 3) | BMP390_OSR_x32);
    BMP390_WriteReg(handle, BMP390_REG_OSR, osr_val);

    BMP390_WriteReg(handle, BMP390_REG_ODR, BMP390_ODR_12p5_HZ);
    HAL_Delay(5);

    uint8_t raw_calib[21] = {0};
    BMP390_ReadBuffer(handle, BMP390_REG_CAL, raw_calib, 21);
    BMP390_LoadCalibrationData(handle, raw_calib);

    return 1;
}

void BMP390_Step(BMP390_Handle_t *handle) {
	uint8_t raw_data[6] = {0};

	BMP390_ReadBuffer(handle, BMP390_REG_DATA, raw_data, 6);

	uint32_t uncomp_press = ((uint32_t)raw_data[2] << 16) |
							((uint32_t)raw_data[1] << 8) |
							raw_data[0];

	uint32_t uncomp_temp = ((uint32_t)raw_data[5] << 16) |
								((uint32_t)raw_data[4] << 8) |
								raw_data[3];

	float comp_temp = BMP390_CompensateTemperature(handle, uncomp_temp);
	float comp_press = BMP390_CompensatePressure(handle, uncomp_press);

	handle->last_temperature = comp_temp;
	handle->last_pressure = comp_press;
	handle->last_altitude = 44330.0f * (1.0f - powf(comp_press/101325.0f, 0.1903f));
}

float BMP390_GetTemperature(BMP390_Handle_t *handle) {
	return handle->last_temperature;
}

float BMP390_GeAltitude(BMP390_Handle_t *handle) {
	return handle->last_altitude;
}

float BMP390_GetPressure(BMP390_Handle_t *handle) {
	return handle->last_pressure;
}

uint8_t BMP390_ReadReg(BMP390_Handle_t *handle, uint8_t reg) {
	uint8_t tx[2], rx[2];
	tx[0] = (reg | BMP390_SPI_READ_BIT);
	tx[1] = 0x00;

	rx[0] = 0, rx[1] = 0;

	BMP390_Select(handle);

	HAL_SPI_Transmit(handle->hspi, tx, 2, HAL_MAX_DELAY);

	HAL_SPI_Receive(handle->hspi, &rx[1], 1, HAL_MAX_DELAY);

	BMP390_Deselect(handle);

	return rx[1];
}

void BMP390_WriteReg(BMP390_Handle_t *handle, uint8_t reg, uint8_t data) {
    uint8_t tx[2];
    tx[0] = (reg & 0x7F);
    tx[1] = data;

    BMP390_Select(handle);
    HAL_SPI_Transmit(handle->hspi, tx, 2, HAL_MAX_DELAY);
    BMP390_Deselect(handle);
}

void BMP390_ReadBuffer(BMP390_Handle_t *handle, uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t tx[2];
    tx[0] = (reg | BMP390_SPI_READ_BIT);
    tx[1] = 0x00;

    BMP390_Select(handle);

    HAL_SPI_Transmit(handle->hspi, tx, 2, HAL_MAX_DELAY);

    HAL_SPI_Receive(handle->hspi, data, len, HAL_MAX_DELAY);

    BMP390_Deselect(handle);
}

static void BMP390_LoadCalibrationData(BMP390_Handle_t *handle, const uintt_8 *raw_data) {
    BMP390_RawCalibData_t *rc = &handle->raw_calib_data;
    BMP390_CalibData_t    *c  = &handle->calib_data;

    rc->nvm_par_t1 = (uint16_t)(raw_data[1] << 8) | raw_data[0];
    rc->nvm_par_t2 = (uint16_t)(raw_data[3] << 8) | raw_data[2];
    rc->nvm_par_t3 = (int8_t)raw_data[4];
    rc->nvm_par_p1 = (int16_t)(raw_data[6] << 8) | raw_data[5];
    rc->nvm_par_p2 = (int16_t)(raw_data[8] << 8) | raw_data[7];
    rc->nvm_par_p3 = (int8_t) raw_data[9];
    rc->nvm_par_p4 = (int8_t) raw_data[10];
    rc->nvm_par_p5 = (uint16_t)(raw_data[12] << 8) | raw_data[11];
    rc->nvm_par_p6 = (uint16_t)(raw_data[14] << 8) | raw_data[13];
    rc->nvm_par_p7 = (int8_t) raw_data[15];
    rc->nvm_par_p8 = (int8_t) raw_data[16];
    rc->nvm_par_p9 = (int16_t)(raw_data[18] << 8) | raw_data[17];
    rc->nvm_par_p10= (int8_t) raw_data[19];
    rc->nvm_par_p11= (int8_t) raw_data[20];

    c->par_t1 = rc->nvm_par_t1 / powf(2.0f, -8.0f);
    c->par_t2 = rc->nvm_par_t2 / powf(2.0f,  30.0f);
    c->par_t3 = rc->nvm_par_t3 / powf(2.0f,  48.0f);

    c->par_p1 = (rc->nvm_par_p1 - powf(2.0f,14.0f)) / powf(2.0f,20.0f);
    c->par_p2 = (rc->nvm_par_p2 - powf(2.0f,14.0f)) / powf(2.0f,29.0f);
    c->par_p3 = rc->nvm_par_p3 / powf(2.0f, 32.0f);
    c->par_p4 = rc->nvm_par_p4 / powf(2.0f, 37.0f);
    c->par_p5 = rc->nvm_par_p5 / powf(2.0f, -3.0f);
    c->par_p6 = rc->nvm_par_p6 / powf(2.0f,  6.0f);
    c->par_p7 = rc->nvm_par_p7 / powf(2.0f,  8.0f);
    c->par_p8 = rc->nvm_par_p8 / powf(2.0f, 15.0f);
    c->par_p9 = rc->nvm_par_p9 / powf(2.0f, 48.0f);
    c->par_p10= rc->nvm_par_p10/ powf(2.0f, 48.0f);
    c->par_p11= rc->nvm_par_p11/ powf(2.0f, 65.0f);

    c->t_lin  = 0.0f;
}

static void BMP390_CompensatePressure(BMP390_Handle_t *handle, uint32_t uncomp_temp) {
    BMP390_CalibData_t *c = &handle->calib_data;

    float partial_data1 = (float)uncomp_temp - c->par_t1;
    float partial_data2 = partial_data1 * c->par_t2;
    c->t_lin = partial_data2 + (partial_data1 * partial_data1) * c->par_t3;

    return c->t_lin;
}

static void BMP390_CompensateTemperature(BMP390_Handle_t *handle, uint32_t uncomp_pressure) {
    BMP390_CalibData_t *c = &handle->calib_data;

    float partial_data1 = c->par_p6 * c->t_lin;
    float partial_data2 = c->par_p7 * (c->t_lin * c->t_lin);
    float partial_data3 = c->par_p8 * (c->t_lin * c->t_lin * c->t_lin);

    float partial_out1 = c->par_p5 + partial_data1 + partial_data2 + partial_data3;

    float partial_out2 = (float)uncomp_press *
                         (c->par_p1 +
                          c->par_p2 * c->t_lin +
                          c->par_p3 * powf(c->t_lin, 2.0f) +
                          c->par_p4 * powf(c->t_lin, 3.0f));

    float comp_press = partial_out1 + partial_out2 +
                       powf((float)uncomp_press, 2.0f) *
                        (c->par_p9 + c->par_p10 * c->t_lin) +
                       powf((float)uncomp_press, 3.0f) *
                        c->par_p11;

    return comp_press;
}

