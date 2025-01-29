#include "bmi088_gyro.h"

static inline void BMI088_Gyro_Select(BMI088_GyroHandle_t *handle) {
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);
}

static inline void BMI088_Gyro_Deselect(BMI088_GyroHandle_t *handle){
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);
}

uint8_t BMI088_Gyro_ReadReg(BMI088_GyroHandle_t *handle, uint8_t reg) {
	uint8_t tx = reg | 0x80;
	uint8_t rx = 0x00;

	BMI088_Gyro_Select(handle);

	HAL_SPI_Transmit(handle->hspi, &tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(handle->hspi, &rx, 1, HAL_MAX_DELAY);

	BMI088_Gyro_Deselect(handle);

	return rx;
}

void BMI088_Gyro_WriteReg(BMI088_GyroHandle_t *handle, uint8_t reg, uint8_t data) {
    uint8_t tx[2];
    tx[0] = (reg & 0x7F);
    tx[1] = data;

    BMI088_Gyro_Select(handle);

    HAL_SPI_Transmit(handle->hspi, tx, 2, HAL_MAX_DELAY);

    BMI088_Gyro_Deselect(handle);
}


void BMI088_Gyro_ReadBuf(BMI088_GyroHandle_t *handle, uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t tx = reg | 0x80;

    BMI088_Gyro_Select(handle);

    HAL_SPI_Transmit(handle->hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(handle->hspi, buf, len, HAL_MAX_DELAY);

    BMI088_Gyro_Deselect(handle);
}

int BMI088_Gyro_Init(BMI088_GyroHandle_t *handle) {

	int status = 1;

    handle->lastGyro[0] = NAN;
    handle->lastGyro[1] = NAN;
    handle->lastGyro[2] = NAN;

    BMI088_Gyro_Deselect(handle);

    uint8_t chipID = BMI088_Gyro_ReadReg(handle, BMI088_GYR_REG_CHIP_ID);
    if (chipID != BMI088_GYR_CHIP_ID) {
        status = 0;
    }

    BMI088_Gyro_WriteReg(handle, BMI088_GYR_REG_SOFTRESET, 0xB6);
    HAL_Delay(50);

    BMI088_Gyro_WriteReg(handle, BMI088_GYR_REG_RANGE, BMI088_GYR_2000DPS_RANGE);

    BMI088_Gyro_WriteReg(handle, BMI088_GYR_REG_BANDWIDTH, BMI088_GYR_ODR_100Hz_BW_32Hz);
    HAL_Delay(50);

    if (BMI088_Gyro_ReadReg(handle, BMI088_GYR_REG_RANGE) != BMI088_GYR_2000DPS_RANGE) {
    	status = 0;
    }

    uint8_t bwRegVal = BMI088_Gyro_ReadReg(handle, BMI088_GYR_REG_BANDWIDTH) & 0x7F; // mask out MSB
    if (bwRegVal != BMI088_GYR_ODR_100Hz_BW_32Hz) {
    	status = 0;
    }

    BMI088_Gyro_Step(handle);
    return status;
}

void BMI088_Gyro_Step(BMI088_GyroHandle_t *handle) {
    uint8_t rawData[6] = {0};
    BMI088_Gyro_ReadBuf(handle, BMI088_GYR_REG_DATA, rawData, 6);

    int16_t raw_x = ((int16_t)rawData[1] << 8) | rawData[0];
    int16_t raw_y = ((int16_t)rawData[3] << 8) | rawData[2];
    int16_t raw_z = ((int16_t)rawData[5] << 8) | rawData[4];

    handle->lastGyro[0] = (float)raw_x / BMI088_GYR_2000DPS_RES_LSB_DPS;
    handle->lastGyro[1] = (float)raw_y / BMI088_GYR_2000DPS_RES_LSB_DPS;
    handle->lastGyro[2] = (float)raw_z / BMI088_GYR_2000DPS_RES_LSB_DPS;
}

void BMI088_Gyro_Get(BMI088_GyroHandle_t *handle, float *data) {
    data[0] = handle->lastGyro[0];
    data[1] = handle->lastGyro[1];
    data[2] = handle->lastGyro[2];
}
