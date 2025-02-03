#include "Drivers/bmi088_accel.h"

static inline void BMI088_Accel_Select(BMI088_AccelHandle_t *handle) {
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);
}

static inline void BMI088_Accel_Deselect(BMI088_AccelHandle_t *handle) {
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);
}

uint8_t BMI088_Accel_ReadReg(BMI088_AccelHandle_t *handle, uint8_t reg) {
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | 0x80;
    tx[1] = 0x00;

    BMI088_Accel_Select(handle);
    HAL_SPI_TransmitReceive(handle->hspi, tx, rx, 2, HAL_MAX_DELAY);

    BMI088_Accel_Deselect(handle);

    return rx[1];
}

void BMI088_Accel_WriteReg(BMI088_AccelHandle_t *handle, uint8_t reg, uint8_t data) {
    uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = data;

    BMI088_Accel_Select(handle);

    HAL_SPI_Transmit(handle->hspi, tx, 2, HAL_MAX_DELAY);

    BMI088_Accel_Deselect(handle);
}

void BMI088_Accel_ReadBuf(BMI088_AccelHandle_t *handle, uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t tx[2];
    tx[0] = (reg | 0x80);
    tx[1] = 0x00;

    BMI088_Accel_Select(handle);

    HAL_SPI_Transmit(handle->hspi, tx, 2, HAL_MAX_DELAY);

    HAL_SPI_Receive(handle->hspi, buf, len, HAL_MAX_DELAY);

    BMI088_Accel_Deselect(handle);
}

int BMI088_Accel_Init(BMI088_AccelHandle_t *handle) {
    int status = 1;

    handle->lastAccel[0] = NAN;
    handle->lastAccel[1] = NAN;
    handle->lastAccel[2] = NAN;

    BMI088_Accel_Select(handle);
    HAL_Delay(1);
    BMI088_Accel_Deselect(handle);
    HAL_Delay(50);

    BMI088_Accel_WriteReg(handle, BMI088_ACC_REG_SOFTRESET, 0xB6);
    HAL_Delay(50);

    (void)BMI088_Accel_ReadReg(handle, BMI088_ACC_REG_CHIP_ID);

    uint8_t chipID = BMI088_Accel_ReadReg(handle, BMI088_ACC_REG_CHIP_ID);
    CDC_Transmit_Print("CHIP ID: %0x02X\r\n", chipID);
    if (chipID != BMI088_ACC_CHIP_ID) {
        status = 0;
    }

    BMI088_Accel_WriteReg(handle, BMI088_ACC_REG_PWR_CONF, 0x00);
    HAL_Delay(50);

    BMI088_Accel_WriteReg(handle, BMI088_ACC_REG_PWR_CTRL, 0x04);

    BMI088_Accel_WriteReg(handle, BMI088_ACC_REG_CONF, handle->samplingConf);

    BMI088_Accel_WriteReg(handle, BMI088_ACC_REG_RANGE, handle->rangeConf);

    HAL_Delay(50);

    uint8_t confRead = BMI088_Accel_ReadReg(handle, BMI088_ACC_REG_CONF);
    if (confRead != handle->samplingConf) {
        status = 0;
    }

    uint8_t rangeRead = BMI088_Accel_ReadReg(handle, BMI088_ACC_REG_RANGE) & 0x03;
    if (rangeRead != (handle->rangeConf & 0x03)) {
        status = 0;
    }

    uint8_t pwrCtrl = BMI088_Accel_ReadReg(handle, BMI088_ACC_REG_PWR_CTRL);
    if (pwrCtrl != 0x04) {
        status = 0;
    }

    BMI088_Accel_Step(handle);

    return status;
}

void BMI088_Accel_Step(BMI088_AccelHandle_t *handle) {
    uint8_t raw[6] = {0};
    BMI088_Accel_ReadBuf(handle, BMI088_ACC_REG_DATA, raw, 6);

    int16_t rawX = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t rawY = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rawZ = (int16_t)((raw[5] << 8) | raw[4]);

    float rangeFactor = (float)(1 << (handle->rangeConf + 1)) * 1.5f;
    float BMI088_MULTIPLIER = (rangeFactor / 32768.0f);

    handle->lastAccel[0] = (float)rawX * BMI088_MULTIPLIER * BMI088_STANDARD_GRAVITY;
    handle->lastAccel[1] = (float)rawY * BMI088_MULTIPLIER * BMI088_STANDARD_GRAVITY;
    handle->lastAccel[2] = (float)rawZ * BMI088_MULTIPLIER * BMI088_STANDARD_GRAVITY;
}

void BMI088_Accel_Get(BMI088_AccelHandle_t *handle, float *outData) {
    outData[0] = handle->lastAccel[0];
    outData[1] = handle->lastAccel[1];
    outData[2] = handle->lastAccel[2];
}
