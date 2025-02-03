#ifndef BMI088_ACCEL_H
#define BMI088_ACCEL_H

#include "stm32g4xx_hal.h"
#include <math.h>

#define BMI088_ACC_CHIP_ID        0x1E
#define BMI088_ACC_24G_RANGE      0x03
#define BMI088_ACC_BWP_OSR4       0x80
#define BMI088_ACC_ODR_200Hz      0x09

#define BMI088_ACC_REG_CHIP_ID    0x00
#define BMI088_ACC_REG_DATA       0x12
#define BMI088_ACC_REG_CONF       0x40
#define BMI088_ACC_REG_RANGE      0x41
#define BMI088_ACC_REG_PWR_CONF   0x7C
#define BMI088_ACC_REG_PWR_CTRL   0x7D
#define BMI088_ACC_REG_SOFTRESET  0x7E

#define BMI088_STANDARD_GRAVITY   9.80665f


typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
    float lastAccel[3];
    uint8_t rangeConf;      // BMI088_ACC_24G_RANGE
    uint8_t samplingConf;   // BMI088_ACC_BWP_OSR4 | BMI088_ACC_ODR_200Hz
} BMI088_AccelHandle_t;

int BMI088_Accel_Init(BMI088_AccelHandle_t *handle);
void BMI088_Accel_Step(BMI088_AccelHandle_t *handle);
void BMI088_Accel_Get(BMI088_AccelHandle_t *handle, float *outData);

uint8_t BMI088_Accel_ReadReg(BMI088_AccelHandle_t *handle, uint8_t reg);
void BMI088_Accel_WriteReg(BMI088_AccelHandle_t *handle, uint8_t reg, uint8_t data);
void BMI088_Accel_ReadBuf(BMI088_AccelHandle_t *handle, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
