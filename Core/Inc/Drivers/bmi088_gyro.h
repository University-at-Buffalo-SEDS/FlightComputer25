#ifndef BMI088_GYRO_H
#define BMI088_GYRO_H

#include "stm32g4xx_hal.h"
#include <math.h>

#define BMI088_GYR_CHIP_ID               0x0F
#define BMI088_GYR_2000DPS_RANGE         0x00
#define BMI088_GYR_2000DPS_RES_LSB_DPS   16.384f
#define BMI088_GYR_ODR_100Hz_BW_32Hz     0x07

// Register addresses
#define BMI088_GYR_REG_CHIP_ID           0x00
#define BMI088_GYR_REG_DATA              0x02
#define BMI088_GYR_REG_RANGE             0x0F
#define BMI088_GYR_REG_BANDWIDTH         0x10
#define BMI088_GYR_REG_SOFTRESET         0x14

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csPort;
	uint16_t csPin;
	float lastGyro[3];
} BMI088_GyroHandle_t;

int BMI088_Gyro_Init(BMI088_GyroHandle_t *handle);
void BMI088_Gyro_Step(BMI088_GyroHandle_t *handle);
void BMI088_Gyro_Get(BMI088_GyroHandle_t *handle, float* data);

uint8_t BMI088_ReadReg(BMI088_GyroHandle_t *handle, uint8_t reg);
void BMI088_WriteReg(BMI088_GyroHandle_t *handle, uint8_t reg, uint8_t data);
void BMI088_ReadBuf(BMI088_GyroHandle_t *handle, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
