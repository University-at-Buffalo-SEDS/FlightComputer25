#ifndef BMI088_H
#define BMI088_H

#include "stm32g4xx_hal.h"
#include <math.h>

// BMI088 accel values
#define BMI088_ACC_24G_RANGE            (0x03)
#define BMI088_ACC_BWP_OSR4             (0x80)
#define BMI088_ACC_ODR_200Hz            (0x09)

// BMI088 accel register addresses
#define BMI088_ACC_REG_CHIP_ID          (0x00)
#define BMI088_ACC_REG_DATA             (0x12)
#define BMI088_ACC_REG_CONF             (0x40)
#define BMI088_ACC_REG_RANGE            (0x41)
#define BMI088_ACC_REG_PWR_CONF         (0x7C)
#define BMI088_ACC_REG_PWR_CTRL         (0x7D)
#define BMI088_ACC_REG_SOFTRESET        (0x7E)

// BMI088 gyro Values
#define BMI088_GYR_CHIP_ID              (0x0F)
#define BMI088_GYR_2000DPS_RANGE        (0x00)
#define BMI088_GYR_ODR_100Hz_BW_32Hz    (0x07)

// BMI088 gyro register addresses
#define BMI088_GYR_REG_CHIP_ID          (0x00)
#define BMI088_GYR_REG_DATA             (0x02)
#define BMI088_GYR_REG_RANGE            (0x0F)
#define BMI088_GYR_REG_BANDWIDTH        (0x10)
#define BMI088_GYR_REG_LPM1             (0x11)
#define BMI088_GYR_REG_SOFTRESET        (0x14)

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csAccelPinBank;
    GPIO_TypeDef *csGyroPinBank;
    uint16_t accelCSPin;
    uint16_t gyroCSPin;
    float lastAccel[3];
    float lastGyro[3];
    float accelConversion;
    float gyroConversion;
} BMI088;

void bmi088_init(BMI088 *imu, SPI_HandleTypeDef *spi, GPIO_TypeDef *csAccelPinBank, GPIO_TypeDef *csGyroPinBank, uint16_t accelCSPin, uint16_t gyroCSPin);

void accel_read_reg(BMI088 *imu, uint8_t regAddr, uint8_t *data);
void gyro_read_reg(BMI088 *imu, uint8_t regAddr, uint8_t *data);

void accel_write_reg(BMI088 *imu, uint8_t regAddr, uint8_t data);
void gyro_write_reg(BMI088 *imu, uint8_t regAddr, uint8_t data);

void accel_step(BMI088 *imu);
void gyro_step(BMI088 *imu);

float *accel_get(BMI088 *imu);
float *gyro_get(BMI088 *imu);

#endif
