#ifndef BMP390_H
#define BMP390_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#define BMP390_CHIP_ID (0x60)
#define ENABLE_PRESSURE (0x01)
#define ENABLE_TEMP (0x02)
#define ENABLE_SENSOR (0x30)

#define OSR_TEMP_X2 (0x01)
#define OSR_PRESSURE_X32 (0x05)

#define ODR_12p5_HZ (0x04)
#define SOFT_RESET (0xB6)

#define BMP390_REG_CHIP_ID (0x00)
#define BMP390_REG_DATA (0x04)
#define BMP390_REG_EVENT (0x10)
#define BMP390_REG_PWR_CTRL (0x1B)
#define BMP390_REG_OSR (0x1C)
#define BMP390_REG_ODR (0x1D)
#define BMP390_REG_IIR (0x1F)
#define BMP390_REG_CAL (0x31)
#define BMP390_REG_CMD (0x7E)

#define BMP390_CONCAT2BYTES(msb, lsb) (((uint16_t)(msb) << 8) | (uint16_t)(lsb))
#define BMP390_CONCAT3BYTES(msb, lsb, xlsb) (((uint32_t)(msb) << 16) | ((uint32_t)(lsb) << 8) | (uint32_t)(xlsb))

typedef struct {
	uint16_t nvm_par_t1;
	uint16_t nvm_par_t2;
	int8_t nvm_par_t3;
	int16_t nvm_par_p1;
	int16_t nvm_par_p2;
	int8_t nvm_par_p3;
	int8_t nvm_par_p4;
	uint16_t nvm_par_p5;
	uint16_t nvm_par_p6;
	int8_t nvm_par_p7;
	int8_t nvm_par_p8;
	int16_t nvm_par_p9;
	int8_t nvm_par_p10;
	int8_t nvm_par_p11;
} BMP390_RawCalibData;

typedef struct {
	float par_t1;
	float par_t2;
	float par_t3;
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float t_lin;
} BMP390_CalibData;

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csPort;
	uint16_t csPin;
	float last_press; // in Pascals
	float last_alt; // in meters
	int16_t last_temp; // in centi-Celsius (°C * 100)
	BMP390_RawCalibData raw_calib;
	BMP390_CalibData calib;
} BMP390;

bool bmp_init(BMP390 *baro, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);
void baro_step(BMP390 *baro);
float baro_get_altitude(BMP390 *baro);
int16_t baro_get_temp(BMP390 *baro);
float baro_get_pressure(BMP390 *baro);

#endif
