#ifndef BMP390_H
#define BMP390_H

#include "stm32g4xx_hal.h"
#include <math.h>

// Registers
#define BMP390_CHIP_ID       (0x60)
#define BMP390_REG_CHIP_ID   (0x00)
#define BMP390_REG_DATA      (0x04)
#define BMP390_REG_PWR_CTRL  (0x1B)
#define BMP390_REG_OSR       (0x1C)
#define BMP390_REG_ODR       (0x1D)
#define BMP390_REG_CAL       (0x31)
#define BMP390_REG_CMD       (0x7E)

#define BMP390_SOFT_RESET       (0xB6)
#define BMP390_ENABLE_PRESSURE  (0x01)
#define BMP390_ENABLE_TEMP      (0x02)
#define BMP390_ENABLE_SENSOR    (0x30)
#define BMP390_OSR_x2           (0x01)
#define BMP390_OSR_x32          (0x05)
#define BMP390_ODR_12p5_HZ      (0x04)

#define BMP390_SPI_READ_BIT     (0x80)

//Time in ms
#define WRITE_TIMEOUT		100
#define READ_TIMEOUT		100
#define TIMEOUT				1000

typedef struct {
    uint16_t nvm_par_t1;
    uint16_t nvm_par_t2;
    int8_t   nvm_par_t3;
    int16_t  nvm_par_p1;
    int16_t  nvm_par_p2;
    int8_t   nvm_par_p3;
    int8_t   nvm_par_p4;
    uint16_t nvm_par_p5;
    uint16_t nvm_par_p6;
    int8_t   nvm_par_p7;
    int8_t   nvm_par_p8;
    int16_t  nvm_par_p9;
    int8_t   nvm_par_p10;
    int8_t   nvm_par_p11;
} BMP390_RawCalibData_t;

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
} BMP390_CalibData_t;

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csPort;
	uint16_t csPin;
	BMP390_CalibData_t calib_data;
	BMP390_RawCalibData_t raw_calib_data;
	float last_altitude;
	float last_pressure;
	float last_temperature;
} BMP390_Handle_t;


int BMP390_Init(BMP390_Handle_t *handle);
void BMP390_Step(BMP390_Handle_t *handle);
float BMP390_GetAltitude(BMP390_Handle_t *handle);
float BMP390_GetPressure(BMP390_Handle_t *handle);
float BMP390_GetTemperature(BMP390_Handle_t *handle);

uint8_t BMP390_ReadReg(BMP390_Handle_t *handle, uint8_t reg);
void BMP390_WriteReg(BMP390_Handle_t *handle, uint8_t reg, uint8_t data);
void BMP390_ReadBuffer(BMP390_Handle_t *handle, uint8_t reg, uint8_t *data_buffer, uint8_t len);

#endif
