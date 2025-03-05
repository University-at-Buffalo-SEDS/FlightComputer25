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


    return 1;
}

void BMP390_Step(BMP390_Handle_t *handle) {

}

float BMP390_GetTemperature(BMP390_Handle_t *handle) {

}

float BMP390_GetAltitude(BMP390_Handle_t *handle) {

}

float BMP390_GetPressure(BMP390_Handle_t *handle) {
}

uint8_t BMP390_ReadReg(BMP390_Handle_t *handle, uint8_t reg) {

}

void BMP390_WriteReg(BMP390_Handle_t *handle, uint8_t reg, uint8_t data) {

}

void BMP390_ReadBuffer(BMP390_Handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len) {

}

static void BMP390_LoadCalibrationData(BMP390_Handle_t *handle, const uint8_t *raw_data) {

}

static float BMP390_CompensatePressure(BMP390_Handle_t *handle, uint32_t uncomp_temp) {

}

static float BMP390_CompensateTemperature(BMP390_Handle_t *handle, uint32_t uncomp_pressure) {

}

