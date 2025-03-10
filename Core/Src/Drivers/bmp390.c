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
	//To read, we follow figure 20 from page 43 of the datasheet
	//https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf#page=43

	//We are performing a read function, so we set the first bit(bit 7) of the register to 1
	reg |= 0x80;

	//Start SPI communication by writing the CS pin low
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);

	//Start by writing register address over SPI
	HAL_SPI_Transmit(handle->hspi, &reg, SPI_DATASIZE_8BIT, READ_TIMEOUT);

	//We receive a dummy byte so we will do nothing with it
	HAL_SPI_Recieve(handle->hspi, &reg, SPI_DATASIZE_8BIT, READ_TIMEOUT);

	//We wish to read one byte of data actual data so we read 1 byte
	uint8_t register_data;
	HAL_SPI_Recieve(handle->hspi, &register_data, SPI_DATASIZE_8BIT, READ_TIMEOUT);

	return register_data;

}

void BMP390_WriteReg(BMP390_Handle_t *handle, uint8_t reg, uint8_t data) {
	//Start SPI communication by writing the CS pin low
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);

	//Here, we are writing the register address and the data to be written as per page 42, Figure 18
	//https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf#page=42
	HAL_SPI_Transmit(handle->hspi, &reg, SPI_DATASIZE_8BIT, WRITE_TIMEOUT);
	HAL_SPI_Transmit(handle->hspi, &data, SPI_DATASIZE_8BIT, WRITE_TIMEOUT);

	//End SPI communication by writing the CS pin high
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);

}

void BMP390_ReadBuffer(BMP390_Handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len) {

}

static void BMP390_LoadCalibrationData(BMP390_Handle_t *handle, const uint8_t *raw_data) {

}

static float BMP390_CompensatePressure(BMP390_Handle_t *handle, uint32_t uncomp_temp) {

}

static float BMP390_CompensateTemperature(BMP390_Handle_t *handle, uint32_t uncomp_pressure) {

}

