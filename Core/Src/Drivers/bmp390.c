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

/**
  * @brief  Reads the given register via SPI
  * @param  Pointer to the SPI handle
  * @param 	Register address
  * @retval Data stored in register
  */
uint8_t BMP390_ReadReg(BMP390_Handle_t *handle, uint8_t reg) {
	//To read, we follow figure 20 from page 43 of the datasheet
	//https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf#page=43

	//We are performing a read function, so we set the first bit(bit 7) of the register to 1
	reg |= 0x80;

	//Start SPI communication by writing the CS pin low
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);

	//We will use TransmitReceive, which will transmit and receive in the same
	//function(crazy huh?).
	//When using the SPI protocol, the first byte the BMP390 will send
	//is a dummy byte, so we have to ignore the first byte. After that
	//first one, we have the actual data, so we put that into the array we
	//use to receive the data
	uint8_t register_data[2];
	HAL_SPI_TransmitReceive(handle->hspi, reg, &register_data, 2, READ_TIMEOUT);

	//Return only the second byte(ignoring the first dummy byte)
	return register_data[1];

}

/**
  * @brief  Writes the given data to the given register
  * @param  Pointer to the SPI handle
  * @param 	Register address
  * @param  Data to write
  */
void BMP390_WriteReg(BMP390_Handle_t *handle, uint8_t reg, uint8_t data) {
	//Start SPI communication by writing the CS pin low
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);

	//Here, we are writing the register address and the data to be written as per page 42, Figure 18
	//https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf#page=42
	HAL_SPI_Transmit(handle->hspi, &reg, 1, WRITE_TIMEOUT);
	HAL_SPI_Transmit(handle->hspi, &data, 1, WRITE_TIMEOUT);

	//End SPI communication by writing the CS pin high
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);

}

/**
  * @brief  Reads the data in a buffer, i.e. reads multiple registers in a row
  * @param  Pointer to the SPI handle
  * @param 	Register address
  * @param  Array to store data
  * @param 	Length of buffer we want to read i.e. how many registers we want to read sequentially
  */
void BMP390_ReadBuffer(BMP390_Handle_t *handle, uint8_t reg, uint8_t *data_buffer, uint8_t len) {
	//To read, we follow figure 20 from page 43 of the datasheet
	//The only difference from the BMP390_ReadReg and BMP390 ReadBuffer is that
	//the BMP390_ReadBuffer reads multiple bytes of data
	//https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf#page=43

	//We are performing a read function, so we set the first bit(bit 7) of the register to 1
	reg |= 0x80;

	//Start SPI communication by writing the CS pin low
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);

	//When using the SPI protocol, the first byte the BMP390 will send
	//is a dummy byte, so we have to ignore the first byte. After that
	//first one, we have the actual data, so we put that into the data buffer
	//https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf#page=43
	uint8_t register_data[len + 1];
	HAL_SPI_TransmitReceive(handle->hspi, &reg, &register_data, len + 1, READ_TIMEOUT);

	for(int i = 0; i < len; i++) {
		data_buffer[i] = register_data[i + 1];
	}
}

static void BMP390_LoadCalibrationData(BMP390_Handle_t *handle, const uint8_t *raw_data) {

}

static float BMP390_CompensatePressure(BMP390_Handle_t *handle, uint32_t uncomp_temp) {

}

static float BMP390_CompensateTemperature(BMP390_Handle_t *handle, uint32_t uncomp_pressure) {

}

