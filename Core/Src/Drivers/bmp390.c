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
	handle->raw_calib_data.nvm_par_t1 = (uint16_t)raw_data[1] << 8 | raw_data[0];
	handle->calib_data.par_t1 = handle->raw_calib_data.nvm_par_t1 / powf(2, -8);
	handle->raw_calib_data.nvm_par_t2 = (uint16_t)raw_data[3] << 8 | raw_data[2];
	handle->calib_data.par_t2 = handle->raw_calib_data.nvm_par_t2 / powf(2, 30);
	handle->raw_calib_data.nvm_par_t3 = raw_data[4];
	handle->calib_data.par_t3 = handle->raw_calib_data.nvm_par_t3 / powf(2, 48);
	handle->raw_calib_data.nvm_par_p1 = (int16_t)raw_data[6] << 8 | raw_data[5];
	handle->calib_data.par_p1 = (handle->raw_calib_data.nvm_par_p1 - powf(2, 14)) / powf(2, 20);
	handle->raw_calib_data.nvm_par_p2 = (int16_t)raw_data[8] << 8 | raw_data[7];
	handle->calib_data.par_p2 = (handle->raw_calib_data.nvm_par_p2 - powf(2, 14)) / powf(2, 29);
	handle->raw_calib_data.nvm_par_p3 = raw_data[9];
	handle->calib_data.par_p3 = handle->raw_calib_data.nvm_par_p3 / powf(2, 32);
	handle->raw_calib_data.nvm_par_p4 = raw_data[10];
	handle->calib_data.par_p4 = handle->raw_calib_data.nvm_par_p4 / powf(2, 37);
	handle->raw_calib_data.nvm_par_p5 = (uint16_t)raw_data[12] << 8 | raw_data[11];
	handle->calib_data.par_p5 = handle->raw_calib_data.nvm_par_p5 / powf(2, -3);
	handle->raw_calib_data.nvm_par_p6 = (uint16_t)raw_data[14] << 8 | raw_data[13];
	handle->calib_data.par_p6 = handle->raw_calib_data.nvm_par_p6 / powf(2, 6);
	handle->raw_calib_data.nvm_par_p7 = raw_data[15];
	handle->calib_data.par_p7 = handle->raw_calib_data.nvm_par_p7 / powf(2, 8);
	handle->raw_calib_data.nvm_par_p8 = raw_data[16];
	handle->calib_data.par_p8 = handle->raw_calib_data.nvm_par_p8 / powf(2, 15);
	handle->raw_calib_data.nvm_par_p9 = (int16_t)raw_data[18] << 8 | raw_data[17];
	handle->calib_data.par_p9 = handle->raw_calib_data.nvm_par_p9 / powf(2, 48);
	handle->raw_calib_data.nvm_par_p10 = raw_data[19];
	handle->calib_data.par_p10 = handle->raw_calib_data.nvm_par_p10 / powf(2, 48);
	handle->raw_calib_data.nvm_par_p11 = raw_data[20];
	handle->calib_data.par_p11 = handle->raw_calib_data.nvm_par_p11 / powf(2, 65);
}

static float BMP390_CompensatePressure(BMP390_Handle_t *handle, uint32_t uncomp_pressure) {
	float partial_data1 = handle->calib_data.par_p6 * handle->calib_data.t_lin;
	float partial_data2 = handle->calib_data.par_p7 * (handle->calib_data.t_lin * handle->calib_data.t_lin);
	float partial_data3 = handle->calib_data.par_p8 * (handle->calib_data.t_lin * handle->calib_data.t_lin * handle->calib_data.t_lin);
	float partial_out1 = handle->calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;
	float partial_out2 = uncomp_pressure * (handle->calib_data.par_p1 + handle->calib_data.par_p2 * handle->calib_data.t_lin + handle->calib_data.par_p3 * pow(handle->calib_data.t_lin, 2) + handle->calib_data.par_p4 * pow(handle->calib_data.t_lin, 3));
	return partial_out1 + partial_out2 + pow(uncomp_pressure, 2) * (handle->calib_data.par_p9 + handle->calib_data.par_p10 * handle->calib_data.t_lin) + pow(uncomp_pressure, 3) * handle->calib_data.par_p11;
}

static float BMP390_CompensateTemperature(BMP390_Handle_t *handle, uint32_t uncomp_temp) {
	float partial_data1 = (float)(uncomp_temp - handle->calib_data.par_t1);
	float partial_data2 = partial_data1 * handle->calib_data.par_t2;
	handle->calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * handle->calib_data.par_t3;
	return handle->calib_data.t_lin;
}

