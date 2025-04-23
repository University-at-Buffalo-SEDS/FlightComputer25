#include <Drivers/bmi088.h>

void accel_read_reg(BMI088 *imu, uint8_t regAddr, uint8_t *data) {
	uint8_t tx[3] = {regAddr | 0x80, 0x00, 0x00};
	uint8_t rx[3];

	HAL_GPIO_WritePin(imu->csAccelPinBank, imu->accelCSPin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(imu->hspi, tx, rx, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(imu->csAccelPinBank, imu->accelCSPin, GPIO_PIN_SET);
	if (status == HAL_OK) {
		*data = rx[2];
	} else {
		debug_print("Error: 0x%02X\r\n", status);
	}
}

void gyro_read_reg(BMI088 *imu, uint8_t regAddr, uint8_t *data) {
	uint8_t tx[2] = {regAddr | 0x80, 0x00};
	uint8_t rx[2];

	HAL_GPIO_WritePin(imu->csGyroPinBank, imu->gyroCSPin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(imu->hspi, tx, rx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(imu->csGyroPinBank, imu->gyroCSPin, GPIO_PIN_SET);
	if (status == HAL_OK) {
		*data = rx[1];
	} else {
		debug_print("Error: 0x%02X\r\n", status);
	}
}

void accel_write_reg(BMI088 *imu, uint8_t regAddr, uint8_t data) {
	uint8_t tx[2] = {regAddr, data};
	HAL_GPIO_WritePin(imu->csAccelPinBank, imu->accelCSPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(imu->hspi, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(imu->csAccelPinBank, imu->accelCSPin, GPIO_PIN_SET);
}

void gyro_write_reg(BMI088 *imu, uint8_t regAddr, uint8_t data) {
	uint8_t tx[2] = {regAddr, data};
	HAL_GPIO_WritePin(imu->csGyroPinBank, imu->gyroCSPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(imu->hspi, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(imu->csGyroPinBank, imu->gyroCSPin, GPIO_PIN_SET);
}

void bmi088_init(BMI088 *imu,
		SPI_HandleTypeDef *spi,
		GPIO_TypeDef *csAccelPinBank,
		GPIO_TypeDef *csGyroPinBank,
		uint16_t accelCSPin,
		uint16_t gyroCSPin) {

	imu->hspi = spi;
	imu->csAccelPinBank = csAccelPinBank;
	imu->csGyroPinBank = csGyroPinBank;

	imu->accelCSPin = accelCSPin;
	imu->gyroCSPin = gyroCSPin;

	float accel_multipilier = 1.0f/(1<<15) * (1<<(BMI088_ACC_24G_RANGE + 1)) * 1.5f;
	imu->accelConversion = 9.80665f * accel_multipilier; // data sheet page 27
	imu->gyroConversion = 0.01745329251f * 1000.0f / 16384.0f; // data sheet page 39 (rad/s)

	// accel setup
	HAL_GPIO_WritePin(imu->csAccelPinBank, imu->accelCSPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(imu->csAccelPinBank, imu->accelCSPin, GPIO_PIN_SET);
	HAL_Delay(50);

	//soft reset the accelerometer
	accel_write_reg(imu, BMI088_ACC_REG_SOFTRESET, 0xB6);
	HAL_Delay(50);

	uint8_t dummy;
	accel_read_reg(imu, BMI088_ACC_REG_CHIP_ID, &dummy);

	uint8_t chipID;
	accel_read_reg(imu, BMI088_ACC_REG_CHIP_ID, &chipID);
	if (chipID != 0x1E) {
		debug_print("Chip ID Reg Address: 0x%02X\r\n", chipID);
	}
	HAL_Delay(10);

	gyro_read_reg(imu, BMI088_GYR_REG_CHIP_ID, &chipID);
	if (chipID != 0x0F) {
		debug_print("fuck fuck 0x%02X\r\n", chipID);
	}
	HAL_Delay(10);

	accel_write_reg(imu, BMI088_ACC_REG_PWR_CONF, 0x00);
	HAL_Delay(10);

	accel_write_reg(imu, BMI088_ACC_REG_RANGE, BMI088_ACC_24G_RANGE);
	HAL_Delay(10);

	accel_write_reg(imu, BMI088_ACC_REG_CONF, BMI088_ACC_ODR_200Hz | BMI088_ACC_BWP_OSR4);
	HAL_Delay(10);


	accel_write_reg(imu, BMI088_ACC_REG_PWR_CTRL, 0x04);
	HAL_Delay(10);

	uint8_t range;
	accel_read_reg(imu, BMI088_ACC_REG_RANGE, &range);
	if ((range & 0x03) != BMI088_ACC_24G_RANGE) {
		debug_print("Wrong accel range set! 0x%02X\r\n", range);
	} else {
		debug_print("Correct accel range set! 0x%02X\r\n", range);
	}

	uint8_t sampling;
	accel_read_reg(imu, BMI088_ACC_REG_CONF, &sampling);
	if (sampling != (BMI088_ACC_ODR_200Hz | BMI088_ACC_BWP_OSR4)) {
		debug_print("Wrong accel sampling set! 0x%02X\r\n", sampling);
	} else {
		debug_print("Correct accel sampling set! 0x%02X\r\n", sampling);
	}

	uint8_t isOn;
	accel_read_reg(imu, BMI088_ACC_REG_PWR_CTRL, &isOn);
	if (isOn != 0x04) {
		debug_print("Accel not turned on! 0x%02X\r\n", isOn);
	} else {
		debug_print("Accel turned on! 0x%02X\r\n", isOn);
	}

	// gyro setup
	HAL_GPIO_WritePin(imu->csGyroPinBank, imu->gyroCSPin, GPIO_PIN_SET);

	gyro_write_reg(imu, BMI088_GYR_REG_SOFTRESET, 0xB6);
	HAL_Delay(250);

	gyro_write_reg(imu, BMI088_GYR_REG_RANGE, BMI088_GYR_2000DPS_RANGE);
	HAL_Delay(10);

	gyro_write_reg(imu, BMI088_GYR_REG_BANDWIDTH, BMI088_GYR_ODR_100Hz_BW_32Hz);
	HAL_Delay(10);
}

void accel_step(BMI088 *imu) {
	uint8_t tx[8] = {(BMI088_ACC_REG_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx[8];

	HAL_GPIO_WritePin(imu->csAccelPinBank, imu->accelCSPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(imu->hspi, tx, rx, 8, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(imu->csAccelPinBank, imu->accelCSPin, GPIO_PIN_SET);

	int16_t raw_x = (int16_t) ((rx[3] << 8) | rx[2]);
	int16_t raw_y = (int16_t) ((rx[5] << 8) | rx[4]);
	int16_t raw_z = (int16_t) ((rx[7] << 8) | rx[6]);

	imu->lastAccel[0] = raw_x * imu->accelConversion;
	imu->lastAccel[1] = raw_y * imu->accelConversion;
	imu->lastAccel[2] = raw_z * imu->accelConversion;
}

void gyro_step(BMI088 *imu) {
	uint8_t tx[7] = {(BMI088_GYR_REG_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx[7];

	HAL_GPIO_WritePin(imu->csGyroPinBank, imu->gyroCSPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(imu->hspi, tx, rx, 7, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(imu->csGyroPinBank, imu->gyroCSPin, GPIO_PIN_SET);

	int16_t raw_x = (int16_t) ((rx[2] << 8) | rx[1]);
	int16_t raw_y = (int16_t) ((rx[4] << 8) | rx[3]);
	int16_t raw_z = (int16_t) ((rx[6] << 8) | rx[5]);

	imu->lastGyro[0] = raw_x * imu->gyroConversion;
	imu->lastGyro[1] = raw_y * imu->gyroConversion;
	imu->lastGyro[2] = raw_z * imu->gyroConversion;
}

float *accel_get(BMI088 *imu) {
	return imu->lastAccel;
}

float *gyro_get(BMI088 *imu) {
	return imu->lastGyro;
}
