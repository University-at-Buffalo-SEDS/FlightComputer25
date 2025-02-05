#include "Drivers/bmi088_accel.h"

static inline void spi_start(accelHandle_t *accel) {
	HAL_GPIO_WritePin(accel->csPort, accel->csPin, GPIO_PIN_RESET);
}

static inline void spi_end(accelHandle_t *accel) {
	HAL_GPIO_WritePin(accel->csPort, accel->csPin, GPIO_PIN_SET);
}

static uint8_t read_reg(accelHandle_t *accel, uint8_t reg) {
	uint8_t tx[3] = {reg | 0x80, 0x00, 0x00};
	uint8_t rx[3] = {0, 0, 0};

	spi_start(accel);
	HAL_SPI_TransmitReceive(accel->hspi, tx, rx, 3, HAL_MAX_DELAY);
	spi_end(accel);

	return rx[2];
}

static void write_reg(accelHandle_t *accel, uint8_t reg, uint8_t data) {
	uint8_t tx[2] = {reg, data};

	spi_start(accel);
	HAL_SPI_Transmit(accel->hspi, tx, 2, HAL_MAX_DELAY);
	spi_end(accel);
}

static void read_buf(accelHandle_t *accel, uint8_t reg, uint8_t *data, uint8_t len) {
	uint8_t header[2] = {reg | 0x80, 0x00};

	spi_start(accel);
}
