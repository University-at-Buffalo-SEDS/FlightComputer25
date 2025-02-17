
#include "Drivers/bmi088_accel.h"

static inline void spi_start(accelHandle_t *accel) {
    HAL_GPIO_WritePin(accel->csPort, accel->csPin, GPIO_PIN_RESET);
}

static inline void spi_end(accelHandle_t *accel) {
    HAL_GPIO_WritePin(accel->csPort, accel->csPin, GPIO_PIN_SET);
}

static uint8_t read_reg(accelHandle_t *accel, uint8_t reg) {
    uint8_t tx[3] = {reg | 0x80, 0x00, 0x00};
    uint8_t rx[3];

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
    HAL_SPI_Transmit(accel->hspi, header, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive(accel->hspi, data, len, HAL_MAX_DELAY);
    spi_end(accel);
}

void accel_init(accelHandle_t *accel) {

    spi_start(accel);
    HAL_Delay(1);
    spi_end(accel);
    HAL_Delay(50);


    write_reg(accel, BMI088_ACC_REG_SOFTRESET, 0xB6);
    HAL_Delay(50);

    (void)read_reg(accel, BMI088_ACC_REG_CHIP_ID);
    uint8_t chip_id = read_reg(accel, BMI088_ACC_REG_CHIP_ID);
    if (chip_id != BMI088_ACC_REG_CHIP_ID) {
        CDC_Transmit_Print("BMI088 Accel not found! Read chip id: 0x%02X\r\n", chip_id);
    } else {
        CDC_Transmit_Print("BMI088 Accel found! Chip id: 0x%02X\r\n", chip_id);
    }

    write_reg(accel, BMI088_ACC_REG_PWR_CONF, 0x00);
    HAL_Delay(50);
    write_reg(accel, BMI088_ACC_REG_PWR_CTRL, 0x04);


    write_reg(accel, BMI088_ACC_REG_CONF, accel->samplingConf);
    write_reg(accel, BMI088_ACC_REG_RANGE, accel->rangeConf);
    HAL_Delay(50);

    if (read_reg(accel, BMI088_ACC_REG_CONF) != accel->samplingConf) {
    	CDC_Transmit_Print("BMI088 Accel incorrect sampling rate set!\r\n");
    }
    if ((read_reg(accel, BMI088_ACC_REG_RANGE) & 0x03) != accel->rangeConf) {
    	CDC_Transmit_Print("BMI088 Accel incorrect range set!\r\n");
    }
    if (read_reg(accel, BMI088_ACC_REG_PWR_CTRL) != 0x04) {
    	CDC_Transmit_Print("BMI088 Accel did not turn on!\r\n");
    }

    chip_id = read_reg(accel, BMI088_ACC_REG_CHIP_ID);
	CDC_Transmit_Print("DID ANOTHER READ OF THE ACC CHIP ID: 0x%02X\r\n", chip_id);

   if (chip_id != BMI088_ACC_REG_CHIP_ID) {
	   CDC_Transmit_Print("BMI088 Accel not found! Read chip id: 0x%02X\r\n", chip_id);
   } else {
	   CDC_Transmit_Print("BMI088 Accel found! Chip id: 0x%02X\r\n", chip_id);
   }

    accel_step(accel);
}

void accel_step(accelHandle_t *accel) {
    uint8_t raw_data[6];
    read_buf(accel, BMI088_ACC_REG_DATA, raw_data, 6);

    uint16_t raw_x = (uint16_t)((raw_data[1] << 8) | raw_data[0]);
    uint16_t raw_y = (uint16_t)((raw_data[3] << 8) | raw_data[2]);
    uint16_t raw_z = (uint16_t)((raw_data[5] << 8) | raw_data[4]);

    float multiplier = (1.0f / (1 << 15)) * (1 << (accel->rangeConf + 1)) * 1.5f;

    accel->lastAccel[0] = raw_x * multiplier * STANDARD_GRAVITY;
    accel->lastAccel[1] = raw_y * multiplier * STANDARD_GRAVITY;
    accel->lastAccel[2] = raw_z * multiplier * STANDARD_GRAVITY;
}

float* accel_get(accelHandle_t *accel) {
	return accel->lastAccel;
}

void accel_print(accelHandle_t *accel)
{
    float *a = accel_get(accel);
    float magnitude = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    CDC_Transmit_Print("Accel: %.2f, %.2f, %.2f (%.2f m/s^2)\r\n", a[0], a[1], a[2], magnitude);
}
