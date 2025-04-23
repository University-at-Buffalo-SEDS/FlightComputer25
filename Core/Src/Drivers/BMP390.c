#include "Drivers/BMP390.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <math.h>

#define BMP390_REG_STATUS       (0x03) // Status register address
#define BMP390_STATUS_DRDY_PRES (1 << 5) // Data ready pressure bit
#define BMP390_STATUS_DRDY_TEMP (1 << 6) // Data ready temperature bit
#define BMP390_SPI_TIMEOUT      (100)


static bool bmp_read_reg(BMP390 *baro, uint8_t reg, uint8_t *data) {
    uint8_t tx[3] = { reg | 0x80, 0x00, 0x00 };
    uint8_t rx[3] = {0};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(baro->csPort, baro->csPin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(baro->hspi, tx, rx, 3, BMP390_SPI_TIMEOUT);
    HAL_GPIO_WritePin(baro->csPort, baro->csPin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        debug_print("BMP390 SPI Error (Read Reg 0x%02X): %d\r\n", reg, status);
        return false;
    }

    *data = rx[2];
    return true;
}

static bool bmp_write_reg(BMP390 *baro, uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg & ~0x80, data };
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(baro->csPort, baro->csPin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(baro->hspi, tx, 2, BMP390_SPI_TIMEOUT);
    HAL_GPIO_WritePin(baro->csPort, baro->csPin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        debug_print("BMP390 SPI Error (Write Reg 0x%02X): %d\r\n", reg, status);
        return false;
    }
    return true;
}

static bool bmp_read_buf(BMP390 *baro, uint8_t reg, uint8_t *data, uint8_t len) {
    if (len == 0) return true;

    #define MAX_SPI_BUF_LEN 32
    if (len > MAX_SPI_BUF_LEN - 2) {
         debug_print("BMP390 Read Buf Error: len %d too large\r\n", len);
         return false;
    }

    uint8_t total = 2 + len;
    uint8_t tx[MAX_SPI_BUF_LEN] = {0};
    uint8_t rx[MAX_SPI_BUF_LEN] = {0};
    HAL_StatusTypeDef status;

    tx[0] = reg | 0x80;

    HAL_GPIO_WritePin(baro->csPort, baro->csPin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(baro->hspi, tx, rx, total, BMP390_SPI_TIMEOUT);
    HAL_GPIO_WritePin(baro->csPort, baro->csPin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        debug_print("BMP390 SPI Error (Read Buf 0x%02X, len %d): %d\r\n", reg, len, status);
        return false;
    }

    memcpy(data, rx + 2, len);
    return true;
}

static void bmp_parse_calib(BMP390 *baro, uint8_t *raw) {
    baro->raw_calib.nvm_par_t1 = BMP390_CONCAT2BYTES(raw[1], raw[0]);
    baro->calib.par_t1 = (float)baro->raw_calib.nvm_par_t1 / powf(2, -8);

    baro->raw_calib.nvm_par_t2 = BMP390_CONCAT2BYTES(raw[3], raw[2]);
    baro->calib.par_t2 = (float)baro->raw_calib.nvm_par_t2 / powf(2, 30);

    baro->raw_calib.nvm_par_t3 = (int8_t)raw[4];
    baro->calib.par_t3 = (float)baro->raw_calib.nvm_par_t3 / powf(2, 48);

    baro->raw_calib.nvm_par_p1 = (int16_t)BMP390_CONCAT2BYTES(raw[6], raw[5]);
    baro->calib.par_p1 = (((float)baro->raw_calib.nvm_par_p1) - powf(2,14)) / powf(2,20);

    baro->raw_calib.nvm_par_p2 = (int16_t)BMP390_CONCAT2BYTES(raw[8], raw[7]);
    baro->calib.par_p2 = (((float)baro->raw_calib.nvm_par_p2) - powf(2,14)) / powf(2,29);

    baro->raw_calib.nvm_par_p3 = (int8_t)raw[9];
    baro->calib.par_p3 = (float)baro->raw_calib.nvm_par_p3 / powf(2,32);

    baro->raw_calib.nvm_par_p4 = (int8_t)raw[10];
    baro->calib.par_p4 = (float)baro->raw_calib.nvm_par_p4 / powf(2,37);

    baro->raw_calib.nvm_par_p5 = BMP390_CONCAT2BYTES(raw[12], raw[11]);
    baro->calib.par_p5 = (float)baro->raw_calib.nvm_par_p5 / powf(2, -3);

    baro->raw_calib.nvm_par_p6 = BMP390_CONCAT2BYTES(raw[14], raw[13]);
    baro->calib.par_p6 = (float)baro->raw_calib.nvm_par_p6 / powf(2,6);

    baro->raw_calib.nvm_par_p7 = (int8_t)raw[15];
    baro->calib.par_p7 = (float)baro->raw_calib.nvm_par_p7 / powf(2,8);

    baro->raw_calib.nvm_par_p8 = (int8_t)raw[16];
    baro->calib.par_p8 = (float)baro->raw_calib.nvm_par_p8 / powf(2,15);

    baro->raw_calib.nvm_par_p9 = (int16_t)BMP390_CONCAT2BYTES(raw[18], raw[17]);
    baro->calib.par_p9 = (float)baro->raw_calib.nvm_par_p9 / powf(2,48);

    baro->raw_calib.nvm_par_p10 = (int8_t)raw[19];
    baro->calib.par_p10 = (float)baro->raw_calib.nvm_par_p10 / powf(2,48);

    baro->raw_calib.nvm_par_p11 = (int8_t)raw[20];
    baro->calib.par_p11 = (float)baro->raw_calib.nvm_par_p11 / powf(2,65);
}

static float bmp_compensate_temperature(BMP390 *baro, uint32_t uncomp_temp) {
    float partial1 = (float)uncomp_temp - baro->calib.par_t1;
    float partial2 = partial1 * baro->calib.par_t2;
    baro->calib.t_lin = partial2 + (partial1 * partial1) * baro->calib.par_t3;
    return baro->calib.t_lin;
}

static float bmp_compensate_pressure(BMP390 *baro, uint32_t uncomp_press) {
    float partial_data1 = baro->calib.par_p6 * baro->calib.t_lin;
    float partial_data2 = baro->calib.par_p7 * (baro->calib.t_lin * baro->calib.t_lin);
    float partial_data3 = baro->calib.par_p8 * (baro->calib.t_lin * baro->calib.t_lin * baro->calib.t_lin);
    float partial_out1 = baro->calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

    float flt_uncomp_press = (float)uncomp_press;

    float partial_data1_p = baro->calib.par_p2 * baro->calib.t_lin;
    float partial_data2_p = baro->calib.par_p3 * (baro->calib.t_lin * baro->calib.t_lin);
    float partial_data3_p = baro->calib.par_p4 * (baro->calib.t_lin * baro->calib.t_lin * baro->calib.t_lin);
    float partial_out2 = flt_uncomp_press * (baro->calib.par_p1 + partial_data1_p + partial_data2_p + partial_data3_p);


    float partial_data1_p2 = flt_uncomp_press * flt_uncomp_press;
    float partial_data2_p2 = baro->calib.par_p9 + baro->calib.par_p10 * baro->calib.t_lin;
    float partial_data3_p2 = partial_data1_p2 * partial_data2_p2;
    float partial_data4_p2 = partial_data3_p2 + (flt_uncomp_press * flt_uncomp_press * flt_uncomp_press) * baro->calib.par_p11;

    float comp_press = partial_out1 + partial_out2 + partial_data4_p2;

    return comp_press;
}

bool bmp_init(BMP390 *baro, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin) {
    baro->hspi = hspi;
    baro->csPort = csPort;
    baro->csPin = csPin;
    baro->last_press = NAN;
    baro->last_alt   = NAN;
    baro->last_temp  = INT16_MIN;

    HAL_GPIO_WritePin(baro->csPort, baro->csPin, GPIO_PIN_SET);
    HAL_Delay(10);

    uint8_t chip_id = 0;
    debug_print("BMP390 Init: Reading Chip ID...\r\n");
    if (!bmp_read_reg(baro, BMP390_REG_CHIP_ID, &chip_id)) {
        debug_print("BMP390 Init ERROR: Failed SPI read for Chip ID\r\n");
        return false;
    }
    if (chip_id != BMP390_CHIP_ID) {
        debug_print("BMP390 Init ERROR: Chip ID Mismatch! Read: 0x%02X Expected: 0x%02X\r\n", chip_id, BMP390_CHIP_ID);
        return false;
    }
    debug_print("BMP390 Init: Chip ID OK (0x%02X).\r\n", chip_id);


    debug_print("BMP390 Init: Sending Soft Reset...\r\n");
    if (!bmp_write_reg(baro, BMP390_REG_CMD, SOFT_RESET)) {
         debug_print("BMP390 Init ERROR: Failed SPI write for Soft Reset\r\n");
         return false;
    }
    HAL_Delay(50);

    uint8_t power_conf = ENABLE_PRESSURE | ENABLE_TEMP | ENABLE_SENSOR; // 0x33
    debug_print("BMP390 Init: Writing PWR_CTRL = 0x%02X...\r\n", power_conf);
    if (!bmp_write_reg(baro, BMP390_REG_PWR_CTRL, power_conf)) {
         debug_print("BMP390 Init ERROR: Failed SPI write for PWR_CTRL\r\n");
         return false;
    }
    HAL_Delay(5);

    uint8_t osr_conf = (OSR_TEMP_X2 << 3) | OSR_PRESSURE_X32; // 0x0D
    debug_print("BMP390 Init: Writing OSR = 0x%02X...\r\n", osr_conf);
    if (!bmp_write_reg(baro, BMP390_REG_OSR, osr_conf)) {
        debug_print("BMP390 Init ERROR: Failed SPI write for OSR\r\n");
        return false;
    }

    debug_print("BMP390 Init: Writing ODR = 0x%02X...\r\n", ODR_12p5_HZ); // 0x04
    if (!bmp_write_reg(baro, BMP390_REG_ODR, ODR_12p5_HZ)) {
        debug_print("BMP390 Init ERROR: Failed SPI write for ODR\r\n");
        return false;
    }
    HAL_Delay(5);

    uint8_t read_pwr = 0, read_osr = 0, read_odr = 0;
    bool readback_ok = true;
    debug_print("BMP390 Init: Reading back configuration...\r\n");
    if (!bmp_read_reg(baro, BMP390_REG_PWR_CTRL, &read_pwr)) readback_ok = false;
    if (!bmp_read_reg(baro, BMP390_REG_OSR, &read_osr)) readback_ok = false;
    if (!bmp_read_reg(baro, BMP390_REG_ODR, &read_odr)) readback_ok = false;

    if (!readback_ok) {
        debug_print("BMP390 Init ERROR: Failed SPI read during configuration readback!\r\n");
        return false;
    }

    debug_print("BMP390 Init Readback: PWR_CTRL=0x%02X (W:0x%02X), OSR=0x%02X (W:0x%02X), ODR=0x%02X (W:0x%02X)\r\n",
                read_pwr, power_conf, read_osr, osr_conf, read_odr, ODR_12p5_HZ);

    if (read_pwr != power_conf || read_osr != osr_conf || read_odr != ODR_12p5_HZ) {
        debug_print("BMP390 Init WARNING: Configuration Readback Mismatch! Sensor may not operate as expected.\r\n");
    } else {
        debug_print("BMP390 Init: Configuration Readback OK.\r\n");
    }

    uint8_t calib_raw[21] = {0};
    debug_print("BMP390 Init: Reading Calibration Data...\r\n");
    if (!bmp_read_buf(baro, BMP390_REG_CAL, calib_raw, 21)) {
        debug_print("BMP390 Init ERROR: Failed SPI read for Calibration Data\r\n");
        return false;
    }
    bmp_parse_calib(baro, calib_raw);

    HAL_Delay(100);

    baro_step(baro);

    return true;
}

void baro_step(BMP390 *baro) {
    uint8_t status_reg = 0;
    uint8_t data[6] = {0};

    if (!bmp_read_reg(baro, BMP390_REG_STATUS, &status_reg)) {
        debug_print("baro_step ERROR: Failed to read STATUS register.\r\n");
        return;
    }

    if (!(status_reg & BMP390_STATUS_DRDY_PRES) || !(status_reg & BMP390_STATUS_DRDY_TEMP)) {
        return;
    }

    if (!bmp_read_buf(baro, BMP390_REG_DATA, data, 6)) {
        debug_print("baro_step ERROR: Failed to read DATA registers after DRDY set!\r\n");
        return;
    }

    uint32_t uncomp_press = BMP390_CONCAT3BYTES(data[2], data[1], data[0]);
    uint32_t uncomp_temp  = BMP390_CONCAT3BYTES(data[5], data[4], data[3]);

    if (uncomp_press == 8388608 || uncomp_temp == 8388608) {
         debug_print("BMP390 WARN: Read reset value (8388608) despite DRDY flags being set!\r\n");
    }

    float t_lin = bmp_compensate_temperature(baro, uncomp_temp);
    baro->last_temp = (int16_t)(t_lin * 100.0f);

    float comp_press = bmp_compensate_pressure(baro, uncomp_press);
    baro->last_press = comp_press;

    baro->last_alt = 44330.0f * (1.0f - powf(comp_press / 101325.0f, 1.0f / 5.255f));
}

float baro_get_altitude(BMP390 *baro) {
    return baro->last_alt;
}

int16_t baro_get_temp(BMP390 *baro) {
    return baro->last_temp;
}

float baro_get_pressure(BMP390 *baro) {
    return baro->last_press;
}
