#include "Drivers/flash.h"
#include "stm32g4xx_hal.h"
#include <string.h>

// Static storage for SPI handle and chip-select configuration.
static SPI_HandleTypeDef *flash_hspi = NULL;
static GPIO_TypeDef *flash_cs_port = NULL;
static uint16_t flash_cs_pin = 0;

// Expected flash device ID for your chip (optional; not used in error handling here)
#define W25Q32JV_DEVICE_ID (0x15U)

//--------------------------------------------------------------------
// Static helper functions
//--------------------------------------------------------------------
static void spi_begin(void)
{
    HAL_GPIO_WritePin(flash_cs_port, flash_cs_pin, GPIO_PIN_RESET);
}

static void spi_end(void)
{
    HAL_GPIO_WritePin(flash_cs_port, flash_cs_pin, GPIO_PIN_SET);
}

static uint8_t flash_status_1(void)
{
    uint8_t cmd = FLASH_CMD_READ_STATUS_REGISTER;
    uint8_t status = 0;

    spi_begin();
    HAL_SPI_Transmit(flash_hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(flash_hspi, &status, 1, HAL_MAX_DELAY);
    spi_end();

    return status;
}

static void flash_write_enable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
    spi_begin();
    HAL_SPI_Transmit(flash_hspi, &cmd, 1, HAL_MAX_DELAY);
    spi_end();
}

void Flash_Setup(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_Pin)
{
    uint8_t cmd;
    uint8_t dummy[3] = {0, 0, 0};
    uint8_t device_id = 0;

    // Save the provided configuration for later use.
    flash_hspi = hspi;
    flash_cs_port = CS_GPIO_Port;
    flash_cs_pin = CS_Pin;

    // Ensure CS is high initially.
    HAL_GPIO_WritePin(flash_cs_port, flash_cs_pin, GPIO_PIN_SET);

    // Release flash from power-down.
    cmd = FLASH_CMD_RELEASE_POWER_DOWN;
    spi_begin();
    HAL_SPI_Transmit(flash_hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(flash_hspi, dummy, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(flash_hspi, &device_id, 1, HAL_MAX_DELAY);
    spi_end();

    // (Optional) Compare device_id with W25Q32JV_DEVICE_ID to verify correct flash.
}

bool Flash_Busy(void)
{
    // Return true if the busy bit is set in the status register.
    return ((flash_status_1() & FLASH_BUSY_MASK) != 0);
}

void Flash_Write(uint32_t page_addr, uint8_t page[EXT_FLASH_PAGE_SIZE])
{
    uint8_t cmd;
    uint8_t addr[3];

    flash_write_enable();

    cmd = FLASH_CMD_PAGE_PROGRAM;
    // Convert the page index into a 24-bit address.
    // Here the page index corresponds to a byte address of (page_addr << 8).
    addr[0] = (uint8_t)((page_addr >> 8) & 0xFF);
    addr[1] = (uint8_t)(page_addr & 0xFF);
    addr[2] = 0;

    spi_begin();
    HAL_SPI_Transmit(flash_hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(flash_hspi, addr, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(flash_hspi, page, EXT_FLASH_PAGE_SIZE, HAL_MAX_DELAY);
    spi_end();

    // Wait for the write operation to complete.
    while (Flash_Busy())
    {
        HAL_Delay(1);
    }
}

void Flash_Read(uint32_t page_addr, uint8_t page[EXT_FLASH_PAGE_SIZE])
{
    uint8_t cmd;
    uint8_t addr[3];

    cmd = FLASH_CMD_READ_DATA;
    addr[0] = (uint8_t)((page_addr >> 8) & 0xFF);
    addr[1] = (uint8_t)(page_addr & 0xFF);
    addr[2] = 0;

    spi_begin();
    HAL_SPI_Transmit(flash_hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(flash_hspi, addr, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(flash_hspi, page, EXT_FLASH_PAGE_SIZE, HAL_MAX_DELAY);
    spi_end();
}

void Flash_Erase(uint32_t page_addr)
{
    uint8_t cmd;
    uint8_t addr[3];

    flash_write_enable();

    cmd = FLASH_CMD_BLOCK_ERASE_32KB;
    addr[0] = (uint8_t)((page_addr >> 8) & 0xFF);
    addr[1] = (uint8_t)(page_addr & 0xFF);
    addr[2] = 0;

    spi_begin();
    HAL_SPI_Transmit(flash_hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(flash_hspi, addr, 3, HAL_MAX_DELAY);
    spi_end();

    // Wait for the erase operation to complete.
    while (Flash_Busy())
    {
        HAL_Delay(1);
    }
}
