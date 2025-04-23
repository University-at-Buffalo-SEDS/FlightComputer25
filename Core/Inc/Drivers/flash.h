#ifndef FLASH_H
#define FLASH_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Flash memory organization */
#define EXT_FLASH_PAGE_SIZE        (256U)
#define EXT_FLASH_BLOCK_SIZE       (65536U)

/* Flash command set */
#define FLASH_CMD_PAGE_PROGRAM         (0x02U)
#define FLASH_CMD_READ_DATA            (0x03U)
#define FLASH_CMD_READ_STATUS_REGISTER (0x05U)
#define FLASH_CMD_WRITE_ENABLE         (0x06U)
#define FLASH_CMD_BLOCK_ERASE_32KB     (0x52U)
#define FLASH_CMD_RELEASE_POWER_DOWN   (0xABU)

/* Flash status register bit masks */
#define FLASH_BUSY_MASK (0x01U)
#define FLASH_WEL_MASK  (0x02U)

void Flash_Setup(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_Pin);

void Flash_Erase(uint32_t page_addr);

void Flash_Write(uint32_t page_addr, uint8_t page[EXT_FLASH_PAGE_SIZE]);

void Flash_Read(uint32_t page_addr, uint8_t page[EXT_FLASH_PAGE_SIZE]);

bool Flash_Busy(void);

#endif
