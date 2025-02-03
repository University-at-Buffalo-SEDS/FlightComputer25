#ifndef FLASH_MEMORY_H
#define FLASH_MEMORY_H

// https://www.ti.com/lit/ug/sprugp2a/sprugp2a.pdf?ts=1738159993071
// https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/5059/W25Q32JV_RevI_5-4-21.pdf

#include "stm32g4xx_hal.h"
#include "stdint.h"

// Flash Memory global variables (maybe turn into const statics?)
#define FLASH_PAGE_SIZE         (256)
#define FLASH_BLOCK_SIZE        (65536)
#define FLASH_PAGES_PER_BLOCK   ((FLASH_BLOCK_SIZE) / (FLASH_PAGE_SIZE))
#define FLASH_PAGE_COUNT        ((1 << 22) / (FLASH_PAGE_SIZE))
#define FLASH_BLOCK_COUNT       ((FLASH_PAGE_COUNT) / (FLASH_PAGES_PER_BLOCK))
#define FLASH_FLIGHTS           (1)
#define FLASH_FLIGHT_PAGES      ((FLASH_PAGES_PER_BLOCK) * ((FLASH_BLOCK_COUNT) / (FLASH_FLIGHTS)))
#define FLASH_FLIGHT_SIZE       ((FLASH_FLIGHT_PAGES) * (FLASH_PAGE_SIZE))

// Main flash interface (returns 0 on fail, 1 otherwise)
int flash_setup();
int flash_erase(size_t page_addr);
int flash_write(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE]);
int flash_read(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE]);
int flash_is_busy();

#endif
