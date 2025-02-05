//#include "../../Inc/Drivers/flash_memory.h"
//
//#define FLASH_DEVICE_ID		(0x15)
//#define FLASH_BUSY_MASK		(0x01)
//#define FLASH_WEL_MASK 		(0x02)
//
//enum FlashInstruction {
//	PAGE_PROGRAM = 0x02,
//	READ_DATA = 0x03,
//	READ_STATUS_REGISTER_1 = 0x05,
// 	WRITE_ENABLE = 0x06,
//	BLOCK_ERASE_32KB = 0x52,
//	RELEASE_POWER_DOWN_DEVICE_ID = 0xAB,
//};
//
//// helper functions, returns 0 on fail, 1 else
//int flash_begin();
//int flash_end();
//int flash_send_command(FlashInstruction instruction);
//uint32_t flash_status_1();
//int flash_busy_internal();
//int flash_write_enable();
//int flash_wait_until_free();
//
