#include "log.h"
#include "Drivers/flash.h"          // Flash driver (see previous implementation)
#include "util/ring_buffer.h"    // Generic ring buffer implementation in C
#include "stm32g4xx_hal.h"  // (Or your appropriate HAL header)
#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
  Macro Definitions
----------------------------------------------------------------------------*/
#define LOG_BUF_SIZE         128    // Maximum number of LogMessage records in memory
#define LOG_WRITE_BUF_SIZE   (sizeof(LogMessage) * LOG_BUF_SIZE)

// Define maximum flash pages available for logging.
// For example, if flash memory is 2^22 bytes and each page is 256 bytes:
#define FLASH_PAGE_COUNT     ((1UL << 22) / EXT_FLASH_PAGE_SIZE)

// The number of pages in one flash block.
#define FLASH_PAGES_PER_BLOCK  (EXT_FLASH_BLOCK_SIZE / EXT_FLASH_PAGE_SIZE)

/*----------------------------------------------------------------------------
  Static Variables and Ring Buffer Storage
----------------------------------------------------------------------------*/
// Storage for LogMessage ring buffer (in RAM)
static LogMessage logBufferStorage[LOG_BUF_SIZE];
static RingBuffer logBuffer; // To hold LogMessage records

// Storage for write buffer (raw bytes for flash page writes)
static uint8_t writeBufferStorage[LOG_WRITE_BUF_SIZE];
static RingBuffer writeBuffer; // To store byte-stream for flash writes

// Variables controlling flash write progress.
static bool write_enabled = false;
static size_t current_page = 0;
static size_t written_pages = 0;

/*----------------------------------------------------------------------------
  Internal Function Prototypes
----------------------------------------------------------------------------*/
static void log_print_msg(const LogMessage *msg);

/*----------------------------------------------------------------------------
  Public Logging Functions
----------------------------------------------------------------------------*/

void log_setup(void)
{
    // Initialize ring buffers.
    ring_buffer_init(&logBuffer, logBufferStorage, LOG_BUF_SIZE, sizeof(LogMessage));
    ring_buffer_init(&writeBuffer, writeBufferStorage, LOG_WRITE_BUF_SIZE, sizeof(uint8_t));

    // Reset flash write pointer.
    current_page = 0;
    written_pages = 0;

    // Optionally, if desired, erase the flash log area now.
     log_erase();  // Uncomment if you want to start with a blank flash.
}

void log_start(void)
{
    write_enabled = true;
    // Immediately call log_step() to process any queued log messages.
    log_step();
}

void log_stop(void)
{
    write_enabled = false;
}

void log_add(const LogMessage *data)
{
    // Attempt to push the log record into the ring buffer.
    // (If buffer is full and overwrite is enabled, data will be overwritten.)
    bool ok = ring_buffer_push(&logBuffer, data, true);
    if (!ok && write_enabled) {
        // In a real system you might log an error message
        debug_print("Log buffer overflow!\r\n");
    }
}

/*
 * Erase the entire flash log area.
 * This iterates over all flash blocks and erases them.
 */
void log_erase(void)
{
    size_t total_blocks = FLASH_PAGE_COUNT / FLASH_PAGES_PER_BLOCK;
    for (size_t block = 0; block < total_blocks; ++block) {
        // Calculate starting page for this block.
        size_t page_addr = block * FLASH_PAGES_PER_BLOCK;
        Flash_Erase(page_addr);
    }
    // Reset write pointer.
    current_page = 0;
    written_pages = 0;
}

/*
 * Print all logged data from flash.
 * This function reads from flash pages [0, written_pages) and prints records.
 */
void log_print_all(void)
{
    if (write_enabled) {
        debug_print("Cannot read while logging is active!\r\n");
        return;
    }

    // Temporary read buffer to accumulate data.
    uint8_t page[EXT_FLASH_PAGE_SIZE];
    // Create a temporary ring buffer to parse the flash content.
    RingBuffer readBuffer;
    // Allocate storage for read buffer equal to one flash page write buffer.
    uint8_t readBufferStorage[LOG_WRITE_BUF_SIZE];
    ring_buffer_init(&readBuffer, readBufferStorage, LOG_WRITE_BUF_SIZE, sizeof(uint8_t));

    LogMessage msg;
    // Loop over all written pages.
    for (size_t page_i = 0; page_i < written_pages; ++page_i) {
        // Read a page from flash.
        Flash_Read(page_i, page);
        // Push this page into the read buffer.
        if (!ring_buffer_push_array(&readBuffer, page, EXT_FLASH_PAGE_SIZE, false)) {
            debug_print("Read buffer error.\r\n");
            break;
        }

        // While there is at least one complete LogMessage in the read buffer,
        // pop it and print.
        while (ring_buffer_used(&readBuffer) >= sizeof(LogMessage)) {
            if (!ring_buffer_pop_array(&readBuffer, (uint8_t *)&msg, sizeof(LogMessage))) {
                break;
            }
            log_print_msg(&msg);
        }
    }
}

/*
 * log_step:
 * This function transfers log messages from the log ring buffer (RAM) into the
 * write ring buffer (a byte stream). When at least one full flash page of data is
 * available in the write buffer, it writes it to flash.
 */
void log_step(void)
{
    if (!write_enabled || written_pages >= FLASH_PAGE_COUNT) {
        return;
    }

    LogMessage temp;
    // Transfer as many LogMessage records as possible from logBuffer to writeBuffer.
    // (We treat the record as a raw block of bytes.)
    while (ring_buffer_available(&writeBuffer) >= sizeof(LogMessage)) {
        if (!ring_buffer_pop(&logBuffer, &temp)) {
            break;
        }
        // Push raw bytes of the log message into the write buffer.
        ring_buffer_push_array(&writeBuffer, (uint8_t *)&temp, sizeof(temp), false);
    }

    uint8_t page[EXT_FLASH_PAGE_SIZE];
    // Write out full flash pages from the write buffer.
    while (written_pages < FLASH_PAGE_COUNT && !Flash_Busy()) {
        // (Optionally, if your flash requires erasing each block before writing,
        // you could check for block boundaries here and erase accordingly.
        // In this implementation, it is assumed that a full erase was done before logging.)

        // Check if a full page is available in the write buffer.
        if (ring_buffer_used(&writeBuffer) < EXT_FLASH_PAGE_SIZE) {
            break;  // Not enough data yet.
        }

        // Pop one flash page worth of data.
        ring_buffer_pop_array(&writeBuffer, page, EXT_FLASH_PAGE_SIZE);

        // Write this page to flash.
        Flash_Write(current_page, page);

        current_page++;
        written_pages++;
    }
}

/*
 * log_print_msg:
 * Prints a log message in CSV format.
 */
static void log_print_msg(const LogMessage *msg)
{
    // Print time in milliseconds.
    debug_print("%lu,", (unsigned long) msg->time_ms);
    // Print flight phase (assuming FlightPhase is an enum; cast to int).
    debug_print("%d,", (int) msg->phase);
    // Print the state vector components.
    debug_print("%.2f,", msg->kf_pos);
    debug_print("%.2f,", msg->kf_vel);
    debug_print("%.2f,", msg->kf_accel);
    // Print altitude.
    debug_print("%.2f,", msg->altitude);
    // Print acceleration values.
    debug_print("%.2f,", msg->accel_x);
    debug_print("%.2f,", msg->accel_y);
    debug_print("%.2f,", msg->accel_z);
    // Print gyro values.
    debug_print("%.2f,", msg->gyro_x);
    debug_print("%.2f,", msg->gyro_y);
    debug_print("%.2f,", msg->gyro_z);
    // Print pressure.
    debug_print("%.2f,", msg->pressure);
    // Print temperature.
    debug_print("%d,", msg->temp);
    // Print apogee.
    debug_print("%lu,", (unsigned long) msg->apogee);
    // Print launched flag as an integer (0 or 1).
    debug_print("%d,", (int) msg->launched);
    // Print landed_time.
    debug_print("%lu\r\n", (unsigned long) msg->landed_time);
}

