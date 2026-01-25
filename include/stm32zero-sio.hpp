#ifndef STM32ZERO_SIO_HPP
#define STM32ZERO_SIO_HPP

/**
 * STM32ZERO Serial I/O Module - Default UART for debug/console
 *
 * sio is the default UART interface configured via stm32zero-conf.h.
 * For additional UARTs, use STM32ZERO_DEFINE_UART() from stm32zero-uart.hpp.
 *
 * Features:
 *   - DMA-based RX with ring buffer and idle line detection
 *   - DMA-based TX with dual buffer
 *   - ISR and Task safe
 *   - write()/read() based I/O (no printf, no newlib)
 *
 * Configuration (stm32zero-conf.h):
 *   - STM32ZERO_SIO_UART: UART handle name (required, e.g., huart3)
 *   - STM32ZERO_SIO_RX_SIZE: RX ring buffer size (default: 256)
 *   - STM32ZERO_SIO_TX_SIZE: TX dual buffer size (default: 4096)
 *   - STM32ZERO_SIO_DMA_SIZE: RX DMA buffer size (default: 64)
 *
 * Requirements:
 *   - UART with DMA RX and TX configured in STM32CubeMX
 *   - UART global interrupt enabled for IDLE detection
 *   - USE_HAL_UART_REGISTER_CALLBACKS=1 in stm32h7xx_hal_conf.h
 *
 * Usage:
 *   // Default UART (sio)
 *   stm32zero::sio::init();
 *   sio::write("Hello\r\n", 7);
 *   sio::readln(buf, sizeof(buf), 1000);
 *
 *   // Additional UARTs - use stm32zero-uart.hpp
 *   STM32ZERO_DEFINE_UART(gps, huart2, 128, 512);
 *   STM32ZERO_INIT_UART(gps, huart2);
 *   gps.write(cmd, len);
 */

#include <cstdint>
#include <cstddef>
#include <cstdarg>
#include <cstdio>

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
#include "semphr.h"
#endif

namespace stm32zero {
namespace sio {

/**
 * Initialize sio module
 *
 * Call after MX_USARTx_UART_Init().
 * Safe to call multiple times (only first call has effect).
 */
void init();

//=============================================================================
// Write (TX)
//=============================================================================

/**
 * Write data to output buffer
 *
 * ISR-safe. If buffer is full, data is truncated.
 *
 * @param data Pointer to data
 * @param len Data length
 * @return Number of bytes actually written
 */
int write(const void* data, size_t len);

/**
 * Formatted write (snprintf + write)
 *
 * ISR-safe. If buffer is full, data is truncated.
 *
 * @param buf Caller-provided buffer for formatting
 * @param size Buffer size
 * @param fmt printf format string
 * @param args va_list arguments
 * @return snprintf return value (>= size means truncated)
 */
int vwritef(char* buf, size_t size, const char* fmt, va_list args);

/**
 * Formatted write (snprintf + write)
 *
 * ISR-safe. If buffer is full, data is truncated.
 *
 * @param buf Caller-provided buffer for formatting
 * @param fmt printf format string
 * @param ... format arguments
 * @return snprintf return value (>= N means truncated)
 */
template<size_t N>
inline int writef(char (&buf)[N], const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	int len = vwritef(buf, N, fmt, args);
	va_end(args);
	return len;
}

int writef(char* buf, size_t size, const char* fmt, ...);

/**
 * Flush pending TX data
 *
 * Starts DMA transfer if there's data and not already busy.
 *
 * @return true if new transfer started
 */
bool flush();

/**
 * Check if TX is in progress
 */
bool is_tx_busy();

/**
 * Get pending bytes in TX buffer
 */
uint16_t tx_pending();

/**
 * Get TX high water mark
 */
uint16_t tx_water_mark();

//=============================================================================
// Read (RX)
//=============================================================================

/**
 * Read data from input buffer
 *
 * ISR-safe. Non-blocking.
 *
 * @param data Pointer to destination buffer
 * @param len Maximum bytes to read
 * @return Number of bytes actually read (0 if buffer empty)
 */
int read(void* data, size_t len);

/**
 * Read data from input buffer with timeout
 *
 * Blocking until requested bytes received or timeout.
 * Useful for binary protocols.
 *
 * @param data Pointer to destination buffer
 * @param len Bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes actually read
 */
int read(void* data, size_t len, uint32_t timeout_ms);

/**
 * Read a line (until newline character)
 *
 * Blocks until newline received or timeout.
 * Newline characters are stripped from result.
 * Buffer is always null-terminated.
 *
 * @param buf Destination buffer
 * @param len Buffer size (including null terminator)
 * @param timeout_ms Timeout in milliseconds
 * @return Number of characters read (excluding null), -1 on timeout with no data
 */
int readln(char* buf, size_t len, uint32_t timeout_ms = UINT32_MAX);

/**
 * Wait until RX data is available
 *
 * @param timeout_ms Timeout in milliseconds
 * @return true if data available, false if timeout
 */
bool wait(uint32_t timeout_ms = UINT32_MAX);

/**
 * Get number of bytes available to read
 */
size_t available();

/**
 * Check if RX buffer is empty
 */
bool is_empty();

/**
 * Get RX high water mark
 */
uint16_t rx_water_mark();

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
/**
 * Get RX semaphore handle (for QueueSet, etc.)
 */
SemaphoreHandle_t semaphore();
#endif

} // namespace sio
} // namespace stm32zero

#endif // STM32ZERO_SIO_HPP
