#ifndef __STM32ZERO_SIO_HPP__
#define __STM32ZERO_SIO_HPP__

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

// Use common Status and IoResult from stm32zero.hpp
using Status = stm32zero::Status;
using IoResult = stm32zero::IoResult;

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
 * @return IoResult { OK on success, count = bytes written }
 */
IoResult write(const void* data, size_t len);

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
 * Check if ready to write (TX buffer has space)
 *
 * @return IoResult { OK if space available, BUFFER_FULL otherwise, count = free space }
 */
IoResult writable();

/**
 * Wait until ready to write
 *
 * @param timeout_ms Timeout in milliseconds
 * @return IoResult { OK if ready, TIMEOUT on timeout, count = free space }
 */
IoResult wait_writable(uint32_t timeout_ms);

/**
 * Wait until all pending TX data is sent
 *
 * @param timeout_ms Timeout in milliseconds
 * @return IoResult { OK if flushed, TIMEOUT on timeout }
 */
IoResult flush(uint32_t timeout_ms);

/**
 * Get TX peak usage
 */
uint16_t write_peak();

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
 * @return IoResult { OK if data read, BUFFER_EMPTY if empty, count = bytes read }
 */
IoResult read(void* data, size_t len);

/**
 * Read data from input buffer with timeout
 *
 * Blocking until requested bytes received or timeout.
 * Useful for binary protocols.
 *
 * @param data Pointer to destination buffer
 * @param len Bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return IoResult { OK on success, TIMEOUT on timeout, count = bytes read }
 */
IoResult read(void* data, size_t len, uint32_t timeout_ms);

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
 * @return IoResult { OK on success, TIMEOUT on timeout, count = characters read }
 */
IoResult readln(char* buf, size_t len, uint32_t timeout_ms = UINT32_MAX);

/**
 * Check if data available to read
 *
 * @return IoResult { OK if data available, BUFFER_EMPTY otherwise, count = available bytes }
 */
IoResult readable();

/**
 * Wait until data available to read
 *
 * @param timeout_ms Timeout in milliseconds
 * @return IoResult { OK if data available, TIMEOUT on timeout, count = available bytes }
 */
IoResult wait_readable(uint32_t timeout_ms = UINT32_MAX);

/**
 * Discard all data in RX buffer
 */
void purge();

/**
 * Get RX peak usage
 */
uint16_t read_peak();

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
/**
 * Get RX semaphore handle (for QueueSet, etc.)
 */
SemaphoreHandle_t semaphore();
#endif

} // namespace sio
} // namespace stm32zero

#endif // __STM32ZERO_SIO_HPP__
