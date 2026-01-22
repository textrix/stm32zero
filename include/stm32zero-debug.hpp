#ifndef STM32ZERO_DEBUG_HPP
#define STM32ZERO_DEBUG_HPP

/**
 * STM32ZERO Debug Module - DMA-based printf output
 *
 * Features:
 *   - Lock-free dual buffer for efficient DMA transmission
 *   - ISR and Task safe (uses critical sections)
 *   - printf() automatically redirected via _write() override
 *
 * Configuration (stm32zero-conf.h):
 *   - STM32ZERO_DEBUG_UART: UART handle name (required, e.g., huart3)
 *   - STM32ZERO_DEBUG_BUFFER_SIZE: Buffer size in bytes (default: 4096)
 *   - STM32ZERO_DEBUG_CACHE_CLEAN: Enable cache clean (default: 0)
 *
 * Requirements:
 *   - Link stm32zero-debug.cpp to your project
 *   - UART with DMA TX configured in STM32CubeMX
 *
 * Usage:
 *   // In HAL_UART_TxCpltCallback (stm32h7xx_it.c):
 *   #include "stm32zero-debug.hpp"
 *   void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
 *       stm32zero::debug::tx_complete_isr();
 *   }
 *
 *   // Then just use printf():
 *   printf("Hello, World!\r\n");
 */

#include <cstdint>
#include <cstddef>

namespace stm32zero {
namespace debug {

/**
 * DMA transfer complete callback
 *
 * Must be called from HAL_UART_TxCpltCallback().
 */
void tx_complete_isr();

/**
 * Write data to debug buffer
 *
 * Thread-safe: can be called from ISR or Task context.
 * If buffer is full, data is truncated.
 *
 * @param data Pointer to data
 * @param len Data length
 * @return Number of bytes actually written
 */
int write(const void* data, size_t len);

/**
 * Flush pending data
 *
 * Starts DMA transfer if there's data and not already busy.
 *
 * @return true if new transfer started
 */
bool flush();

/**
 * Check if transmission is in progress
 */
bool is_busy();

/**
 * Get pending bytes in current fill buffer
 */
uint16_t pending();

/**
 * Get high water mark (maximum buffer usage)
 */
uint16_t water_mark();

/**
 * Get buffer size
 */
size_t buffer_size();

} // namespace debug
} // namespace stm32zero

#endif // STM32ZERO_DEBUG_HPP
