#ifndef STM32ZERO_SOUT_HPP
#define STM32ZERO_SOUT_HPP

/**
 * STM32ZERO Stdout Module - DMA-based printf output
 *
 * Features:
 *   - Lock-free dual buffer for efficient DMA transmission
 *   - ISR and Task safe (auto-detects context via IPSR register)
 *   - printf() works from both Task and ISR context
 *   - Auto callback registration (no manual ISR code needed)
 *
 * Configuration (stm32zero-conf.h):
 *   - STM32ZERO_STDOUT_UART: UART handle name (required, e.g., huart3)
 *   - STM32ZERO_STDOUT_BUFFER_SIZE: Buffer size in bytes (default: 4096)
 *   - STM32ZERO_STDOUT_CACHE_CLEAN: Enable cache clean (default: 0)
 *
 * Requirements:
 *   - Link stm32zero-stdout.cpp to your project
 *   - UART with DMA TX configured in STM32CubeMX
 *   - USE_HAL_UART_REGISTER_CALLBACKS=1 in stm32h7xx_hal_conf.h
 *
 * STM32CubeMX DMA Settings:
 *   - HAL: Enable both DMA and DMA interrupt (NVIC)
 *   - LL:  Enable DMA only (interrupt not required)
 *
 * Usage:
 *   // In main.c after MX_USARTx_UART_Init():
 *   stm32zero::sout::init();
 *
 *   // Then just use printf():
 *   printf("Hello, World!\r\n");
 */

#include <cstdint>
#include <cstddef>

namespace stm32zero {
namespace sout {

/**
 * Initialize stdout module
 *
 * Registers UART TX complete callback automatically.
 * Call after MX_USARTx_UART_Init().
 *
 * Requires USE_HAL_UART_REGISTER_CALLBACKS=1 in stm32h7xx_hal_conf.h
 */
void init();

/**
 * DMA transfer complete callback
 *
 * Called automatically if init() was used.
 * Can also be called manually from HAL_UART_TxCpltCallback().
 */
void tx_complete_isr();

/**
 * Write data to stdout buffer
 *
 * ISR-safe: auto-detects context and uses appropriate critical section.
 * Can be called from both Task and ISR context.
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
 * ISR-safe: can be called from both Task and ISR context.
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

} // namespace sout
} // namespace stm32zero

#endif // STM32ZERO_SOUT_HPP
