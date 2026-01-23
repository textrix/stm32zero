#ifndef STM32ZERO_SIN_HPP
#define STM32ZERO_SIN_HPP

/**
 * STM32ZERO Stdin Module - DMA-based UART RX with Idle Line Detection
 *
 * Features:
 *   - Ring buffer for asynchronous data reception
 *   - ISR and Task safe (auto-detects context via IPSR register)
 *   - Idle line detection for efficient line-by-line input
 *   - Auto callback registration (no manual ISR code needed)
 *
 * Configuration (stm32zero-conf.h):
 *   - STM32ZERO_STDIN_UART: UART handle name (required, e.g., huart3)
 *   - STM32ZERO_STDIN_BUFFER_SIZE: Ring buffer size in bytes (default: 256)
 *   - STM32ZERO_STDIN_DMA_SIZE: DMA buffer size in bytes (default: 64)
 *
 * Requirements:
 *   - Link stm32zero-sin.cpp to your project
 *   - UART with DMA RX configured in STM32CubeMX
 *   - USE_HAL_UART_REGISTER_CALLBACKS=1 in stm32h7xx_hal_conf.h
 *
 * STM32CubeMX DMA Settings:
 *   - HAL: Enable both DMA and DMA interrupt (NVIC)
 *   - Enable UART global interrupt for IDLE detection
 *
 * Usage:
 *   // In main.c after MX_USARTx_UART_Init():
 *   stm32zero::sin::init();
 *
 *   // Read received data:
 *   char buf[64];
 *   int n = stm32zero::sin::read(buf, sizeof(buf));
 *
 *   // Or use scanf/fgets via newlib:
 *   scanf("%d", &value);
 */

#include <cstdint>
#include <cstddef>

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
#include "semphr.h"
#endif

namespace stm32zero {
namespace sin {

/**
 * Initialize stdin module
 *
 * Registers UART RxEvent callback automatically.
 * Starts DMA reception with idle line detection.
 * Call after MX_USARTx_UART_Init().
 *
 * Requires USE_HAL_UART_REGISTER_CALLBACKS=1 in stm32h7xx_hal_conf.h
 */
void init();

/**
 * Read data from stdin buffer
 *
 * ISR-safe: auto-detects context and uses appropriate critical section.
 * Can be called from both Task and ISR context.
 * Non-blocking: returns immediately if no data available.
 *
 * @param data Pointer to destination buffer
 * @param len Maximum bytes to read
 * @return Number of bytes actually read (0 if buffer empty)
 */
int read(void* data, size_t len);

/**
 * Wait until data is available
 *
 * FreeRTOS: blocks on semaphore
 * Bare-metal: uses WFI instruction for low-power wait
 *
 * @param timeout_ms Timeout in milliseconds (default: UINT32_MAX for infinite)
 * @return true if data available, false if timeout
 */
bool wait(uint32_t timeout_ms = UINT32_MAX);

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
/**
 * Get internal semaphore handle
 *
 * For use with QueueSet or other synchronization patterns.
 * Semaphore is given from ISR when data is received.
 *
 * @return FreeRTOS semaphore handle
 */
SemaphoreHandle_t semaphore();
#endif

/**
 * Read a line (until newline character)
 *
 * Reads characters until '\n' or '\r' is received.
 * The newline character is NOT included in the buffer.
 * Buffer is always null-terminated.
 *
 * @param buf Destination buffer
 * @param len Buffer size (including null terminator)
 * @param timeout_ms Timeout in milliseconds (default: UINT32_MAX for infinite)
 * @return Number of characters read (excluding null), -1 on timeout with no data
 */
int readln(char* buf, size_t len, uint32_t timeout_ms = UINT32_MAX);

/**
 * Get number of bytes available to read
 */
size_t available();

/**
 * Check if buffer is empty
 */
bool is_empty();

/**
 * Get high water mark (maximum buffer usage)
 */
uint16_t water_mark();

/**
 * Get buffer size
 */
size_t buffer_size();

} // namespace sin
} // namespace stm32zero

#endif // STM32ZERO_SIN_HPP
