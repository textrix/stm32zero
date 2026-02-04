/**
 * STM32ZERO Configuration File
 *
 * Copy this file to your project's include directory as "stm32zero-conf.h"
 * and modify the settings as needed.
 *
 * Include path priority:
 *   1. Project include directory (where your stm32zero-conf.h resides)
 *   2. STM32ZERO/include directory
 */

#ifndef __STM32ZERO_CONF_H__
#define __STM32ZERO_CONF_H__

//=============================================================================
// Memory Section Attributes
//=============================================================================

/**
 * Override default section names if needed.
 * STM32ZERO_SECTION() macro is available for use here.
 *
 * Defaults:
 *   STM32ZERO_ITCM      -> .itcmram
 *   STM32ZERO_DTCM      -> .dtcmram       (zero-initialized)
 *   STM32ZERO_DTCM_DATA -> .dtcmram_data  (initialized from Flash)
 *   STM32ZERO_DMA       -> .dma
 *   STM32ZERO_DMA_TX    -> .dma_tx
 *   STM32ZERO_DMA_RX    -> .dma_rx
 */
// #define STM32ZERO_ITCM       STM32ZERO_SECTION(".itcmram")
// #define STM32ZERO_DTCM       STM32ZERO_SECTION(".dtcmram")
// #define STM32ZERO_DTCM_DATA  STM32ZERO_SECTION(".dtcmram_data")
// #define STM32ZERO_DMA        STM32ZERO_SECTION(".sram4")
// #define STM32ZERO_DMA_TX     STM32ZERO_SECTION(".sram4_tx")
// #define STM32ZERO_DMA_RX     STM32ZERO_SECTION(".sram4_rx")

//=============================================================================
// Assert Configuration
//=============================================================================

/**
 * Disable STM32ZERO assert handler
 *
 * By default, STM32ZERO provides __assert and __assert_func implementations
 * for newlib's assert.h, outputting assertion info via Serial I/O UART.
 *
 * Define this macro if you:
 *   - Use another library that provides assert handlers
 *   - Want to implement your own custom assert handler
 *   - Want to use the default newlib assert behavior
 *
 * Default: disabled (STM32ZERO provides assert handlers)
 */
// #define STM32ZERO_NO_ASSERT  1

//=============================================================================
// Serial I/O Configuration (sio module)
//=============================================================================

/**
 * UART number for standard I/O (required)
 *
 * Specify the UART peripheral number (1-8). This automatically generates:
 *   - STM32ZERO_SIO_HANDLE: HAL handle name (e.g., huart3)
 *   - STM32ZERO_SIO_REG: Register base (e.g., USART3 or UART3)
 *
 * Example: For USART3/huart3, set STM32ZERO_SIO_NUM to 3
 *
 * STM32CubeMX Settings:
 *   - Enable UART with DMA RX and DMA TX
 *   - Enable UART global interrupt (NVIC) for IDLE detection
 *   - Enable USE_HAL_UART_REGISTER_CALLBACKS=1 in stm32h7xx_hal_conf.h
 */
#define STM32ZERO_SIO_NUM  3

/**
 * RX ring buffer size in bytes
 *
 * Holds received data until read by the application.
 * Placed in .dtcmram section for fast access.
 *
 * Default: 256
 */
// #define STM32ZERO_SIO_RX_SIZE  256

/**
 * TX dual buffer size in bytes (each buffer)
 *
 * Total memory usage: 2 x STM32ZERO_SIO_TX_SIZE
 * Placed in .dma_tx section.
 *
 * Default: 4096
 */
// #define STM32ZERO_SIO_TX_SIZE  4096

/**
 * RX DMA buffer size in bytes
 *
 * Placed in .dma_rx section for DMA access.
 * Smaller values provide faster idle line detection response.
 *
 * Default: 64
 */
// #define STM32ZERO_SIO_DMA_SIZE  64

/**
 * Disable printf/scanf support
 *
 * By default, _write() and _read() syscalls are redirected to sio module,
 * allowing standard printf() and scanf() to work via DMA-backed serial I/O.
 *
 * Define this macro to disable stdio support:
 *   - Reduces code size (no newlib printf/scanf)
 *   - Eliminates unpredictable stack usage
 *   - No heap usage from stdio
 *
 * Default: enabled (commented out)
 */
// #define STM32ZERO_SIO_NO_STDIO  1

//=============================================================================
// UART Callback Configuration
//=============================================================================

/**
 * User-provided weak callbacks for UART
 *
 * This option is only relevant when USE_HAL_UART_REGISTER_CALLBACKS=0.
 *
 * By default (when not defined), STM32ZERO provides weak callback implementations
 * (HAL_UARTEx_RxEventCallback and HAL_UART_TxCpltCallback) that automatically
 * route events to STM32ZERO UART instances.
 *
 * Define this macro if you need to:
 *   - Handle additional UARTs not managed by STM32ZERO
 *   - Add custom processing before/after STM32ZERO handling
 *   - Integrate with other libraries that also need these callbacks
 *
 * When defined, you must provide your own weak callback implementations
 * and call the STM32ZERO processing functions:
 *
 * Example (C++ - include stm32zero-uart.hpp):
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
 *   {
 *       stm32zero::uart::process_rx_event(huart, Size);
 *       if (huart == &huart4) my_custom_rx_handler(Size);
 *   }
 *
 * Example (C - include stm32zero-callbacks.h):
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
 *   {
 *       stm32zero_uart_process_rx_event(huart, Size);
 *       if (huart == &huart4) my_custom_rx_handler(Size);
 *   }
 *
 * Default: disabled (STM32ZERO provides weak callbacks)
 */
// #define STM32ZERO_USER_UART_CALLBACKS  1

/**
 * User-provided weak callbacks for FDCAN
 *
 * This option is only relevant when USE_HAL_FDCAN_REGISTER_CALLBACKS=0.
 *
 * By default (when not defined), STM32ZERO provides weak callback implementations
 * (HAL_FDCAN_RxFifo0Callback, HAL_FDCAN_TxBufferCompleteCallback, etc.) that
 * automatically route events to STM32ZERO FDCAN instances.
 *
 * Define this macro if you need to:
 *   - Handle additional FDCAN peripherals not managed by STM32ZERO
 *   - Add custom processing before/after STM32ZERO handling
 *   - Integrate with other libraries that also need these callbacks
 *
 * When defined, you must provide your own weak callback implementations
 * and call the STM32ZERO processing functions:
 *
 * Example (C++ - include stm32zero-fdcan.hpp):
 *   void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
 *   {
 *       stm32zero::fdcan::process_rx_fifo0(hfdcan, RxFifo0ITs);
 *       if (hfdcan == &hfdcan2) my_custom_handler();
 *   }
 *
 * Example (C - include stm32zero-callbacks.h):
 *   void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
 *   {
 *       stm32zero_fdcan_process_rx_fifo0(hfdcan, RxFifo0ITs);
 *   }
 *   void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t BufferIndexes)
 *   {
 *       stm32zero_fdcan_process_tx_complete(hfdcan, BufferIndexes);
 *   }
 *   void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef* hfdcan)
 *   {
 *       stm32zero_fdcan_process_tx_fifo_empty(hfdcan);
 *   }
 *   void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
 *   {
 *       stm32zero_fdcan_process_error(hfdcan, ErrorStatusITs);
 *   }
 *
 * Default: disabled (STM32ZERO provides weak callbacks)
 */
// #define STM32ZERO_USER_FDCAN_CALLBACKS  1

//=============================================================================
// RTOS Selection
//=============================================================================

/**
 * Select one RTOS (set to 1 to enable)
 */
// #define STM32ZERO_RTOS_FREERTOS  1
// #define STM32ZERO_RTOS_THREADX   1

//=============================================================================
// Namespace Alias
//=============================================================================

/**
 * Short namespace alias for convenience.
 * If defined, allows using the alias instead of 'stm32zero'.
 *
 * Example:
 *   #define STM32ZERO_NAMESPACE_ALIAS zero
 *
 *   // Then use:
 *   zero::sio::write("Hello", 5);
 *   zero::ustim::get();
 */
// #define STM32ZERO_NAMESPACE_ALIAS zero

#endif // __STM32ZERO_CONF_H__
