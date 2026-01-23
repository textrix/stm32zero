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

#ifndef STM32ZERO_CONF_H
#define STM32ZERO_CONF_H

//=============================================================================
// Cache Configuration
//=============================================================================

/**
 * D-Cache line size in bytes
 *
 * If not defined, falls back to __SCB_DCACHE_LINE_SIZE (CMSIS),
 * or 0 if cache is not available.
 *
 * Common values:
 *   STM32H7: 32
 *   STM32F7: 32
 *   STM32F4: 0 (no cache)
 */
// #define STM32ZERO_CACHE_LINE_SIZE  32

//=============================================================================
// Memory Section Attributes
//=============================================================================

/**
 * Override default section names if needed.
 * Default values are used if not defined.
 *
 * Defaults:
 *   STM32ZERO_ITCM      -> .itcmram
 *   STM32ZERO_DTCM      -> .dtcmram       (zero-initialized)
 *   STM32ZERO_DTCM_DATA -> .dtcmram_data  (initialized from Flash)
 *   STM32ZERO_DMA       -> .dma
 *   STM32ZERO_DMA_TX    -> .dma_tx
 *   STM32ZERO_DMA_RX    -> .dma_rx
 */
// #define STM32ZERO_ITCM       __attribute__((section(".itcmram")))
// #define STM32ZERO_DTCM       __attribute__((section(".dtcmram")))
// #define STM32ZERO_DTCM_DATA  __attribute__((section(".dtcmram_data")))
// #define STM32ZERO_DMA        __attribute__((section(".sram4")))
// #define STM32ZERO_DMA_TX     __attribute__((section(".sram4_tx")))
// #define STM32ZERO_DMA_RX     __attribute__((section(".sram4_rx")))

//=============================================================================
// Serial I/O Configuration (sio module)
//=============================================================================

/**
 * UART handle for standard I/O (required)
 *
 * Shared by both stdin (sin module) and stdout (sout module).
 * Uses DMA for both RX (with idle line detection) and TX (dual buffer).
 *
 * STM32CubeMX Settings:
 *   - Enable UART with DMA RX and DMA TX
 *   - Enable UART global interrupt (NVIC) for IDLE detection
 *   - Enable USE_HAL_UART_REGISTER_CALLBACKS=1 in stm32h7xx_hal_conf.h
 */
#define STM32ZERO_SIO_UART  huart3

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

//=============================================================================
// RTOS Selection
//=============================================================================

/**
 * Select one RTOS (set to 1 to enable)
 */
// #define STM32ZERO_RTOS_FREERTOS  1
// #define STM32ZERO_RTOS_THREADX   1

#endif // STM32ZERO_CONF_H
