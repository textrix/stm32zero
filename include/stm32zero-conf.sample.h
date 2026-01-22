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
 *   STM32ZERO_ITCM   -> .itcmram
 *   STM32ZERO_DTCM   -> .dtcmram
 *   STM32ZERO_DMA    -> .dma
 *   STM32ZERO_DMA_TX -> .dma_tx
 *   STM32ZERO_DMA_RX -> .dma_rx
 */
// #define STM32ZERO_ITCM    __attribute__((section(".itcmram")))
// #define STM32ZERO_DTCM    __attribute__((section(".dtcmram")))
// #define STM32ZERO_DMA     __attribute__((section(".sram4")))
// #define STM32ZERO_DMA_TX  __attribute__((section(".sram4_tx")))
// #define STM32ZERO_DMA_RX  __attribute__((section(".sram4_rx")))

//=============================================================================
// RTOS Selection
//=============================================================================

/**
 * Select one RTOS (set to 1 to enable)
 */
// #define STM32ZERO_RTOS_FREERTOS  1
// #define STM32ZERO_RTOS_THREADX   1

#endif // STM32ZERO_CONF_H
