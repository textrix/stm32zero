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
// FDCAN Configuration
//=============================================================================

/**
 * FDCAN clock frequency in Hz
 *
 * Used for calculating bit timing parameters.
 * Default: 80000000 (80MHz)
 */
// #define STM32ZERO_FDCAN_CLOCK_HZ  80000000

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
