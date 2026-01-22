#ifndef STM32ZERO_FREERTOS_HPP
#define STM32ZERO_FREERTOS_HPP

/**
 * STM32ZERO FreeRTOS Utilities
 *
 * Features:
 *   - CMSIS-RTOS v1/v2 compatible priority enum
 *   - Static task creation (planned)
 *   - Static synchronization objects (planned)
 *
 * Requirements:
 *   - FreeRTOS with CMSIS-RTOS wrapper
 *   - Include after cmsis_os.h or cmsis_os2.h
 */

#include <cstdint>

namespace stm32zero {
namespace freertos {

//=============================================================================
// Task Priority (CMSIS-RTOS compatible)
//=============================================================================

/**
 * Task priority levels for FreeRTOS
 *
 * Values auto-select based on CMSIS-RTOS version:
 *   - CMSIS-RTOS v2: 1, 8, 16, 24, 32, 40, 48
 *   - CMSIS-RTOS v1 / FreeRTOS native: 0, 1, 2, 3, 4, 5, 6
 *
 * Usage with xTaskCreate:
 *     xTaskCreate(func, "name", 128, NULL, +Priority::NORMAL, NULL);
 *
 * Usage with osThreadNew (CMSIS-RTOS v2):
 *     attr.priority = (osPriority_t)+Priority::NORMAL;
 */
enum class Priority : uint32_t {
#if defined(osCMSIS) && (osCMSIS >= 0x20000U)
	// CMSIS-RTOS v2
	IDLE         = 1,
	LOW          = 8,
	BELOW_NORMAL = 16,
	NORMAL       = 24,
	ABOVE_NORMAL = 32,
	HIGH         = 40,
	REALTIME     = 48
#else
	// CMSIS-RTOS v1 / FreeRTOS native
	IDLE         = 0,
	LOW          = 1,
	BELOW_NORMAL = 2,
	NORMAL       = 3,
	ABOVE_NORMAL = 4,
	HIGH         = 5,
	REALTIME     = 6
#endif
};

/**
 * Convert Priority to underlying type (uint32_t)
 *
 * Usage:
 *     +Priority::NORMAL  // returns 24 (CMSIS v2) or 3 (CMSIS v1)
 */
constexpr uint32_t operator+(Priority p) noexcept
{
	return static_cast<uint32_t>(p);
}

} // namespace freertos
} // namespace stm32zero

#endif // STM32ZERO_FREERTOS_HPP
