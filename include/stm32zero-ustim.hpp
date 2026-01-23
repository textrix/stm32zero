#ifndef STM32ZERO_USTIM_HPP
#define STM32ZERO_USTIM_HPP

/**
 * STM32ZERO Microsecond Timer - 48-bit lock-free counter
 *
 * Features:
 *   - 48-bit microsecond counter using 3x 16-bit cascaded timers
 *   - Lock-free read (safe from any context including ISR)
 *   - Inline implementation for zero function call overhead
 *   - Range: ~8.9 years before overflow
 *
 * Hardware setup (STM32CubeMX):
 *   - TIM3 (L): Master, 1MHz clock (prescaler = APB_freq/1MHz - 1)
 *   - TIM4 (M): Slave of TIM3 overflow (ITR2), external clock mode
 *   - TIM12 (H): Slave of TIM4 overflow (ITR0), external clock mode
 *
 * Configuration (stm32zero-conf.h):
 *   - STM32ZERO_USTIM_L: Low timer instance (default: TIM3)
 *   - STM32ZERO_USTIM_M: Middle timer instance (default: TIM4)
 *   - STM32ZERO_USTIM_H: High timer instance (default: TIM12)
 *
 * Usage:
 *   stm32zero::ustim::init();  // once at startup
 *
 *   uint64_t start = stm32zero::ustim::get();
 *   // ... do work ...
 *   uint64_t elapsed = stm32zero::ustim::get() - start;
 */

#include "main.h"
#include <cstdint>
#include <cassert>

//=============================================================================
// Configuration
//=============================================================================

#if __has_include("stm32zero-conf.h")
#include "stm32zero-conf.h"
#endif

#ifndef STM32ZERO_USTIM_L
#define STM32ZERO_USTIM_L	TIM3
#endif

#ifndef STM32ZERO_USTIM_M
#define STM32ZERO_USTIM_M	TIM4
#endif

#ifndef STM32ZERO_USTIM_H
#define STM32ZERO_USTIM_H	TIM12
#endif

namespace stm32zero {
namespace ustim {

#ifndef NDEBUG
extern bool initialized_;
#endif

/**
 * Initialize microsecond timer
 *
 * Starts the cascaded timers. Call once after timer peripheral init.
 * Timers must be pre-configured in STM32CubeMX:
 *   - TIM3: Master, 1MHz, Update event as TRGO
 *   - TIM4: Slave (ITR2), External Clock Mode 1
 *   - TIM12: Slave (ITR0), External Clock Mode 1
 */
void init();

/**
 * Get current 48-bit microsecond count
 *
 * Lock-free: safe to call from any context (task, ISR, critical section)
 * Inline: zero function call overhead
 *
 * Algorithm:
 *   1. Read H, M, L in sequence
 *   2. Re-read L and M to detect overflow
 *   3. If L or M wrapped, retry
 *
 * @return Microseconds since init (48-bit value in uint64_t)
 */
__STATIC_FORCEINLINE uint64_t get()
{
	assert(initialized_ && "ustim::init() must be called before get()");

	uint16_t h, m1, m2, l1, l2;

	do {
		h  = STM32ZERO_USTIM_H->CNT;
		m1 = STM32ZERO_USTIM_M->CNT;
		l1 = STM32ZERO_USTIM_L->CNT;
		l2 = STM32ZERO_USTIM_L->CNT;
		m2 = STM32ZERO_USTIM_M->CNT;
	} while (l2 < l1 || m2 < m1);

	return (static_cast<uint64_t>(h) << 32) | (static_cast<uint64_t>(m2) << 16) | static_cast<uint64_t>(l2);
}

/**
 * Get elapsed time in microseconds
 *
 * @param start Previous value from get()
 * @return Elapsed microseconds (handles 48-bit wrap correctly)
 */
__STATIC_FORCEINLINE uint64_t elapsed(uint64_t start)
{
	constexpr uint64_t mask = (1ULL << 48) - 1;
	return (get() - start) & mask;
}

} // namespace ustim
} // namespace stm32zero

#endif // STM32ZERO_USTIM_HPP
