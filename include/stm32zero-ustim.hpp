#ifndef __STM32ZERO_USTIM_HPP__
#define __STM32ZERO_USTIM_HPP__

/**
 * STM32ZERO Microsecond Timer - 48-bit lock-free counter
 *
 * Features:
 *   - 48-bit microsecond counter using cascaded timers
 *   - Supports 2-timer (32+16 or 16+32) and 3-timer (16+16+16) modes
 *   - Lock-free read (safe from any context including ISR)
 *   - Inline implementation for zero function call overhead
 *   - Direct register access (TIM->CNT) - no HAL dependency at runtime
 *   - Range: ~8.9 years before overflow
 *
 * Hardware setup (STM32CubeMX):
 *   For 32+16 mode (2 timers):
 *     - TIM2 (Low/32-bit): Master, 1MHz clock, Update event as TRGO
 *     - TIM4 (High/16-bit): Slave, External clock mode
 *
 *   For 16+16+16 mode (3 timers):
 *     - TIM3 (L): Master, 1MHz clock, Update event as TRGO
 *     - TIM4 (M): Slave of TIM3, External clock mode, TRGO
 *     - TIM12 (H): Slave of TIM4, External clock mode
 *
 * Usage:
 *   Configure in stm32zero-conf.h:
 *     // 2-timer mode (32+16 or 16+32)
 *     #define STM32ZERO_USTIM_LOW   5
 *     #define STM32ZERO_USTIM_HIGH  8
 *
 *     // Or 3-timer mode (16+16+16)
 *     #define STM32ZERO_USTIM_LOW   3
 *     #define STM32ZERO_USTIM_MID   4
 *     #define STM32ZERO_USTIM_HIGH  12
 *
 *   Then use:
 *     stm32zero::ustim::init();
 *     uint64_t t = stm32zero::ustim::get();
 *     uint64_t elapsed = stm32zero::ustim::elapsed(start);
 *
 *   Or use template directly:
 *     using ustim_2t = stm32zero::Ustim<stm32zero::TIM<5>, stm32zero::TIM<8>>;
 *     using ustim_3t = stm32zero::Ustim<stm32zero::TIM<3>, stm32zero::TIM<4>, stm32zero::TIM<12>>;
 *     ustim_2t::init();
 *     ustim_3t::init();
 */

#include "main.h"
#include "stm32zero-tim.hpp"
#include "stm32zero.hpp"  // for CriticalSection
#include <cstdint>
#include <cassert>

#if __has_include("stm32zero-conf.h")
#include "stm32zero-conf.h"
#endif

namespace stm32zero {

//=============================================================================
// Ustim - 48-bit microsecond timer (variadic template)
//=============================================================================

/**
 * Primary template declaration (not defined)
 */
template<typename... Timers>
class Ustim;

//=============================================================================
// 2-Timer specialization (32+16 or 16+32)
//=============================================================================

template<typename LowTim, typename HighTim>
class Ustim<LowTim, HighTim> {
	static constexpr int low_bits = LowTim::bits;
	static constexpr int high_bits = HighTim::bits;
	static_assert(low_bits + high_bits == 48,
		"Total timer bits must be 48 (e.g., 32+16 or 16+32)");

public:
#ifndef NDEBUG
	static inline bool initialized_ = false;
#endif

	static void init()
	{
		LowTim::ptr()->CR1 |= TIM_CR1_CEN;
		HighTim::ptr()->CR1 |= TIM_CR1_CEN;
#ifndef NDEBUG
		initialized_ = true;
#endif
	}

	__STATIC_FORCEINLINE uint64_t get()
	{
		assert(initialized_ && "Ustim::init() must be called before get()");

		using LowCnt = decltype(LowTim::ptr()->CNT);
		using HighCnt = decltype(HighTim::ptr()->CNT);

		HighCnt h1, h2;
		LowCnt l;

		do {
			h1 = HighTim::ptr()->CNT;
			l  = LowTim::ptr()->CNT;
			h2 = HighTim::ptr()->CNT;
		} while (h1 != h2);

		return (static_cast<uint64_t>(h1) << low_bits) | l;
	}

	__STATIC_FORCEINLINE uint64_t elapsed(uint64_t start)
	{
		constexpr uint64_t mask = (1ULL << 48) - 1;
		return (get() - start) & mask;
	}

	__STATIC_FORCEINLINE void spin(uint32_t us)
	{
		uint64_t start = get();
		while (elapsed(start) < us) {}
	}

	__STATIC_FORCEINLINE void delay_us(uint32_t us)
	{
		CriticalSection cs;
		spin(us);
	}
};

//=============================================================================
// 3-Timer specialization (16+16+16)
//=============================================================================

template<typename LowTim, typename MidTim, typename HighTim>
class Ustim<LowTim, MidTim, HighTim> {
	static_assert(LowTim::bits == 16, "LowTim must be 16-bit for 3-timer mode");
	static_assert(MidTim::bits == 16, "MidTim must be 16-bit for 3-timer mode");
	static_assert(HighTim::bits == 16, "HighTim must be 16-bit for 3-timer mode");

public:
#ifndef NDEBUG
	static inline bool initialized_ = false;
#endif

	static void init()
	{
		LowTim::ptr()->CR1 |= TIM_CR1_CEN;
		MidTim::ptr()->CR1 |= TIM_CR1_CEN;
		HighTim::ptr()->CR1 |= TIM_CR1_CEN;
#ifndef NDEBUG
		initialized_ = true;
#endif
	}

	__STATIC_FORCEINLINE uint64_t get()
	{
		assert(initialized_ && "Ustim::init() must be called before get()");

		uint16_t h, m1, m2, l1, l2;

		do {
			h  = HighTim::ptr()->CNT;
			m1 = MidTim::ptr()->CNT;
			l1 = LowTim::ptr()->CNT;
			l2 = LowTim::ptr()->CNT;
			m2 = MidTim::ptr()->CNT;
		} while (l2 < l1 || m2 < m1);

		return (static_cast<uint64_t>(h) << 32) |
		       (static_cast<uint64_t>(m2) << 16) |
		       static_cast<uint64_t>(l2);
	}

	__STATIC_FORCEINLINE uint64_t elapsed(uint64_t start)
	{
		constexpr uint64_t mask = (1ULL << 48) - 1;
		return (get() - start) & mask;
	}

	__STATIC_FORCEINLINE void spin(uint32_t us)
	{
		uint64_t start = get();
		while (elapsed(start) < us) {}
	}

	__STATIC_FORCEINLINE void delay_us(uint32_t us)
	{
		CriticalSection cs;
		spin(us);
	}
};

//=============================================================================
// Automatic ustim type definition based on stm32zero-conf.h macros
//=============================================================================

#if defined(STM32ZERO_USTIM_LOW) && defined(STM32ZERO_USTIM_MID) && defined(STM32ZERO_USTIM_HIGH)
// 3-timer mode (16+16+16)
using ustim = Ustim<TIM<STM32ZERO_USTIM_LOW>, TIM<STM32ZERO_USTIM_MID>, TIM<STM32ZERO_USTIM_HIGH>>;

#elif defined(STM32ZERO_USTIM_LOW) && defined(STM32ZERO_USTIM_HIGH)
// 2-timer mode (32+16 or 16+32)
using ustim = Ustim<TIM<STM32ZERO_USTIM_LOW>, TIM<STM32ZERO_USTIM_HIGH>>;

#endif

} // namespace stm32zero

#endif // __STM32ZERO_USTIM_HPP__
