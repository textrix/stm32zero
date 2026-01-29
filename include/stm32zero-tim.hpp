#ifndef STM32ZERO_TIM_HPP_
#define STM32ZERO_TIM_HPP_

/**
 * STM32ZERO Timer Metadata Template
 *
 * Provides compile-time timer information (bits, pointer).
 * Uses TIMx_BASE addresses for constexpr 32-bit detection
 * (equivalent to IS_TIM_32B_COUNTER_INSTANCE but constexpr).
 *
 * Usage:
 *   constexpr int bits = stm32zero::TIM<2>::bits;  // 32 or 16
 *   TIM_TypeDef* tim = stm32zero::TIM<2>::ptr();
 */

#include "main.h"
#include <cstdint>

namespace stm32zero {

//=============================================================================
// 32-bit timer detection (constexpr version of IS_TIM_32B_COUNTER_INSTANCE)
//=============================================================================

namespace detail {

constexpr bool is_tim_32b_counter(uintptr_t base)
{
	return false
#ifdef TIM2_BASE
	       || (base == TIM2_BASE)
#endif
#ifdef TIM5_BASE
	       || (base == TIM5_BASE)
#endif
	       ;
}

} // namespace detail

//=============================================================================
// Timer metadata template
//=============================================================================

template<int N>
struct TIM;

//=============================================================================
// Timer specialization macro
//=============================================================================

#define STM32ZERO_DEFINE_TIM(n) \
template<> \
struct TIM<n> { \
	static constexpr int bits = detail::is_tim_32b_counter(TIM##n##_BASE) ? 32 : 16; \
	static TIM_TypeDef* ptr() { return TIM##n; } \
}

#ifdef TIM1
STM32ZERO_DEFINE_TIM(1);
#endif
#ifdef TIM2
STM32ZERO_DEFINE_TIM(2);
#endif
#ifdef TIM3
STM32ZERO_DEFINE_TIM(3);
#endif
#ifdef TIM4
STM32ZERO_DEFINE_TIM(4);
#endif
#ifdef TIM5
STM32ZERO_DEFINE_TIM(5);
#endif
#ifdef TIM6
STM32ZERO_DEFINE_TIM(6);
#endif
#ifdef TIM7
STM32ZERO_DEFINE_TIM(7);
#endif
#ifdef TIM8
STM32ZERO_DEFINE_TIM(8);
#endif
#ifdef TIM12
STM32ZERO_DEFINE_TIM(12);
#endif
#ifdef TIM13
STM32ZERO_DEFINE_TIM(13);
#endif
#ifdef TIM14
STM32ZERO_DEFINE_TIM(14);
#endif
#ifdef TIM15
STM32ZERO_DEFINE_TIM(15);
#endif
#ifdef TIM16
STM32ZERO_DEFINE_TIM(16);
#endif
#ifdef TIM17
STM32ZERO_DEFINE_TIM(17);
#endif

#undef STM32ZERO_DEFINE_TIM

} // namespace stm32zero

#endif // STM32ZERO_TIM_HPP_
