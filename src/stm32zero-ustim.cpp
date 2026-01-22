/**
 * STM32ZERO Microsecond Timer - init implementation
 */

#include "stm32zero-ustim.hpp"

namespace stm32zero {
namespace ustim {

#ifndef NDEBUG
bool initialized_ = false;
#endif

void init()
{
	STM32ZERO_USTIM_L->CR1 |= TIM_CR1_CEN;
	STM32ZERO_USTIM_M->CR1 |= TIM_CR1_CEN;
	STM32ZERO_USTIM_H->CR1 |= TIM_CR1_CEN;

#ifndef NDEBUG
	initialized_ = true;
#endif
}

} // namespace ustim
} // namespace stm32zero
