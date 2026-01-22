/**
 * STM32ZERO Microsecond Timer - init implementation
 */

#include "stm32zero-ustim.hpp"

namespace stm32zero {
namespace ustim {

void init()
{
	STM32ZERO_USTIM_L->CR1 |= TIM_CR1_CEN;
	STM32ZERO_USTIM_M->CR1 |= TIM_CR1_CEN;
	STM32ZERO_USTIM_H->CR1 |= TIM_CR1_CEN;
}

} // namespace ustim
} // namespace stm32zero
