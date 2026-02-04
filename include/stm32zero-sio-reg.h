/**
 * STM32ZERO Serial I/O Register Definitions (C compatible)
 *
 * This header provides C-compatible macros for UART register access.
 * Used by stm32zero-sio.hpp (C++) and stm32zero-assert.c (C).
 *
 * Configuration (stm32zero-conf.h):
 *   - STM32ZERO_SIO_NUM: UART number (1-8, e.g., 3 for USART3/huart3)
 */

#ifndef STM32ZERO_SIO_REG_H_
#define STM32ZERO_SIO_REG_H_

#include "stm32zero-conf.h"

/* Token pasting macros */
#define STM32ZERO_PASTE_(a, b) a##b
#define STM32ZERO_PASTE(a, b) STM32ZERO_PASTE_(a, b)

/* Generate HAL handle name: huart1, huart2, ... huart8 */
#if defined(STM32ZERO_SIO_NUM)
  #define STM32ZERO_SIO_HANDLE  STM32ZERO_PASTE(huart, STM32ZERO_SIO_NUM)
#endif

/*
 * Generate UART/USART register base: USART1 or UART1, etc.
 * STM32 chips define either USARTx or UARTx (or both) depending on peripheral
 */
#if defined(STM32ZERO_SIO_NUM)
  #if STM32ZERO_SIO_NUM == 1
    #if defined(USART1)
      #define STM32ZERO_SIO_REG USART1
    #elif defined(UART1)
      #define STM32ZERO_SIO_REG UART1
    #endif
  #elif STM32ZERO_SIO_NUM == 2
    #if defined(USART2)
      #define STM32ZERO_SIO_REG USART2
    #elif defined(UART2)
      #define STM32ZERO_SIO_REG UART2
    #endif
  #elif STM32ZERO_SIO_NUM == 3
    #if defined(USART3)
      #define STM32ZERO_SIO_REG USART3
    #elif defined(UART3)
      #define STM32ZERO_SIO_REG UART3
    #endif
  #elif STM32ZERO_SIO_NUM == 4
    #if defined(USART4)
      #define STM32ZERO_SIO_REG USART4
    #elif defined(UART4)
      #define STM32ZERO_SIO_REG UART4
    #endif
  #elif STM32ZERO_SIO_NUM == 5
    #if defined(USART5)
      #define STM32ZERO_SIO_REG USART5
    #elif defined(UART5)
      #define STM32ZERO_SIO_REG UART5
    #endif
  #elif STM32ZERO_SIO_NUM == 6
    #if defined(USART6)
      #define STM32ZERO_SIO_REG USART6
    #elif defined(UART6)
      #define STM32ZERO_SIO_REG UART6
    #endif
  #elif STM32ZERO_SIO_NUM == 7
    #if defined(USART7)
      #define STM32ZERO_SIO_REG USART7
    #elif defined(UART7)
      #define STM32ZERO_SIO_REG UART7
    #endif
  #elif STM32ZERO_SIO_NUM == 8
    #if defined(USART8)
      #define STM32ZERO_SIO_REG USART8
    #elif defined(UART8)
      #define STM32ZERO_SIO_REG UART8
    #endif
  #else
    #error "STM32ZERO_SIO_NUM must be 1-8"
  #endif

  #if !defined(STM32ZERO_SIO_REG)
    #error "No USART/UART peripheral found for STM32ZERO_SIO_NUM"
  #endif
#endif

#endif /* STM32ZERO_SIO_REG_H_ */
