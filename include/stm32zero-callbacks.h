/**
 * STM32ZERO Callback API for C code
 *
 * Include this header in .c files to call STM32ZERO callback processing functions.
 * This is only needed when using STM32ZERO_USER_UART_CALLBACKS or
 * STM32ZERO_USER_FDCAN_CALLBACKS with USE_HAL_*_REGISTER_CALLBACKS=0.
 *
 * Usage (in .c file):
 *   #include "stm32zero-callbacks.h"
 *
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
 *   {
 *       stm32zero_uart_process_rx_event(huart, Size);
 *   }
 */

#ifndef STM32ZERO_CALLBACKS_H_
#define STM32ZERO_CALLBACKS_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* UART Callback API                                                         */
/*===========================================================================*/

/**
 * Process UART RX event
 *
 * Call from HAL_UARTEx_RxEventCallback() when using STM32ZERO_USER_UART_CALLBACKS.
 *
 * @param huart HAL UART handle
 * @param size  Number of bytes received
 */
void stm32zero_uart_process_rx_event(UART_HandleTypeDef* huart, uint16_t size);

/**
 * Process UART TX complete event
 *
 * Call from HAL_UART_TxCpltCallback() when using STM32ZERO_USER_UART_CALLBACKS.
 *
 * @param huart HAL UART handle
 */
void stm32zero_uart_process_tx_complete(UART_HandleTypeDef* huart);

/*===========================================================================*/
/* FDCAN Callback API                                                        */
/*===========================================================================*/

#if defined(HAL_FDCAN_MODULE_ENABLED)

/**
 * Process FDCAN RX FIFO0 event
 *
 * Call from HAL_FDCAN_RxFifo0Callback() when using STM32ZERO_USER_FDCAN_CALLBACKS.
 *
 * @param hfdcan HAL FDCAN handle
 * @param rx_fifo0_its RX FIFO0 interrupt flags
 */
void stm32zero_fdcan_process_rx_fifo0(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its);

/**
 * Process FDCAN TX buffer complete event
 *
 * Call from HAL_FDCAN_TxBufferCompleteCallback() when using STM32ZERO_USER_FDCAN_CALLBACKS.
 *
 * @param hfdcan HAL FDCAN handle
 * @param buffer_indexes Completed buffer indexes
 */
void stm32zero_fdcan_process_tx_complete(FDCAN_HandleTypeDef* hfdcan, uint32_t buffer_indexes);

/**
 * Process FDCAN TX FIFO empty event
 *
 * Call from HAL_FDCAN_TxFifoEmptyCallback() when using STM32ZERO_USER_FDCAN_CALLBACKS.
 *
 * @param hfdcan HAL FDCAN handle
 */
void stm32zero_fdcan_process_tx_fifo_empty(FDCAN_HandleTypeDef* hfdcan);

/**
 * Process FDCAN error status event
 *
 * Call from HAL_FDCAN_ErrorStatusCallback() when using STM32ZERO_USER_FDCAN_CALLBACKS.
 *
 * @param hfdcan HAL FDCAN handle
 * @param error_status_its Error status interrupt flags
 */
void stm32zero_fdcan_process_error(FDCAN_HandleTypeDef* hfdcan, uint32_t error_status_its);

#endif /* HAL_FDCAN_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* STM32ZERO_CALLBACKS_H_ */
