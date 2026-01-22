#ifndef STM32ZERO_DEBUG_HPP
#define STM32ZERO_DEBUG_HPP

/**
 * STM32ZERO Debug Module - DMA-based printf output
 *
 * Requirements:
 *   - Include HAL headers BEFORE this header (e.g., #include "main.h")
 *   - UART with DMA TX configured
 *   - For FreeRTOS: include FreeRTOS.h and task.h before this header
 *
 * Usage:
 *   // In one .cpp file:
 *   #include "main.h"
 *   #include "stm32zero-debug.hpp"
 *   STM32ZERO_DEBUG_DEFINE(4096);
 *
 *   void init() {
 *       stm32zero::debug::buffer.set_uart(&huart3);
 *   }
 *
 *   // In HAL_UART_TxCpltCallback:
 *   stm32zero::debug::buffer.tx_complete_isr();
 */

#include "stm32zero.hpp"
#include <cstring>

//=============================================================================
// Configuration (override in stm32zero-conf.h)
//=============================================================================

#if __has_include("stm32zero-conf.h")
#include "stm32zero-conf.h"
#endif

#ifndef STM32ZERO_DEBUG_BUFFER_SIZE
#define STM32ZERO_DEBUG_BUFFER_SIZE	4096
#endif

namespace stm32zero {
namespace debug {

//=============================================================================
// DualBuffer - Lock-free dual buffer for DMA transmission
//=============================================================================

/**
 * Lock-free dual buffer for efficient DMA-based UART output
 *
 * Design:
 *   - Two buffers alternating: one filling, one transmitting
 *   - No mutex/semaphore, uses critical sections
 *   - ISR and Task safe
 *   - Buffers are external (allocated by user with section attributes)
 *
 * @tparam Size Buffer size in bytes (each buffer)
 *
 * Usage:
 *     // Use STM32ZERO_DEBUG_DEFINE macro instead of manual instantiation
 *     STM32ZERO_DEBUG_DEFINE(4096);
 */
template<size_t Size>
class DualBuffer {
	static_assert(Size > 0, "Buffer size must be greater than 0");
	static_assert(Size <= 65535, "Buffer size must fit in uint16_t");

public:
	/**
	 * Set buffer pointers
	 *
	 * @param buf0 Pointer to first buffer
	 * @param buf1 Pointer to second buffer
	 */
	void set_buffers(volatile uint8_t* buf0, volatile uint8_t* buf1)
	{
		buffers_[0] = buf0;
		buffers_[1] = buf1;
	}

	/**
	 * Write data to the buffer
	 *
	 * Thread-safe: can be called from ISR or Task context.
	 * If buffer is full, data is truncated.
	 *
	 * @param data Pointer to data
	 * @param len Data length
	 * @return Number of bytes actually written
	 */
	int write(const void* data, size_t len)
	{
		if (data == nullptr || len == 0) {
			return 0;
		}

		int written = 0;

		enter_critical();

		uint8_t idx = fill_idx_;
		uint16_t pos = fill_pos_[idx];
		uint16_t available = Size - pos;

		uint16_t to_write = (len > available) ? available : static_cast<uint16_t>(len);

		if (to_write > 0) {
			std::memcpy(const_cast<uint8_t*>(&buffers_[idx][pos]),
				    data, to_write);

			fill_pos_[idx] = pos + to_write;
			written = to_write;

			// Update water mark
			if (fill_pos_water_mark_[idx] < fill_pos_[idx]) {
				fill_pos_water_mark_[idx] = fill_pos_[idx];
			}

			// Start DMA if not busy
			if (!tx_busy_) {
				start_dma_locked();
			}
		}

		exit_critical();

		return written;
	}

	/**
	 * DMA transfer complete callback
	 *
	 * Must be called from HAL_UART_TxCpltCallback().
	 */
	void tx_complete_isr()
	{
		enter_critical_from_isr();

		tx_busy_ = false;

		// Check if there's pending data
		if (fill_pos_[fill_idx_] > 0) {
			start_dma_locked();
		}

		exit_critical_from_isr();
	}

	/**
	 * Set UART handle for DMA transmission
	 *
	 * Must be called before any write operations.
	 *
	 * @param huart Pointer to UART handle
	 */
	void set_uart(UART_HandleTypeDef* huart)
	{
		huart_ = huart;
	}

	/**
	 * Flush pending data
	 *
	 * Starts DMA transfer if there's data and not already busy.
	 *
	 * @return true if new transfer started
	 */
	bool flush()
	{
		bool started = false;

		enter_critical();

		if (!tx_busy_ && fill_pos_[fill_idx_] > 0) {
			started = start_dma_locked();
		}

		exit_critical();

		return started;
	}

	/**
	 * Check if transmission is in progress
	 */
	bool is_busy() const { return tx_busy_; }

	/**
	 * Get pending bytes in current fill buffer
	 */
	uint16_t pending() const { return fill_pos_[fill_idx_]; }

	/**
	 * Get high water mark (maximum buffer usage)
	 */
	uint16_t water_mark() const
	{
		uint16_t wm0 = fill_pos_water_mark_[0];
		uint16_t wm1 = fill_pos_water_mark_[1];
		return (wm0 > wm1) ? wm0 : wm1;
	}

	/**
	 * Get buffer size
	 */
	static constexpr size_t size() { return Size; }

	/**
	 * Get aligned buffer size (for cache operations)
	 */
	static constexpr size_t aligned_size()
	{
		if constexpr (cache_line_size > 0) {
			return align_up<cache_line_size>(Size);
		} else {
			return Size;
		}
	}

private:
	bool start_dma_locked()
	{
		uint8_t tx_idx = fill_idx_;
		uint16_t tx_len = fill_pos_[tx_idx];

		if (tx_len == 0) {
			return false;
		}

		// Swap to other buffer
		fill_idx_ = 1 - tx_idx;
		fill_pos_[fill_idx_] = 0;

		tx_busy_ = true;

		// Cache clean for DMA
		if constexpr (cache_line_size > 0) {
			SCB_CleanDCache_by_Addr(
				reinterpret_cast<uint32_t*>(
					const_cast<uint8_t*>(buffers_[tx_idx])),
				aligned_size());
		}

		// Start HAL DMA transfer
		HAL_UART_Transmit_DMA(huart_,
			const_cast<uint8_t*>(buffers_[tx_idx]),
			tx_len);

		return true;
	}

	// Critical section helpers
	void enter_critical()
	{
#if defined(configUSE_PREEMPTION) && (configUSE_PREEMPTION == 1)
		taskENTER_CRITICAL();
#else
		primask_ = __get_PRIMASK();
		__disable_irq();
#endif
	}

	void exit_critical()
	{
#if defined(configUSE_PREEMPTION) && (configUSE_PREEMPTION == 1)
		taskEXIT_CRITICAL();
#else
		__set_PRIMASK(primask_);
#endif
	}

	void enter_critical_from_isr()
	{
#if defined(configUSE_PREEMPTION) && (configUSE_PREEMPTION == 1)
		saved_interrupt_status_ = taskENTER_CRITICAL_FROM_ISR();
#else
		primask_ = __get_PRIMASK();
		__disable_irq();
#endif
	}

	void exit_critical_from_isr()
	{
#if defined(configUSE_PREEMPTION) && (configUSE_PREEMPTION == 1)
		taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status_);
#else
		__set_PRIMASK(primask_);
#endif
	}

	// Buffer pointers (external storage)
	volatile uint8_t* buffers_[2] = {nullptr, nullptr};

	// Buffer state
	volatile uint16_t fill_pos_[2] = {0, 0};
	volatile uint8_t fill_idx_ = 0;
	volatile bool tx_busy_ = false;

	// High water marks
	volatile uint16_t fill_pos_water_mark_[2] = {0, 0};

	// UART handle
	UART_HandleTypeDef* huart_ = nullptr;

	// Critical section state
#if defined(configUSE_PREEMPTION) && (configUSE_PREEMPTION == 1)
	UBaseType_t saved_interrupt_status_ = 0;
#else
	uint32_t primask_ = 0;
#endif
};

//=============================================================================
// Global debug buffer instance (declare in user code)
//=============================================================================

/**
 * Declare global debug buffer with storage
 *
 * Usage in a .cpp file:
 *     STM32ZERO_DEBUG_DEFINE(4096);
 *
 * This creates:
 *     - Two DMA-safe buffers in .dma_tx section
 *     - stm32zero::debug::buffer (DualBuffer instance)
 *     - _write() function override for printf()
 */
#define STM32ZERO_DEBUG_DEFINE(SIZE)						\
	STM32ZERO_DMA_TX static volatile uint8_t				\
		stm32zero_debug_buf0_[stm32zero::cache_align(SIZE)]		\
		__attribute__((aligned(stm32zero::cache_line_size > 0 ?		\
			stm32zero::cache_line_size : 4)));			\
	STM32ZERO_DMA_TX static volatile uint8_t				\
		stm32zero_debug_buf1_[stm32zero::cache_align(SIZE)]		\
		__attribute__((aligned(stm32zero::cache_line_size > 0 ?		\
			stm32zero::cache_line_size : 4)));			\
	namespace stm32zero { namespace debug {					\
		DualBuffer<SIZE> buffer;					\
		static struct BufferInit_ {					\
			BufferInit_() {						\
				buffer.set_buffers(stm32zero_debug_buf0_,	\
						   stm32zero_debug_buf1_);	\
			}							\
		} buffer_init_;							\
	}}									\
	extern "C" int _write(int file, char* ptr, int len) {			\
		(void)file;							\
		return stm32zero::debug::buffer.write(ptr, len);		\
	}

/**
 * Declare extern reference to debug buffer
 *
 * Usage in header or other .cpp files:
 *     STM32ZERO_DEBUG_EXTERN(4096);
 */
#define STM32ZERO_DEBUG_EXTERN(SIZE)						\
	namespace stm32zero { namespace debug {					\
		extern DualBuffer<SIZE> buffer;					\
	}}

} // namespace debug
} // namespace stm32zero

#endif // STM32ZERO_DEBUG_HPP
