#ifndef STM32ZERO_HPP
#define STM32ZERO_HPP

#include <cstdint>
#include <cstddef>

//=============================================================================
// Configuration
//=============================================================================

#if __has_include("stm32zero-conf.h")
#include "stm32zero-conf.h"
#endif

//=============================================================================
// FreeRTOS (optional)
//=============================================================================

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
#include "FreeRTOS.h"
#include "task.h"
#endif

namespace stm32zero {

//=============================================================================
// Array size
//=============================================================================

/**
 * Returns the number of elements in an array at compile time
 *
 * @tparam T Array element type
 * @tparam N Array size
 * @return Number of elements in the array
 *
 * Usage:
 *     int arr[10];
 *     constexpr auto n = dimof(arr);  // 10
 *
 *     int arr2[3][4][5];
 *     dimof(arr2)         // 3
 *     dimof(arr2[0])      // 4
 *     dimof(arr2[0][0])   // 5
 *
 * Passing a pointer causes compile error:
 *     int* ptr = arr;
 *     dimof(ptr);  // error!
 */
template<typename T, size_t N>
constexpr size_t dimof(const T (&)[N]) noexcept
{
	return N;
}

//=============================================================================
// Power of 2
//=============================================================================

/**
 * Check if a value is a power of 2 at compile time
 *
 * @tparam T Integer type
 * @param x Value to check
 * @return true if x is a power of 2, false otherwise
 *
 * Usage:
 *     static_assert(is_power_of_2(64), "must be power of 2");
 *
 *     is_power_of_2(0)   // false
 *     is_power_of_2(1)   // true  (2^0)
 *     is_power_of_2(2)   // true  (2^1)
 *     is_power_of_2(3)   // false
 *     is_power_of_2(4)   // true  (2^2)
 *     is_power_of_2(64)  // true  (2^6)
 */
template<typename T>
constexpr bool is_power_of_2(T x) noexcept
{
	return (x != 0) && ((x & (x - 1)) == 0);
}

//=============================================================================
// Alignment
//=============================================================================

/**
 * Round up a value to the specified alignment
 *
 * @tparam T Value type (must be integral)
 * @tparam Alignment Alignment boundary (must be power of 2)
 * @param value Value to align
 * @return Value rounded up to the nearest multiple of Alignment
 *
 * Usage:
 *     align_up<32>(100)  // 128
 *     align_up<32>(128)  // 128
 *     align_up<32>(1)    // 32
 *
 *     align_up<7>(100)   // compile error! 7 is not power of 2
 */
template<size_t Alignment, typename T>
constexpr T align_up(T value) noexcept
{
	static_assert(is_power_of_2(Alignment), "Alignment must be power of 2");
	return (value + Alignment - 1) & ~(Alignment - 1);
}

/**
 * Round up a value to the specified alignment (runtime version)
 *
 * @tparam T Value type (must be integral)
 * @param value Value to align
 * @param alignment Alignment boundary (must be power of 2)
 * @return Value rounded up to the nearest multiple of alignment
 *
 * Note: Prefer the template version align_up<N>() when alignment is known
 *       at compile time for better error checking.
 *
 * Usage:
 *     align_up(100, 32)  // 128
 */
template<typename T>
constexpr T align_up(T value, size_t alignment) noexcept
{
	return (value + alignment - 1) & ~(alignment - 1);
}

/**
 * D-Cache line size in bytes
 *
 * Priority:
 *   1. STM32ZERO_CACHE_LINE_SIZE (user defined in stm32zero-conf.h)
 *   2. __SCB_DCACHE_LINE_SIZE (CMSIS provided)
 *   3. 0 (no cache)
 */
#if defined(STM32ZERO_CACHE_LINE_SIZE)
constexpr size_t cache_line_size = STM32ZERO_CACHE_LINE_SIZE;
#elif defined(__SCB_DCACHE_LINE_SIZE)
constexpr size_t cache_line_size = __SCB_DCACHE_LINE_SIZE;
#else
constexpr size_t cache_line_size = 0;
#endif

static_assert(cache_line_size == 0 || is_power_of_2(cache_line_size),
	"cache_line_size must be 0 or power of 2");

/**
 * Round up a value to cache line boundary
 *
 * @tparam T Value type (must be integral)
 * @param value Value to align
 * @return Value rounded up to the nearest cache line boundary,
 *         or unchanged if cache_line_size is 0
 *
 * Usage:
 *     // STM32H7 (cache_line_size = 32)
 *     cache_align(100)  // 128
 *     cache_align(32)   // 32
 *     cache_align(1)    // 32
 *
 *     // STM32F4 (cache_line_size = 0)
 *     cache_align(100)  // 100 (unchanged)
 */
template<typename T>
constexpr T cache_align(T value) noexcept
{
	if constexpr (cache_line_size == 0) {
		return value;
	} else {
		return align_up<cache_line_size>(value);
	}
}

//=============================================================================
// Section attributes (multi-compiler support)
//=============================================================================

// Section placement - not provided by CMSIS
#if defined(__ICCARM__)
  // IAR Compiler
  #if (__VER__ >= 8000000)
    // IAR v8+ supports __attribute__
    #define STM32ZERO_SECTION(s)  __attribute__((section(s)))
  #else
    // IAR v7 and earlier: use @ operator
    #define STM32ZERO_SECTION(s)  @ s
  #endif
#else
  // GCC, ARM Compiler 5, ARM Compiler 6 (armclang)
  #define STM32ZERO_SECTION(s)  __attribute__((section(s)))
#endif

#ifndef STM32ZERO_ITCM
#define STM32ZERO_ITCM        STM32ZERO_SECTION(".itcmram")
#endif

#ifndef STM32ZERO_DTCM
#define STM32ZERO_DTCM        STM32ZERO_SECTION(".dtcmram")
#endif

#ifndef STM32ZERO_DTCM_DATA
#define STM32ZERO_DTCM_DATA   STM32ZERO_SECTION(".dtcmram_data")
#endif

#ifndef STM32ZERO_DMA
#define STM32ZERO_DMA         STM32ZERO_SECTION(".dma")
#endif

#ifndef STM32ZERO_DMA_TX
#define STM32ZERO_DMA_TX      STM32ZERO_SECTION(".dma_tx")
#endif

#ifndef STM32ZERO_DMA_RX
#define STM32ZERO_DMA_RX      STM32ZERO_SECTION(".dma_rx")
#endif

//=============================================================================
// DMA Buffer
//=============================================================================

/**
 * Cache-aligned DMA buffer with size validation
 *
 * @tparam Size Actual buffer size in bytes
 *
 * Features:
 *   - Cache line aligned (or unaligned if cache_line_size == 0)
 *   - Actual size and aligned size separated via union
 *   - Initializer list size validation (warns if exceeds Size)
 *   - operator[] for direct access
 *
 * Usage:
 *     STM32ZERO_DMA_TX DmaBuffer<64> tx_buf;
 *     STM32ZERO_DMA_RX DmaBuffer<128> rx_buf = {{ 0 }};
 *
 *     tx_buf[0] = 0x55;
 *     HAL_SPI_Transmit_DMA(&hspi, tx_buf.data(), tx_buf.size());
 *
 * Memory layout (cache_line_size = 32, Size = 50):
 *     |-- data_[50] --|
 *     |-- aligned_[64] ----------------------|
 *     ^-- aligned to 32 bytes
 */
template<size_t Size>
class DmaBuffer {
	static_assert(Size > 0, "Size must be greater than 0");

	static constexpr size_t aligned_size_ =
		(cache_line_size > 0) ? align_up<cache_line_size>(Size) : Size;

	union {
		volatile uint8_t data_[Size];
		volatile uint8_t aligned_[aligned_size_];
	};

public:
	volatile uint8_t& operator[](size_t i) { return data_[i]; }
	volatile uint8_t operator[](size_t i) const { return data_[i]; }

	volatile uint8_t* data() { return data_; }
	const volatile uint8_t* data() const { return data_; }

	static constexpr size_t size() { return Size; }
	static constexpr size_t aligned_size() { return aligned_size_; }
};

//=============================================================================
// ISR detection (Cortex-M)
//=============================================================================

/**
 * Check if currently executing in ISR context
 *
 * Uses Cortex-M IPSR register:
 *   - IPSR == 0: Thread mode (normal code)
 *   - IPSR != 0: Handler mode (ISR)
 *
 * @return true if in ISR, false otherwise
 */
static inline bool is_in_isr()
{
#if defined(__CORTEX_M)
	return (__get_IPSR() != 0);
#else
	return false;
#endif
}

//=============================================================================
// Critical Section (RAII)
//=============================================================================

/**
 * RAII guard for critical section
 *
 * Automatically detects ISR/Task context and uses appropriate API:
 *   - FreeRTOS: taskENTER_CRITICAL / taskENTER_CRITICAL_FROM_ISR
 *   - Bare-metal: PRIMASK-based interrupt disable/enable
 *
 * Usage:
 *   void some_function() {
 *       CriticalSection cs;  // auto enter
 *       // ... critical section code ...
 *   }  // auto exit
 *
 *   void ISR_Handler() {
 *       CriticalSection cs;  // ISR mode auto-detected
 *       // ... critical section code ...
 *   }  // state auto-restored
 */
class CriticalSection {
public:
	CriticalSection()
	{
		in_isr_ = is_in_isr();
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
		if (in_isr_) {
			saved_state_ = taskENTER_CRITICAL_FROM_ISR();
		} else {
			taskENTER_CRITICAL();
		}
#else
		saved_state_ = __get_PRIMASK();
		__disable_irq();
#endif
	}

	~CriticalSection()
	{
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
		if (in_isr_) {
			taskEXIT_CRITICAL_FROM_ISR(saved_state_);
		} else {
			taskEXIT_CRITICAL();
		}
#else
		__set_PRIMASK(saved_state_);
#endif
	}

	// Non-copyable, non-movable
	CriticalSection(const CriticalSection&) = delete;
	CriticalSection& operator=(const CriticalSection&) = delete;
	CriticalSection(CriticalSection&&) = delete;
	CriticalSection& operator=(CriticalSection&&) = delete;

private:
	bool in_isr_;
	uint32_t saved_state_;
};

//=============================================================================
// Blocking Wait (non-RTOS only)
//=============================================================================

#if !defined(STM32ZERO_RTOS_FREERTOS) || (STM32ZERO_RTOS_FREERTOS == 0)

/**
 * Wait until condition is true or timeout (low-power)
 *
 * Uses WFI instruction between checks for power efficiency.
 * Relies on SysTick or other interrupts to wake periodically.
 *
 * @tparam Predicate Callable returning bool
 * @param cond Condition to check (returns true when satisfied)
 * @param timeout_ms Timeout in milliseconds (0 = non-blocking, returns immediately)
 * @return true if condition satisfied, false if timeout
 *
 * Usage:
 *     // Wait for flag
 *     wait_until([]{ return data_ready; }, 1000);
 *
 *     // Wait for GPIO
 *     wait_until([]{ return HAL_GPIO_ReadPin(GPIOA, PIN_0); }, 500);
 *
 *     // Non-blocking check
 *     wait_until([]{ return flag; }, 0);
 */
template<typename Predicate>
bool wait_until(Predicate cond, uint32_t timeout_ms)
{
	if (cond()) {
		return true;
	}

	if (timeout_ms == 0) {
		return false;
	}

	uint32_t start = HAL_GetTick();
	while (!cond()) {
		if ((HAL_GetTick() - start) >= timeout_ms) {
			return false;
		}
		__WFI();
	}
	return true;
}

#endif // !STM32ZERO_RTOS_FREERTOS

} // namespace stm32zero

#endif // STM32ZERO_HPP
