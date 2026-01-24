#ifndef STM32ZERO_UART_HPP
#define STM32ZERO_UART_HPP

/**
 * STM32ZERO UART Module - Multi-UART support with DMA
 *
 * For the default/primary UART, use stm32zero-sio.hpp instead.
 * This module is for defining additional UART instances.
 *
 * Features:
 *   - Multiple UART instances with independent buffers
 *   - DMA-based TX (dual buffer) and RX (ring buffer + idle detection)
 *   - Thread-safe with mutex protection
 *   - Blocking readln with timeout
 *   - write()/read() based I/O (no printf, no newlib)
 *
 * Usage:
 *   // In source file (.cpp) - define uart instance
 *   STM32ZERO_DEFINE_UART(gps, huart2, 128, 512);         // rx_dma = 128
 *   STM32ZERO_DEFINE_UART(gps, huart2, 128, 512, 64);     // rx_dma = 64
 *
 *   // Initialize after MX_USARTx_UART_Init()
 *   STM32ZERO_INIT_UART(gps, huart2);
 *
 *   // Use
 *   gps.write(cmd, len);
 *   gps.readln(buf, sizeof(buf), 1000);
 */

#include <cstdint>
#include <cstddef>

#include "stm32zero.hpp"

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
#include "stm32zero-freertos.hpp"
#endif

namespace stm32zero {
namespace uart {

//=============================================================================
// RingBuffer (for RX)
//=============================================================================

class RingBuffer {
public:
	RingBuffer() = default;

	void init(volatile uint8_t* buf, size_t size);

	size_t push(const uint8_t* data, size_t len);
	size_t pop(uint8_t* data, size_t len);
	size_t pop_until(uint8_t* data, size_t max_len, uint8_t delimiter, bool* found);

	size_t available() const;
	bool is_empty() const;
	size_t size() const { return size_; }
	uint16_t water_mark() const { return water_mark_; }

private:
	bool is_empty_locked() const;
	bool is_full_locked() const;
	uint16_t used_locked() const;
	size_t find_locked(uint8_t delimiter) const;
	size_t pop_locked(uint8_t* data, size_t len);

	volatile uint8_t* buffer_ = nullptr;
	size_t size_ = 0;
	volatile uint16_t head_ = 0;
	volatile uint16_t tail_ = 0;
	volatile uint16_t water_mark_ = 0;
};

//=============================================================================
// DualBuffer (for TX)
//=============================================================================

class DualBuffer {
public:
	using TxStartFunc = void (*)(void* ctx, const uint8_t* data, uint16_t len);

	DualBuffer() = default;

	void init(volatile uint8_t* buf0, volatile uint8_t* buf1, size_t size);
	void set_tx_callback(TxStartFunc func, void* ctx);

	int write(const void* data, size_t len);
	void tx_complete_isr();
	bool flush();

	bool is_busy() const { return tx_busy_; }
	uint16_t pending() const { return fill_pos_[fill_idx_]; }
	uint16_t water_mark() const;
	size_t size() const { return size_; }

private:
	bool start_dma_locked();

	volatile uint8_t* buffers_[2] = {nullptr, nullptr};
	size_t size_ = 0;
	volatile uint16_t fill_pos_[2] = {0, 0};
	volatile uint8_t fill_idx_ = 0;
	volatile bool tx_busy_ = false;
	volatile uint16_t water_mark_[2] = {0, 0};

	TxStartFunc tx_func_ = nullptr;
	void* tx_ctx_ = nullptr;
};

//=============================================================================
// Uart
//=============================================================================

class Uart {
public:
	Uart() = default;

	void init(UART_HandleTypeDef* huart,
		  RingBuffer* rx_buf,
		  DualBuffer* tx_buf,
		  volatile uint8_t* rx_dma,
		  size_t rx_dma_size);

	int write(const void* data, size_t len);
	int read(void* data, size_t len);
	int read(void* data, size_t len, uint32_t timeout_ms);

	bool wait(uint32_t timeout_ms);
	int readln(char* buf, size_t len, uint32_t timeout_ms);

	bool flush();
	size_t available();
	bool is_empty();
	bool is_tx_busy();
	uint16_t pending();
	uint16_t rx_water_mark();
	uint16_t tx_water_mark();

	UART_HandleTypeDef* handle() const { return huart_; }

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	SemaphoreHandle_t semaphore() { return rx_sem_.handle(); }
#endif

	// ISR callbacks (called internally)
	void rx_event_isr(uint16_t size);
	void tx_complete_isr();

private:
	static void tx_start_callback(void* ctx, const uint8_t* data, uint16_t len);

	UART_HandleTypeDef* huart_ = nullptr;
	RingBuffer* rx_buf_ = nullptr;
	DualBuffer* tx_buf_ = nullptr;
	volatile uint8_t* rx_dma_ = nullptr;
	size_t rx_dma_size_ = 0;

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	freertos::StaticBinarySemaphore rx_sem_;
	freertos::StaticMutex tx_mutex_;
#endif
};

} // namespace uart
} // namespace stm32zero

//=============================================================================
// STM32ZERO_DEFINE_UART Macro
//=============================================================================

/**
 * Define a uart instance with proper memory sections
 *
 * IMPORTANT: Use in source file (.cpp) only, not in headers.
 *
 * @param name         Instance name (used as variable name)
 * @param huart        UART handle (e.g., huart2)
 * @param tx_size      TX dual buffer size in bytes (each buffer)
 * @param rx_size      RX ring buffer size in bytes
 * @param rx_dma_size  (optional) RX DMA buffer size, default = rx_size / 4
 *
 * Usage:
 *   STM32ZERO_DEFINE_UART(gps, huart2, 128, 512);        // rx_dma = 128
 *   STM32ZERO_DEFINE_UART(gps, huart2, 128, 512, 64);    // rx_dma = 64
 *   STM32ZERO_INIT_UART(gps, huart2);
 *   gps.write(cmd, len);
 */
#define STM32ZERO_DEFINE_UART_IMPL_(name, huart, tx_size, rx_size, rx_dma_size) \
	STM32ZERO_DMA_RX static stm32zero::DmaBuffer<rx_dma_size> name##_rx_dma_; \
	STM32ZERO_DMA_TX static stm32zero::DmaBuffer<tx_size> name##_tx_dma0_; \
	STM32ZERO_DMA_TX static stm32zero::DmaBuffer<tx_size> name##_tx_dma1_; \
	STM32ZERO_DTCM static uint8_t name##_rx_buf_storage_[rx_size]; \
	STM32ZERO_DTCM static stm32zero::uart::RingBuffer name##_rx_buf_; \
	STM32ZERO_DTCM static stm32zero::uart::DualBuffer name##_tx_buf_; \
	static stm32zero::uart::Uart name; \
	static constexpr size_t name##_rx_dma_size_ = rx_dma_size; \
	static struct name##_init_t_ { \
		name##_init_t_() { \
			name##_rx_buf_.init(name##_rx_buf_storage_, rx_size); \
			name##_tx_buf_.init(name##_tx_dma0_.data(), name##_tx_dma1_.data(), tx_size); \
		} \
	} name##_auto_init_

#define STM32ZERO_DEFINE_UART_4_(n, h, t, r)       STM32ZERO_DEFINE_UART_IMPL_(n, h, t, r, (r) / 4)
#define STM32ZERO_DEFINE_UART_5_(n, h, t, r, d)    STM32ZERO_DEFINE_UART_IMPL_(n, h, t, r, d)
#define STM32ZERO_GET_UART_MACRO_(_1, _2, _3, _4, _5, NAME, ...) NAME

#define STM32ZERO_DEFINE_UART(...) \
	STM32ZERO_GET_UART_MACRO_(__VA_ARGS__, STM32ZERO_DEFINE_UART_5_, STM32ZERO_DEFINE_UART_4_)(__VA_ARGS__)

#define STM32ZERO_INIT_UART(name, huart) \
	name.init(&huart, &name##_rx_buf_, &name##_tx_buf_, name##_rx_dma_.data(), name##_rx_dma_size_)

#endif // STM32ZERO_UART_HPP
