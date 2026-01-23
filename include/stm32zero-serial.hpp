#ifndef STM32ZERO_SERIAL_HPP
#define STM32ZERO_SERIAL_HPP

/**
 * STM32ZERO Serial Module - Multi-UART support with DMA
 *
 * Features:
 *   - Multiple UART instances with independent buffers
 *   - DMA-based TX (dual buffer) and RX (ring buffer + idle detection)
 *   - Thread-safe with mutex protection
 *   - Blocking readln with timeout
 *
 * Usage:
 *   // In source file - define serial instance
 *   DEFINE_SERIAL(debug, huart1, 256, 256, 64);
 *   DEFINE_SERIAL(gps, huart2, 512, 128, 64);
 *
 *   // Initialize after MX_USARTx_UART_Init()
 *   debug.init();
 *   gps.init();
 *
 *   // Use
 *   debug.write("Hello\n", 6);
 *   gps.readln(buf, sizeof(buf), 1000);
 */

#include <cstdint>
#include <cstddef>

#include "stm32zero.hpp"

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
#include "stm32zero-freertos.hpp"
#endif

namespace stm32zero {
namespace serial {

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

	bool is_busy() const { return tx_busy_; }
	size_t size() const { return size_; }

private:
	bool start_dma_locked();

	volatile uint8_t* buffers_[2] = {nullptr, nullptr};
	size_t size_ = 0;
	volatile uint16_t fill_pos_[2] = {0, 0};
	volatile uint8_t fill_idx_ = 0;
	volatile bool tx_busy_ = false;

	TxStartFunc tx_func_ = nullptr;
	void* tx_ctx_ = nullptr;
};

//=============================================================================
// Serial
//=============================================================================

class Serial {
public:
	Serial() = default;

	void init(UART_HandleTypeDef* huart,
		  RingBuffer* rx_buf,
		  DualBuffer* tx_buf,
		  volatile uint8_t* rx_dma,
		  size_t rx_dma_size);

	int write(const void* data, size_t len);
	int read(void* data, size_t len);

	bool wait(uint32_t timeout_ms);
	int readln(char* buf, size_t len, uint32_t timeout_ms);

	size_t available();
	bool is_empty();
	bool is_tx_busy();

	UART_HandleTypeDef* handle() const { return huart_; }

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

} // namespace serial
} // namespace stm32zero

//=============================================================================
// DEFINE_SERIAL Macro
//=============================================================================

/**
 * Define a serial instance with proper memory sections
 *
 * @param name      Instance name (used as variable name)
 * @param huart     UART handle (e.g., huart1)
 * @param rx_size   RX ring buffer size in bytes
 * @param tx_size   TX dual buffer size in bytes (each buffer)
 * @param dma_size  RX DMA buffer size in bytes
 *
 * Usage:
 *   DEFINE_SERIAL(debug, huart1, 256, 256, 64);
 *   debug.init();
 *   debug.write("Hello\n", 6);
 */
#define DEFINE_SERIAL(name, huart, rx_size, tx_size, dma_size) \
	STM32ZERO_DMA_RX static stm32zero::DmaBuffer<dma_size> name##_rx_dma_; \
	STM32ZERO_DMA_TX static stm32zero::DmaBuffer<tx_size> name##_tx_dma0_; \
	STM32ZERO_DMA_TX static stm32zero::DmaBuffer<tx_size> name##_tx_dma1_; \
	STM32ZERO_DTCM static uint8_t name##_rx_buf_storage_[rx_size]; \
	STM32ZERO_DTCM static stm32zero::serial::RingBuffer name##_rx_buf_; \
	STM32ZERO_DTCM static stm32zero::serial::DualBuffer name##_tx_buf_; \
	static stm32zero::serial::Serial name; \
	static struct name##_init_t_ { \
		name##_init_t_() { \
			name##_rx_buf_.init(name##_rx_buf_storage_, rx_size); \
			name##_tx_buf_.init(name##_tx_dma0_.data(), name##_tx_dma1_.data(), tx_size); \
		} \
	} name##_auto_init_

#define INIT_SERIAL(name, huart, dma_size) \
	name.init(&huart, &name##_rx_buf_, &name##_tx_buf_, name##_rx_dma_.data(), dma_size)

#endif // STM32ZERO_SERIAL_HPP
