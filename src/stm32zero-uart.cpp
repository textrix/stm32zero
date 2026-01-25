/**
 * STM32ZERO UART Module Implementation
 */

#include "main.h"
#include "stm32zero-uart.hpp"
#include <cstring>

namespace stm32zero {
namespace uart {

//=============================================================================
// RingBuffer Implementation
//=============================================================================

void RingBuffer::init(volatile uint8_t* buf, size_t size)
{
	buffer_ = buf;
	size_ = size;
	head_ = 0;
	tail_ = 0;
}

size_t RingBuffer::push(const uint8_t* data, size_t len)
{
	if (data == nullptr || len == 0) {
		return 0;
	}

	CriticalSection cs;

	size_t written = 0;
	while (written < len && !is_full_locked()) {
		buffer_[head_] = data[written++];
		head_ = (head_ + 1) % size_;
	}

	uint16_t used = used_locked();
	if (used > water_mark_) {
		water_mark_ = used;
	}

	return written;
}

size_t RingBuffer::pop(uint8_t* data, size_t len)
{
	if (data == nullptr || len == 0) {
		return 0;
	}

	CriticalSection cs;

	size_t read_count = 0;
	while (read_count < len && !is_empty_locked()) {
		data[read_count++] = buffer_[tail_];
		tail_ = (tail_ + 1) % size_;
	}

	return read_count;
}

size_t RingBuffer::pop_until(uint8_t* data, size_t max_len, uint8_t delimiter, bool* found)
{
	if (data == nullptr || max_len == 0 || found == nullptr) {
		if (found) *found = false;
		return 0;
	}

	CriticalSection cs;

	size_t delim_pos = find_locked(delimiter);
	size_t avail = used_locked();

	*found = (delim_pos < avail);

	size_t to_read = *found ? (delim_pos + 1) : avail;
	if (to_read > max_len) {
		to_read = max_len;
		*found = false;
	}

	return pop_locked(data, to_read);
}

size_t RingBuffer::available() const
{
	CriticalSection cs;
	return used_locked();
}

bool RingBuffer::is_empty() const
{
	CriticalSection cs;
	return is_empty_locked();
}

bool RingBuffer::is_empty_locked() const
{
	return head_ == tail_;
}

bool RingBuffer::is_full_locked() const
{
	return ((head_ + 1) % size_) == tail_;
}

uint16_t RingBuffer::used_locked() const
{
	if (head_ >= tail_) {
		return head_ - tail_;
	} else {
		return size_ - tail_ + head_;
	}
}

size_t RingBuffer::find_locked(uint8_t delimiter) const
{
	uint16_t idx = tail_;
	size_t count = 0;
	while (idx != head_) {
		if (buffer_[idx] == delimiter) {
			return count;
		}
		idx = (idx + 1) % size_;
		count++;
	}
	return count;
}

size_t RingBuffer::pop_locked(uint8_t* data, size_t len)
{
	size_t read_count = 0;
	while (read_count < len && !is_empty_locked()) {
		data[read_count++] = buffer_[tail_];
		tail_ = (tail_ + 1) % size_;
	}
	return read_count;
}

//=============================================================================
// DualBuffer Implementation
//=============================================================================

void DualBuffer::init(volatile uint8_t* buf0, volatile uint8_t* buf1, size_t size)
{
	buffers_[0] = buf0;
	buffers_[1] = buf1;
	size_ = size;
	fill_pos_[0] = 0;
	fill_pos_[1] = 0;
	fill_idx_ = 0;
	tx_busy_ = false;
}

void DualBuffer::set_tx_callback(TxStartFunc func, void* ctx)
{
	tx_func_ = func;
	tx_ctx_ = ctx;
}

int DualBuffer::write(const void* data, size_t len)
{
	if (data == nullptr || len == 0) {
		return 0;
	}

	int written = 0;

	{
		CriticalSection cs;

		uint8_t idx = fill_idx_;
		uint16_t pos = fill_pos_[idx];
		uint16_t available = size_ - pos;

		uint16_t to_write = (len > available) ? available : static_cast<uint16_t>(len);

		if (to_write > 0) {
			std::memcpy(const_cast<uint8_t*>(&buffers_[idx][pos]), data, to_write);
			fill_pos_[idx] = pos + to_write;
			written = to_write;

			if (water_mark_[idx] < fill_pos_[idx]) {
				water_mark_[idx] = fill_pos_[idx];
			}

			if (!tx_busy_) {
				start_dma_locked();
			}
		}
	}

	return written;
}

bool DualBuffer::flush()
{
	CriticalSection cs;

	if (!tx_busy_ && fill_pos_[fill_idx_] > 0) {
		return start_dma_locked();
	}

	return false;
}

uint16_t DualBuffer::water_mark() const
{
	uint16_t wm0 = water_mark_[0];
	uint16_t wm1 = water_mark_[1];
	return (wm0 > wm1) ? wm0 : wm1;
}

void DualBuffer::tx_complete_isr()
{
	CriticalSection cs;

	tx_busy_ = false;

	if (fill_pos_[fill_idx_] > 0) {
		start_dma_locked();
	}
}

bool DualBuffer::start_dma_locked()
{
	uint8_t tx_idx = fill_idx_;
	uint16_t tx_len = fill_pos_[tx_idx];

	if (tx_len == 0 || tx_func_ == nullptr) {
		return false;
	}

	fill_idx_ = 1 - tx_idx;
	fill_pos_[fill_idx_] = 0;

	tx_busy_ = true;

	tx_func_(tx_ctx_, const_cast<uint8_t*>(buffers_[tx_idx]), tx_len);

	return true;
}

//=============================================================================
// Uart Implementation
//=============================================================================

// UART -> Uart mapping table
static constexpr size_t MAX_UART_INSTANCES = 8;
static Uart* uart_instances_[MAX_UART_INSTANCES] = {nullptr};
static UART_HandleTypeDef* uart_handles_[MAX_UART_INSTANCES] = {nullptr};
static size_t uart_count_ = 0;

static Uart* find_uart(UART_HandleTypeDef* huart)
{
	for (size_t i = 0; i < uart_count_; i++) {
		if (uart_handles_[i] == huart) {
			return uart_instances_[i];
		}
	}
	return nullptr;
}

static void register_uart(UART_HandleTypeDef* huart, Uart* uart)
{
	if (uart_count_ < MAX_UART_INSTANCES) {
		uart_handles_[uart_count_] = huart;
		uart_instances_[uart_count_] = uart;
		uart_count_++;
	}
}

static void rx_event_callback_static(UART_HandleTypeDef* huart, uint16_t size)
{
	auto* self = find_uart(huart);
	if (self) {
		self->rx_event_isr(size);
	}
}

static void tx_complete_callback_static(UART_HandleTypeDef* huart)
{
	auto* self = find_uart(huart);
	if (self) {
		self->tx_complete_isr();
	}
}

void Uart::init(UART_HandleTypeDef* huart,
		  RingBuffer* rx_buf,
		  DualBuffer* tx_buf,
		  volatile uint8_t* rx_dma,
		  size_t rx_dma_size)
{
	huart_ = huart;
	rx_buf_ = rx_buf;
	tx_buf_ = tx_buf;
	rx_dma_ = rx_dma;
	rx_dma_size_ = rx_dma_size;

	tx_buf_->set_tx_callback(tx_start_callback, this);

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	rx_sem_.create();
	tx_mutex_.create();
#endif

	// Register for callback lookup
	register_uart(huart, this);

	HAL_UART_RegisterRxEventCallback(huart_, rx_event_callback_static);
	HAL_UART_RegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID, tx_complete_callback_static);

	// Start RX DMA
	HAL_UARTEx_ReceiveToIdle_DMA(huart_, const_cast<uint8_t*>(rx_dma_), rx_dma_size_);
}

void Uart::tx_start_callback(void* ctx, const uint8_t* data, uint16_t len)
{
	auto* self = static_cast<Uart*>(ctx);
	HAL_UART_Transmit_DMA(self->huart_, const_cast<uint8_t*>(data), len);
}

void Uart::rx_event_isr(uint16_t size)
{
	rx_buf_->push(const_cast<uint8_t*>(rx_dma_), size);

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	rx_sem_.give_from_isr();
#endif

	HAL_UARTEx_ReceiveToIdle_DMA(huart_, const_cast<uint8_t*>(rx_dma_), rx_dma_size_);
}

void Uart::tx_complete_isr()
{
	tx_buf_->tx_complete_isr();
}

int Uart::write(const void* data, size_t len)
{
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	freertos::MutexLock lock(tx_mutex_);
#endif
	return tx_buf_->write(data, len);
}

int Uart::vwritef(char* buf, size_t size, const char* fmt, va_list args)
{
	int len = vsnprintf(buf, size, fmt, args);
	if (len > 0) {
		size_t n = (static_cast<size_t>(len) < size)
			 ? static_cast<size_t>(len) : size - 1;
		write(buf, n);
	}
	return len;
}

int Uart::writef(char* buf, size_t size, const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	int len = vwritef(buf, size, fmt, args);
	va_end(args);
	return len;
}

int Uart::read(void* data, size_t len)
{
	return static_cast<int>(rx_buf_->pop(static_cast<uint8_t*>(data), len));
}

int Uart::read(void* data, size_t len, uint32_t timeout_ms)
{
	if (data == nullptr || len == 0) {
		return -1;
	}

	uint8_t* ptr = static_cast<uint8_t*>(data);
	size_t received = 0;

	while (received < len) {
		if (!wait(timeout_ms)) {
			break;
		}
		received += read(ptr + received, len - received);
	}

	return static_cast<int>(received);
}

bool Uart::wait(uint32_t timeout_ms)
{
	if (!rx_buf_->is_empty()) {
		return true;
	}

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	return rx_sem_.take(pdMS_TO_TICKS(timeout_ms));
#else
	return wait_until([this]{ return !rx_buf_->is_empty(); }, timeout_ms);
#endif
}

int Uart::readln(char* buf, size_t len, uint32_t timeout_ms)
{
	if (buf == nullptr || len == 0) {
		return -1;
	}

	size_t pos = 0;
	size_t max_chars = len - 1;
	bool got_data = false;

	while (pos < max_chars) {
		if (!wait(timeout_ms)) {
			if (!got_data) {
				buf[0] = '\0';
				return -1;
			}
			break;
		}

		// Read one byte at a time
		uint8_t ch;
		size_t n = rx_buf_->pop(&ch, 1);
		if (n == 0) {
			continue;
		}

		// Skip leading CR/LF (from previous line)
		if (!got_data && (ch == '\r' || ch == '\n')) {
			continue;
		}

		// Check for line ending (CR or LF)
		if (ch == '\r' || ch == '\n') {
			break;
		}

		got_data = true;
		buf[pos++] = ch;
	}

	buf[pos] = '\0';
	return static_cast<int>(pos);
}

bool Uart::flush()
{
	return tx_buf_->flush();
}

size_t Uart::available()
{
	return rx_buf_->available();
}

bool Uart::is_empty()
{
	return rx_buf_->is_empty();
}

bool Uart::is_tx_busy()
{
	return tx_buf_->is_busy();
}

uint16_t Uart::pending()
{
	return tx_buf_->pending();
}

uint16_t Uart::rx_water_mark()
{
	return rx_buf_->water_mark();
}

uint16_t Uart::tx_water_mark()
{
	return tx_buf_->water_mark();
}

} // namespace uart
} // namespace stm32zero
