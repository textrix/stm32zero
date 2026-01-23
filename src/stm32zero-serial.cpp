/**
 * STM32ZERO Serial Module Implementation
 */

#include "main.h"
#include "stm32zero-serial.hpp"
#include <cstring>

namespace stm32zero {
namespace serial {

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

			if (!tx_busy_) {
				start_dma_locked();
			}
		}
	}

	return written;
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
// Serial Implementation
//=============================================================================

// UART -> Serial mapping table
static constexpr size_t MAX_SERIAL_INSTANCES = 8;
static Serial* serial_instances_[MAX_SERIAL_INSTANCES] = {nullptr};
static UART_HandleTypeDef* uart_handles_[MAX_SERIAL_INSTANCES] = {nullptr};
static size_t serial_count_ = 0;

static Serial* find_serial(UART_HandleTypeDef* huart)
{
	for (size_t i = 0; i < serial_count_; i++) {
		if (uart_handles_[i] == huart) {
			return serial_instances_[i];
		}
	}
	return nullptr;
}

static void register_serial(UART_HandleTypeDef* huart, Serial* serial)
{
	if (serial_count_ < MAX_SERIAL_INSTANCES) {
		uart_handles_[serial_count_] = huart;
		serial_instances_[serial_count_] = serial;
		serial_count_++;
	}
}

static void rx_event_callback_static(UART_HandleTypeDef* huart, uint16_t size)
{
	auto* self = find_serial(huart);
	if (self) {
		self->rx_event_isr(size);
	}
}

static void tx_complete_callback_static(UART_HandleTypeDef* huart)
{
	auto* self = find_serial(huart);
	if (self) {
		self->tx_complete_isr();
	}
}

void Serial::init(UART_HandleTypeDef* huart,
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
	register_serial(huart, this);

	HAL_UART_RegisterRxEventCallback(huart_, rx_event_callback_static);
	HAL_UART_RegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID, tx_complete_callback_static);

	// Start RX DMA
	HAL_UARTEx_ReceiveToIdle_DMA(huart_, const_cast<uint8_t*>(rx_dma_), rx_dma_size_);
}

void Serial::tx_start_callback(void* ctx, const uint8_t* data, uint16_t len)
{
	auto* self = static_cast<Serial*>(ctx);
	HAL_UART_Transmit_DMA(self->huart_, const_cast<uint8_t*>(data), len);
}

void Serial::rx_event_isr(uint16_t size)
{
	rx_buf_->push(const_cast<uint8_t*>(rx_dma_), size);

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	rx_sem_.give_from_isr();
#endif

	HAL_UARTEx_ReceiveToIdle_DMA(huart_, const_cast<uint8_t*>(rx_dma_), rx_dma_size_);
}

void Serial::tx_complete_isr()
{
	tx_buf_->tx_complete_isr();
}

int Serial::write(const void* data, size_t len)
{
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	freertos::MutexLock lock(tx_mutex_);
#endif
	return tx_buf_->write(data, len);
}

int Serial::read(void* data, size_t len)
{
	return static_cast<int>(rx_buf_->pop(static_cast<uint8_t*>(data), len));
}

bool Serial::wait(uint32_t timeout_ms)
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

int Serial::readline(char* buf, size_t len, uint32_t timeout_ms)
{
	if (buf == nullptr || len == 0) {
		return -1;
	}

	size_t pos = 0;
	size_t max_chars = len - 1;

	while (pos < max_chars) {
		if (!wait(timeout_ms)) {
			if (pos == 0) {
				buf[0] = '\0';
				return -1;
			}
			break;
		}

		bool found = false;
		size_t n = rx_buf_->pop_until(
			reinterpret_cast<uint8_t*>(buf + pos),
			max_chars - pos,
			'\n',
			&found
		);

		pos += n;

		if (found) {
			while (pos > 0 && (buf[pos - 1] == '\n' || buf[pos - 1] == '\r')) {
				pos--;
			}
			break;
		}
	}

	buf[pos] = '\0';
	return static_cast<int>(pos);
}

size_t Serial::available()
{
	return rx_buf_->available();
}

bool Serial::is_empty()
{
	return rx_buf_->is_empty();
}

bool Serial::is_tx_busy()
{
	return tx_buf_->is_busy();
}

} // namespace serial
} // namespace stm32zero
