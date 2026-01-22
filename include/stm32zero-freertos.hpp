#ifndef STM32ZERO_FREERTOS_HPP
#define STM32ZERO_FREERTOS_HPP

/**
 * STM32ZERO FreeRTOS Utilities
 *
 * Features:
 *   - CMSIS-RTOS v1/v2 compatible priority enum
 *   - Static task creation with zero-overhead wrapper
 *   - Static queue with type-safe operations
 *
 * Requirements:
 *   - FreeRTOS with CMSIS-RTOS wrapper
 *   - Include after cmsis_os.h or cmsis_os2.h
 *
 * Usage:
 *   #include "stm32zero-freertos.hpp"
 *   using namespace stm32zero::freertos;
 *
 *   StaticTask<256> task;  // 256 words = 1024 bytes on 32-bit
 *   task.create(myFunc, "Task", Priority::NORMAL);
 *
 *   StaticQueue<sizeof(Message), 10> queue;
 *   queue.create();
 */

#include <cstdint>
#include <cstddef>
#include <type_traits>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

namespace stm32zero {
namespace freertos {

//=============================================================================
// Task Priority (CMSIS-RTOS compatible)
//=============================================================================

/**
 * Task priority levels for FreeRTOS
 *
 * Values auto-select based on CMSIS-RTOS version:
 *   - CMSIS-RTOS v2: 1, 8, 16, 24, 32, 40, 48
 *   - CMSIS-RTOS v1 / FreeRTOS native: 0, 1, 2, 3, 4, 5, 6
 *
 * Usage:
 *     task.create(func, "name", Priority::NORMAL);
 *     xTaskCreate(func, "name", 128, NULL, +Priority::NORMAL, NULL);
 */
enum class Priority : UBaseType_t {
#if defined(osCMSIS) && (osCMSIS >= 0x20000U)
	// CMSIS-RTOS v2
	IDLE         = 1,
	LOW          = 8,
	BELOW_NORMAL = 16,
	NORMAL       = 24,
	ABOVE_NORMAL = 32,
	HIGH         = 40,
	REALTIME     = 48
#else
	// CMSIS-RTOS v1 / FreeRTOS native
	IDLE         = 0,
	LOW          = 1,
	BELOW_NORMAL = 2,
	NORMAL       = 3,
	ABOVE_NORMAL = 4,
	HIGH         = 5,
	REALTIME     = 6
#endif
};

/**
 * Convert Priority to UBaseType_t
 *
 * Usage:
 *     +Priority::NORMAL  // returns native priority value
 */
constexpr UBaseType_t operator+(Priority p) noexcept
{
	return static_cast<UBaseType_t>(p);
}

//=============================================================================
// Type Aliases
//=============================================================================

using TaskHandle = TaskHandle_t;
using QueueHandle = QueueHandle_t;

//=============================================================================
// Static Task
//=============================================================================

/**
 * Static FreeRTOS task with compile-time stack size
 *
 * @tparam StackWords Stack size in words (same unit as xTaskCreate)
 *
 * Usage:
 *   StaticTask<256> task;  // 256 words = 1024 bytes on 32-bit
 *
 *   void taskFunc(void* param) {
 *       for (;;) { ... }
 *   }
 *
 *   task.create(taskFunc, "MyTask", Priority::NORMAL);
 *   task.create(taskFunc, "MyTask", Priority::NORMAL, &myData);
 *
 *   // Access FreeRTOS handle for native API
 *   vTaskSuspend(task.handle());
 */
template<size_t StackWords>
class StaticTask {
public:
	static_assert(StackWords >= configMINIMAL_STACK_SIZE,
		"Stack size must be at least configMINIMAL_STACK_SIZE");

	constexpr StaticTask() = default;

	// Non-copyable, non-movable
	StaticTask(const StaticTask&) = delete;
	StaticTask& operator=(const StaticTask&) = delete;
	StaticTask(StaticTask&&) = delete;
	StaticTask& operator=(StaticTask&&) = delete;

	/**
	 * Create task with void* parameter
	 */
	TaskHandle_t create(TaskFunction_t func,
			    const char* name,
			    Priority priority,
			    void* param = nullptr)
	{
		handle_ = xTaskCreateStatic(
			func, name, StackWords, param, +priority,
			stack_, &tcb_
		);
		return handle_;
	}

	/**
	 * Create task with typed pointer parameter
	 */
	template<typename T>
	TaskHandle_t create(void (*func)(T*),
			    const char* name,
			    Priority priority,
			    T* param)
	{
		handle_ = xTaskCreateStatic(
			reinterpret_cast<TaskFunction_t>(func),
			name, StackWords, static_cast<void*>(param), +priority,
			stack_, &tcb_
		);
		return handle_;
	}

	/**
	 * Create task with integral parameter
	 */
	template<typename T>
	typename std::enable_if<std::is_integral<T>::value, TaskHandle_t>::type
	create(void (*func)(T),
	       const char* name,
	       Priority priority,
	       T param)
	{
		handle_ = xTaskCreateStatic(
			reinterpret_cast<TaskFunction_t>(func),
			name, StackWords,
			reinterpret_cast<void*>(static_cast<uintptr_t>(param)), +priority,
			stack_, &tcb_
		);
		return handle_;
	}

	/** Get task handle for FreeRTOS API */
	TaskHandle_t handle() const { return handle_; }

	/** Check if task is created */
	bool is_created() const { return handle_ != nullptr; }

	/** Explicit conversion to TaskHandle_t */
	explicit operator TaskHandle_t() const { return handle_; }

	static constexpr size_t stack_words() { return StackWords; }
	static constexpr size_t stack_bytes() { return StackWords * sizeof(StackType_t); }

private:
	StaticTask_t tcb_{};
	StackType_t stack_[StackWords]{};
	TaskHandle_t handle_ = nullptr;
};

//=============================================================================
// Static Queue
//=============================================================================

/**
 * Static FreeRTOS queue with compile-time item size and length
 *
 * @tparam ItemSize Size of each item in bytes
 * @tparam Length Maximum number of items in queue
 *
 * Usage:
 *   struct Message { int type; int data; };
 *   StaticQueue<sizeof(Message), 10> queue;  // 10 messages
 *   queue.create();
 *
 *   Message msg = {1, 42};
 *   queue.send(&msg);
 *   queue.receive(&msg);
 *
 *   // From ISR
 *   queue.send_from_isr(&msg);
 */
template<size_t ItemSize, size_t Length>
class StaticQueue {
public:
	static_assert(ItemSize > 0, "ItemSize must be greater than 0");
	static_assert(Length > 0, "Length must be greater than 0");

	constexpr StaticQueue() = default;

	// Non-copyable, non-movable
	StaticQueue(const StaticQueue&) = delete;
	StaticQueue& operator=(const StaticQueue&) = delete;
	StaticQueue(StaticQueue&&) = delete;
	StaticQueue& operator=(StaticQueue&&) = delete;

	/**
	 * Create the queue
	 */
	QueueHandle_t create()
	{
		handle_ = xQueueCreateStatic(
			Length, ItemSize,
			storage_, &tcb_
		);
		return handle_;
	}

	// -------------------------------------------------------------------------
	// Send operations
	// -------------------------------------------------------------------------

	/**
	 * Send item to queue (blocking)
	 * @param item Pointer to item
	 * @param timeout Ticks to wait (default: forever)
	 * @return true if sent
	 */
	bool send(const void* item, TickType_t timeout = portMAX_DELAY)
	{
		return xQueueSend(handle_, item, timeout) == pdTRUE;
	}

	/**
	 * Send item to front of queue
	 */
	bool send_to_front(const void* item, TickType_t timeout = portMAX_DELAY)
	{
		return xQueueSendToFront(handle_, item, timeout) == pdTRUE;
	}

	/**
	 * Send item from ISR
	 */
	bool send_from_isr(const void* item)
	{
		BaseType_t woken = pdFALSE;
		BaseType_t result = xQueueSendFromISR(handle_, item, &woken);
		portYIELD_FROM_ISR(woken);
		return result == pdTRUE;
	}

	/**
	 * Send item to front from ISR
	 */
	bool send_to_front_from_isr(const void* item)
	{
		BaseType_t woken = pdFALSE;
		BaseType_t result = xQueueSendToFrontFromISR(handle_, item, &woken);
		portYIELD_FROM_ISR(woken);
		return result == pdTRUE;
	}

	// -------------------------------------------------------------------------
	// Receive operations
	// -------------------------------------------------------------------------

	/**
	 * Receive item from queue (blocking)
	 * @param item Pointer to buffer for received item
	 * @param timeout Ticks to wait (default: forever)
	 * @return true if received
	 */
	bool receive(void* item, TickType_t timeout = portMAX_DELAY)
	{
		return xQueueReceive(handle_, item, timeout) == pdTRUE;
	}

	/**
	 * Peek item without removing from queue
	 */
	bool peek(void* item, TickType_t timeout = 0)
	{
		return xQueuePeek(handle_, item, timeout) == pdTRUE;
	}

	/**
	 * Receive item from ISR
	 */
	bool receive_from_isr(void* item)
	{
		BaseType_t woken = pdFALSE;
		BaseType_t result = xQueueReceiveFromISR(handle_, item, &woken);
		portYIELD_FROM_ISR(woken);
		return result == pdTRUE;
	}

	// -------------------------------------------------------------------------
	// Query operations
	// -------------------------------------------------------------------------

	/** Get number of items in queue */
	size_t count() const
	{
		return static_cast<size_t>(uxQueueMessagesWaiting(handle_));
	}

	/** Get number of free slots */
	size_t available() const
	{
		return static_cast<size_t>(uxQueueSpacesAvailable(handle_));
	}

	/** Check if queue is empty */
	bool is_empty() const { return count() == 0; }

	/** Check if queue is full */
	bool is_full() const { return available() == 0; }

	/** Reset queue to empty state */
	void reset() { xQueueReset(handle_); }

	/** Get queue handle for FreeRTOS API */
	QueueHandle_t handle() const { return handle_; }

	/** Check if queue is created */
	bool is_created() const { return handle_ != nullptr; }

	/** Explicit conversion to QueueHandle_t */
	explicit operator QueueHandle_t() const { return handle_; }

	static constexpr size_t item_size() { return ItemSize; }
	static constexpr size_t length() { return Length; }

private:
	StaticQueue_t tcb_{};
	uint8_t storage_[ItemSize * Length]{};
	QueueHandle_t handle_ = nullptr;
};

//=============================================================================
// Utility Functions
//=============================================================================

/**
 * Delay for specified ticks
 */
inline void delay(TickType_t ticks)
{
	vTaskDelay(ticks);
}

/**
 * Delay until specified wake time (for periodic tasks)
 */
inline void delay_until(TickType_t* prev_wake, TickType_t period)
{
	vTaskDelayUntil(prev_wake, period);
}

/**
 * Yield to other tasks of same priority
 */
inline void yield()
{
	taskYIELD();
}

/**
 * Get current tick count
 */
inline TickType_t get_tick_count()
{
	return xTaskGetTickCount();
}

} // namespace freertos
} // namespace stm32zero

#endif // STM32ZERO_FREERTOS_HPP
