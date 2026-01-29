#ifndef STM32ZERO_FREERTOS_HPP_
#define STM32ZERO_FREERTOS_HPP_

/**
 * STM32ZERO FreeRTOS Utilities
 *
 * Features:
 *   - CMSIS-RTOS v1/v2 compatible priority enum
 *   - Static task creation with zero-overhead wrapper
 *   - Static queue with type-safe operations
 *   - Generic RAII guard: ScopedLock<T> for any lockable object
 *   - RAII mutex locks: MutexLock, RawMutexLock
 *   - Static mutex and semaphore wrappers
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
 *
 *   // RAII critical section
 *   { CriticalSection cs; ... }
 *
 *   // RAII mutex lock
 *   StaticMutex mutex; mutex.create();
 *   { MutexLock lock(mutex); ... }
 *
 *   // Raw handle mutex lock
 *   SemaphoreHandle_t raw = xSemaphoreCreateMutex();
 *   { RawMutexLock lock(raw); ... }
 */

#include <cstdint>
#include <cstddef>
#include <type_traits>

#include "stm32zero.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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

//=============================================================================
// Critical Section (from stm32zero.hpp)
//=============================================================================

// CriticalSection is defined in stm32zero.hpp
// Bring it into the freertos namespace for convenience
using stm32zero::CriticalSection;

//=============================================================================
// Static Mutex
//=============================================================================

/**
 * Static FreeRTOS mutex with RAII support
 *
 * Usage:
 *   StaticMutex g_mutex;
 *
 *   void init() {
 *       g_mutex.create();
 *   }
 *
 *   void thread_safe_function() {
 *       MutexLock lock(g_mutex);  // auto lock
 *       if (!lock) return;        // timeout check (optional)
 *       // ... protected code ...
 *   }  // auto unlock
 */
class StaticMutex {
public:
	StaticMutex() = default;

	// Non-copyable, non-movable
	StaticMutex(const StaticMutex&) = delete;
	StaticMutex& operator=(const StaticMutex&) = delete;
	StaticMutex(StaticMutex&&) = delete;
	StaticMutex& operator=(StaticMutex&&) = delete;

	SemaphoreHandle_t create()
	{
		handle_ = xSemaphoreCreateMutexStatic(&tcb_);
		return handle_;
	}

	SemaphoreHandle_t handle() const { return handle_; }
	bool is_created() const { return handle_ != nullptr; }

	bool lock(TickType_t timeout = portMAX_DELAY)
	{
		return xSemaphoreTake(handle_, timeout) == pdTRUE;
	}

	void unlock()
	{
		xSemaphoreGive(handle_);
	}

private:
	StaticSemaphore_t tcb_{};
	SemaphoreHandle_t handle_ = nullptr;
};

//=============================================================================
// ScopedLock (Generic RAII Guard)
//=============================================================================

/**
 * Generic RAII lock guard for any lockable object
 *
 * Requirements for Lockable type:
 *   - bool lock(TickType_t timeout) - acquire lock, return true if successful
 *   - void unlock() - release lock
 *
 * Usage:
 *   StaticMutex g_mutex;
 *   StaticBinarySemaphore g_sem;
 *
 *   void thread_safe_function() {
 *       ScopedLock lock(g_mutex);   // auto lock
 *       if (!lock) return;          // timeout check
 *       // ... protected code ...
 *   }  // auto unlock
 *
 *   void wait_for_event() {
 *       ScopedLock lock(g_sem, pdMS_TO_TICKS(1000));
 *       if (!lock) return;  // timeout
 *       // ... event handling ...
 *   }  // auto unlock
 */
template<typename Lockable>
class ScopedLock {
public:
	explicit ScopedLock(Lockable& obj, TickType_t timeout = portMAX_DELAY)
		: obj_(obj), locked_(obj.lock(timeout))
	{
	}

	~ScopedLock()
	{
		if (locked_) {
			obj_.unlock();
		}
	}

	bool is_locked() const { return locked_; }
	explicit operator bool() const { return locked_; }

	// Non-copyable, non-movable
	ScopedLock(const ScopedLock&) = delete;
	ScopedLock& operator=(const ScopedLock&) = delete;
	ScopedLock(ScopedLock&&) = delete;
	ScopedLock& operator=(ScopedLock&&) = delete;

private:
	Lockable& obj_;
	bool locked_;
};

/**
 * RAII guard for StaticMutex (alias for ScopedLock<StaticMutex>)
 *
 * Usage:
 *   StaticMutex g_mutex;
 *
 *   void thread_safe_function() {
 *       MutexLock lock(g_mutex);
 *       if (!lock) return;  // timeout
 *       // ... protected code ...
 *   }  // auto unlock
 */
using MutexLock = ScopedLock<StaticMutex>;

//=============================================================================
// Static Binary Semaphore
//=============================================================================

/**
 * Static FreeRTOS binary semaphore
 *
 * Usage:
 *   StaticBinarySemaphore g_sem;
 *
 *   void init() {
 *       g_sem.create();
 *   }
 *
 *   void wait_for_event() {
 *       g_sem.take(pdMS_TO_TICKS(1000));  // wait with timeout
 *   }
 *
 *   void ISR_Handler() {
 *       g_sem.give_from_isr();  // signal from ISR
 *   }
 */
class StaticBinarySemaphore {
public:
	StaticBinarySemaphore() = default;

	// Non-copyable, non-movable
	StaticBinarySemaphore(const StaticBinarySemaphore&) = delete;
	StaticBinarySemaphore& operator=(const StaticBinarySemaphore&) = delete;
	StaticBinarySemaphore(StaticBinarySemaphore&&) = delete;
	StaticBinarySemaphore& operator=(StaticBinarySemaphore&&) = delete;

	SemaphoreHandle_t create()
	{
		handle_ = xSemaphoreCreateBinaryStatic(&tcb_);
		return handle_;
	}

	SemaphoreHandle_t handle() const { return handle_; }
	bool is_created() const { return handle_ != nullptr; }

	bool take(TickType_t timeout = portMAX_DELAY)
	{
		return xSemaphoreTake(handle_, timeout) == pdTRUE;
	}

	void give()
	{
		xSemaphoreGive(handle_);
	}

	bool give_from_isr()
	{
		BaseType_t woken = pdFALSE;
		BaseType_t result = xSemaphoreGiveFromISR(handle_, &woken);
		portYIELD_FROM_ISR(woken);
		return result == pdTRUE;
	}

private:
	StaticSemaphore_t tcb_{};
	SemaphoreHandle_t handle_ = nullptr;
};

//=============================================================================
// Static Counting Semaphore
//=============================================================================

/**
 * Static FreeRTOS counting semaphore
 *
 * @tparam MaxCount Maximum semaphore count
 *
 * Usage:
 *   StaticCountingSemaphore<10> g_sem;
 *
 *   void init() {
 *       g_sem.create(5);  // initial count = 5
 *   }
 *
 *   void acquire_resource() {
 *       g_sem.take();
 *   }
 *
 *   void release_resource() {
 *       g_sem.give();
 *   }
 */
template<UBaseType_t MaxCount>
class StaticCountingSemaphore {
public:
	static_assert(MaxCount > 0, "MaxCount must be greater than 0");

	StaticCountingSemaphore() = default;

	// Non-copyable, non-movable
	StaticCountingSemaphore(const StaticCountingSemaphore&) = delete;
	StaticCountingSemaphore& operator=(const StaticCountingSemaphore&) = delete;
	StaticCountingSemaphore(StaticCountingSemaphore&&) = delete;
	StaticCountingSemaphore& operator=(StaticCountingSemaphore&&) = delete;

	SemaphoreHandle_t create(UBaseType_t initial_count = 0)
	{
		handle_ = xSemaphoreCreateCountingStatic(MaxCount, initial_count, &tcb_);
		return handle_;
	}

	SemaphoreHandle_t handle() const { return handle_; }
	bool is_created() const { return handle_ != nullptr; }

	bool take(TickType_t timeout = portMAX_DELAY)
	{
		return xSemaphoreTake(handle_, timeout) == pdTRUE;
	}

	void give()
	{
		xSemaphoreGive(handle_);
	}

	bool give_from_isr()
	{
		BaseType_t woken = pdFALSE;
		BaseType_t result = xSemaphoreGiveFromISR(handle_, &woken);
		portYIELD_FROM_ISR(woken);
		return result == pdTRUE;
	}

	UBaseType_t count() const
	{
		return uxSemaphoreGetCount(handle_);
	}

	static constexpr UBaseType_t max_count() { return MaxCount; }

private:
	StaticSemaphore_t tcb_{};
	SemaphoreHandle_t handle_ = nullptr;
};

//=============================================================================
// Raw Mutex Lock (RAII)
//=============================================================================

/**
 * RAII guard for raw SemaphoreHandle_t (mutex)
 *
 * Use this when working with raw FreeRTOS mutex handles directly.
 * For StaticMutex, prefer MutexLock (ScopedLock<StaticMutex>) instead.
 *
 * Usage:
 *   SemaphoreHandle_t raw_mutex = xSemaphoreCreateMutex();
 *
 *   void thread_safe_function() {
 *       RawMutexLock lock(raw_mutex);
 *       if (!lock) return;  // timeout
 *       // ... protected code ...
 *   }  // auto unlock
 */
class RawMutexLock {
public:
	explicit RawMutexLock(SemaphoreHandle_t mutex, TickType_t timeout = portMAX_DELAY)
		: mutex_(mutex), locked_(xSemaphoreTake(mutex, timeout) == pdTRUE)
	{
	}

	~RawMutexLock()
	{
		if (locked_) {
			xSemaphoreGive(mutex_);
		}
	}

	bool is_locked() const { return locked_; }
	explicit operator bool() const { return locked_; }

	// Non-copyable, non-movable
	RawMutexLock(const RawMutexLock&) = delete;
	RawMutexLock& operator=(const RawMutexLock&) = delete;
	RawMutexLock(RawMutexLock&&) = delete;
	RawMutexLock& operator=(RawMutexLock&&) = delete;

private:
	SemaphoreHandle_t mutex_;
	bool locked_;
};

} // namespace freertos
} // namespace stm32zero

#endif // STM32ZERO_FREERTOS_HPP_
