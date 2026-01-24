# STM32ZERO

Zero-Overhead C++ Utility Library for STM32

[한국어](README.ko.md)

## Features

- **Zero-overhead**: Extensive use of templates, constexpr, and inline functions
- **No runtime cost**: No virtual functions, RTTI, heap allocation, or dynamic polymorphism
- **Modern Embedded C++**: Clean C-like C++ style optimized for embedded systems
- **Static allocation**: All resources are statically allocated at compile time

## Modules

### Core (`stm32zero.hpp`)

Basic utilities for embedded development:

- `dimof(arr)` - Compile-time array size (type-safe alternative to `sizeof(arr)/sizeof(arr[0])`)
- `is_power_of_2(x)` - Compile-time power of 2 check
- `align_up<N>(value)` - Align value to N-byte boundary
- `cache_align(value)` - Align to cache line boundary (STM32H7: 32 bytes)
- `is_in_isr()` - Detect ISR context via Cortex-M IPSR register
- `DmaBuffer<Size>` - Cache-aligned DMA buffer with size validation
- `CriticalSection` - RAII guard for interrupt-safe critical sections
- `wait_until(cond, timeout)` - Low-power blocking wait with WFI (non-RTOS only)

**Section Placement Macros:**
- `STM32ZERO_ITCM` - Place code in ITCM RAM (fast execution)
- `STM32ZERO_DTCM` - Place data in DTCM RAM (fast access, no cache)
- `STM32ZERO_DMA` / `STM32ZERO_DMA_TX` / `STM32ZERO_DMA_RX` - DMA-safe memory regions

### Serial I/O (`stm32zero-sio.hpp`)

DMA-based UART input/output with dual buffer TX and ring buffer RX:

- ISR and Task safe (auto-detects context)
- TX: Lock-free dual buffering, no blocking
- RX: Ring buffer with idle line detection
- `readln()` with timeout and proper `\r\n` handling
- write()/read() based I/O (no printf, no newlib)

```cpp
#include "stm32zero-sio.hpp"

// After MX_USARTx_UART_Init()
stm32zero::sio::init();

// Write (works from anywhere - task or ISR)
sio::write("Hello\r\n", 7);

// Read with timeout
char buf[128];
int n = sio::readln(buf, sizeof(buf), 1000);
```

**Design Notes:**

- **DMA for maximum throughput**: Uses HAL with DMA for non-blocking, high-speed transfers
- **No printf by design**: Removed to ensure static-only memory allocation (zero-overhead principle)
- **No newlib hooks**: `_write`/`_read` are intentionally NOT implemented to avoid newlib dependency

**Important for logging:**

- **Non-blocking write**: If TX buffer is full, data is **truncated** (not waiting)
- Use `tx_water_mark()` to monitor buffer usage and detect overflow conditions
- Increase `STM32ZERO_SIO_TX_SIZE` if truncation occurs frequently

```cpp
// Check buffer usage after heavy logging
uint16_t wm = sio::tx_water_mark();
uint16_t size = 4096;  // STM32ZERO_SIO_TX_SIZE
if (wm > size * 80 / 100) {
    // Warning: buffer usage exceeded 80%
}
```

### UART (`stm32zero-uart.hpp`)

Multi-instance UART support for additional serial ports:

```cpp
#include "stm32zero-uart.hpp"

// Define uart instance with buffer sizes
STM32ZERO_DEFINE_UART(gps, huart2, 128, 512);

// After MX_USARTx_UART_Init()
STM32ZERO_INIT_UART(gps, huart2);

// Use
gps.write("$PMTK...\r\n", 10);
gps.readln(buf, sizeof(buf), 1000);
```

### Microsecond Timer (`stm32zero-ustim.hpp`)

48-bit lock-free microsecond counter using 3 cascaded 16-bit timers:

- Lock-free reads (safe from any context)
- Inline implementation (zero call overhead)
- Range: ~8.9 years before overflow

```cpp
#include "stm32zero-ustim.hpp"

stm32zero::ustim::init();

uint64_t start = stm32zero::ustim::get();
// ... do work ...
uint64_t elapsed_us = stm32zero::ustim::get() - start;
```

### FreeRTOS (`stm32zero-freertos.hpp`)

Zero-overhead FreeRTOS wrappers:

**Priority Enum** (CMSIS-RTOS v1/v2 compatible):
```cpp
using namespace stm32zero::freertos;

// Works with both native FreeRTOS and CMSIS-RTOS
task.create(func, "Task", Priority::NORMAL);
xTaskCreate(func, "Task", 128, NULL, +Priority::HIGH, NULL);
```

**Static Task** (zero heap allocation):
```cpp
StaticTask<256> task;  // 256 words = 1024 bytes on 32-bit

task.create(myFunc, "MyTask", Priority::NORMAL);
task.create(myFunc, "MyTask", Priority::HIGH, &userData);
```

**Static Queue**:
```cpp
StaticQueue<sizeof(Message), 10> queue;  // 10 messages
queue.create();

Message msg = {1, 42};
queue.send(&msg);
queue.receive(&msg);
queue.send_from_isr(&msg);  // ISR-safe
```

**Static Mutex & Semaphores**:
```cpp
StaticMutex mutex;
mutex.create();
{ MutexLock lock(mutex); /* protected code */ }

StaticBinarySemaphore sem;
sem.create();
sem.take(pdMS_TO_TICKS(1000));  // wait with timeout
sem.give_from_isr();            // signal from ISR
```

## Requirements

- C++17 or later
- STM32 HAL Drivers
- ARM Cortex-M processor

## Installation

Add as a git submodule:

```bash
git submodule add https://github.com/textrix/stm32zero.git STM32ZERO
```

Include path: `STM32ZERO/include`

## Configuration

Create `stm32zero-conf.h` in your include path (optional):

```c
// Serial I/O module UART handle
#define STM32ZERO_SIO_UART      huart3
#define STM32ZERO_SIO_RX_SIZE   256
#define STM32ZERO_SIO_TX_SIZE   4096
#define STM32ZERO_SIO_DMA_SIZE  64

// Microsecond timer instances
#define STM32ZERO_USTIM_L       TIM3
#define STM32ZERO_USTIM_M       TIM4
#define STM32ZERO_USTIM_H       TIM12

// FreeRTOS support
#define STM32ZERO_RTOS_FREERTOS 1

// Cache line size override (auto-detected if not defined)
#define STM32ZERO_CACHE_LINE_SIZE 32
```

## Code Style

- **Indentation**: Tab (8 spaces width)
- **Naming**: `snake_case` for functions/variables, `PascalCase` for classes/enums
- **Enum values**: UPPERCASE (`Priority::NORMAL`, `Priority::HIGH`)

## License

MIT License
