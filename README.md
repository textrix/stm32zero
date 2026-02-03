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

// Formatted write (caller provides buffer)
char buf[128];
sio::writef(buf, "Value: %d\r\n", 42);           // array reference (size auto-deduced)
sio::writef(buf, sizeof(buf), "Value: %d\r\n", 42);  // pointer + explicit size

// Read with timeout
int n = sio::readln(buf, sizeof(buf), 1000);
```

**Design Notes:**

- **DMA for maximum throughput**: Uses HAL with DMA for non-blocking, high-speed transfers
- **No printf by design**: Removed to ensure static-only memory allocation (zero-overhead principle)
- **No newlib hooks**: `_write`/`_read` are intentionally NOT implemented to avoid newlib dependency

**Important for logging:**

- **Non-blocking write**: If TX buffer is full, data is **truncated** (not waiting)
- Use `write_peak()` to monitor buffer usage and detect overflow conditions
- Increase `STM32ZERO_SIO_TX_SIZE` if truncation occurs frequently

```cpp
// Check buffer usage after heavy logging
uint16_t peak = sio::write_peak();
uint16_t size = 4096;  // STM32ZERO_SIO_TX_SIZE
if (peak > size * 80 / 100) {
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

48-bit lock-free microsecond counter using cascaded timers:

- Supports 2-timer (32+16 or 16+32) and 3-timer (16+16+16) modes
- Lock-free reads (safe from any context)
- Inline implementation (zero call overhead)
- Range: ~8.9 years before overflow

```cpp
#include "stm32zero-ustim.hpp"

stm32zero::ustim::init();

uint64_t start = stm32zero::ustim::get();
// ... do work ...
uint64_t elapsed_us = stm32zero::ustim::elapsed(start);
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

### FDCAN (`stm32zero-fdcan.hpp`)

CAN-FD support with static allocation:

```cpp
#include "stm32zero-fdcan.hpp"

STM32ZERO_DEFINE_FDCAN(can1, hfdcan1, 16);

// After MX_FDCAN1_Init()
STM32ZERO_INIT_FDCAN(can1, hfdcan1);

can1.set_format(fdcan::FrameFormat::FD_BRS);
can1.set_nominal(fdcan::Bitrate::K500);
can1.set_data(fdcan::Bitrate::M2);
can1.set_filter_range(0x100, 0x1FF);
can1.open();

can1.write(0x100, data, 8, 1000);           // Standard ID
can1.write_ext(0x18DAF110, data, 8, 1000);  // Extended ID

fdcan::RxMessage msg;
can1.read(&msg, 1000);

// TX/RX status check
if (can1.writable()) { ... }   // TX FIFO has free slot
if (can1.readable()) { ... }   // RX message available

// Wait functions
can1.wait_writable(1000);      // Wait until TX possible
can1.wait_readable(1000);      // Wait until RX available
can1.flush(1000);              // Wait for all TX to complete
can1.purge();                  // Clear RX buffer
```

**H7/H5 Portability:**

By default, Message RAM configuration uses STM32CubeMX settings as-is (works for both H7 and H5 series). To override MX settings programmatically, define `FDCAN_OVERRIDE_MX_CONFIG=1`.

**Clock Configuration (Code Size Optimization):**

By default, FDCAN clock is auto-detected via `HAL_RCCEx_GetPeriphCLKFreq()`, which adds ~2-3KB to code size. To avoid this, define the clock frequency in `stm32zero-conf.h`:

```c
// Value from CubeMX .ioc file: RCC.FDCANFreq_Value
#define STM32ZERO_FDCAN_CLOCK_HZ  80000000UL
```

In Debug builds, the macro value is verified against the actual clock to catch CubeMX configuration changes.

**Hardware Note:**

CAN transceiver's STBY (standby) pin must be set **LOW** for the transceiver to be active. If STBY pin is floating or HIGH, the transceiver enters standby mode and CAN communication will fail with protocol errors (PEA/ARA in IR register).

**Low-Power Wake-up (Remote Wake via RXD):**

Transceivers like MCP2561/2FD support low-power standby mode with bus activity monitoring:

- In Standby mode (STBY=HIGH), transmitter and high-speed receiver are disabled to minimize power consumption
- Low-power receiver and wake-up filter remain active to monitor the bus
- RXD shows a **delayed** representation of the CAN bus (due to wake-up filter)
- Dominant state on CAN bus causes **negative edge** on RXD pin, interrupting the MCU
- MCU must set STBY=LOW to return to Normal mode for high-speed communication

```cpp
// Example: Configure RXD pin (PD0) as EXTI input for wake-up
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(EXTI0_IRQn);

// In EXTI callback: wake up and activate transceiver
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);  // Activate
    }
}
```

**STM32 Low-Power Mode Considerations:**

- **Stop mode**: Any GPIO can wake via EXTI. Clear pending interrupts before entering (`NVIC->ICPR[n]`), suspend SysTick (`HAL_SuspendTick()`). PRIMASK (`__disable_irq()`) blocks handler execution but EXTI still wakes WFI.
- **Standby mode**: Only dedicated WKUP pins can wake (e.g., PA0=WKUP1, PA2=WKUP2 on STM32H7). Route RXD to WKUP pin if needed.

## Requirements

- C++17 or later
- STM32 HAL Drivers
- ARM Cortex-M processor

### Why C++17?

STM32ZERO uses C++17 features for cleaner, more maintainable code:

- **`if constexpr`**: Compile-time branching without template specialization boilerplate
- **`inline` variables**: Header-only library support without separate .cpp definitions

C++14 alternatives exist but add unnecessary complexity. Modern ARM GCC toolchains (including STM32CubeCLT) fully support C++17.

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

// Microsecond timer (timer numbers, not handles)
// 2-timer mode (32+16 or 16+32):
#define STM32ZERO_USTIM_LOW     5
#define STM32ZERO_USTIM_HIGH    8
// Or 3-timer mode (16+16+16):
#define STM32ZERO_USTIM_LOW     3
#define STM32ZERO_USTIM_MID     4
#define STM32ZERO_USTIM_HIGH    12

// FreeRTOS support
#define STM32ZERO_RTOS_FREERTOS 1

// FDCAN clock (from CubeMX .ioc: RCC.FDCANFreq_Value)
// Saves ~2-3KB by avoiding HAL_RCCEx_GetPeriphCLKFreq()
#define STM32ZERO_FDCAN_CLOCK_HZ  80000000UL

// Namespace alias (optional)
#define STM32ZERO_NAMESPACE_ALIAS zero
```

With namespace alias, you can use `zero::sio::write()` instead of `stm32zero::sio::write()`.

## Code Style

- **Indentation**: Tab (8 spaces width)
- **Naming**: `snake_case` for functions/variables, `PascalCase` for classes/enums
- **Enum values**: UPPERCASE (`Priority::NORMAL`, `Priority::HIGH`)

## License

MIT License
