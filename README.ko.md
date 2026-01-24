# STM32ZERO

STM32를 위한 Zero-Overhead C++ 유틸리티 라이브러리

[English](README.md)

## 특징

- **Zero-overhead**: 템플릿, constexpr, 인라인 함수 적극 활용
- **런타임 비용 없음**: 가상 함수, RTTI, 힙 할당, 동적 다형성 사용 안함
- **Modern Embedded C++**: 임베디드에 최적화된 C 스타일 C++
- **정적 할당**: 모든 리소스 컴파일 타임에 정적 할당

## 모듈

### Core (`stm32zero.hpp`)

임베디드 개발을 위한 기본 유틸리티:

- `dimof(arr)` - 컴파일 타임 배열 크기 (`sizeof(arr)/sizeof(arr[0])`의 타입 안전 대안)
- `is_power_of_2(x)` - 컴파일 타임 2의 거듭제곱 검사
- `align_up<N>(value)` - N 바이트 경계로 정렬
- `cache_align(value)` - 캐시 라인 경계로 정렬 (STM32H7: 32 바이트)
- `is_in_isr()` - Cortex-M IPSR 레지스터로 ISR 컨텍스트 감지
- `DmaBuffer<Size>` - 크기 검증이 포함된 캐시 정렬 DMA 버퍼
- `CriticalSection` - 인터럽트 안전 크리티컬 섹션용 RAII 가드
- `wait_until(cond, timeout)` - WFI 기반 저전력 블로킹 대기 (non-RTOS 전용)

**섹션 배치 매크로:**
- `STM32ZERO_ITCM` - ITCM RAM에 코드 배치 (빠른 실행)
- `STM32ZERO_DTCM` - DTCM RAM에 데이터 배치 (빠른 접근, 캐시 없음)
- `STM32ZERO_DMA` / `STM32ZERO_DMA_TX` / `STM32ZERO_DMA_RX` - DMA 안전 메모리 영역

### Serial I/O (`stm32zero-sio.hpp`)

듀얼 버퍼 TX와 링 버퍼 RX 기반 DMA UART 입출력:

- ISR과 Task 모두 안전 (컨텍스트 자동 감지)
- TX: Lock-free 이중 버퍼링, 블로킹 없음
- RX: Idle line 감지 기반 링 버퍼
- 타임아웃과 `\r\n` 처리가 포함된 `readln()`
- write()/read() 기반 I/O (printf 없음, newlib 없음)

```cpp
#include "stm32zero-sio.hpp"

// MX_USARTx_UART_Init() 이후
stm32zero::sio::init();

// 쓰기 (어디서든 동작 - task 또는 ISR)
sio::write("Hello\r\n", 7);

// 타임아웃 있는 읽기
char buf[128];
int n = sio::readln(buf, sizeof(buf), 1000);
```

**설계 노트:**

- **최대 처리량을 위한 DMA**: 논블로킹 고속 전송을 위해 HAL + DMA 사용
- **의도적 printf 제거**: 정적 메모리만 사용하기 위해 제거 (zero-overhead 원칙)
- **newlib hook 미구현**: newlib 의존성 방지를 위해 `_write`/`_read` 의도적 미구현

**로깅 사용시 주의:**

- **논블로킹 쓰기**: TX 버퍼가 가득 차면 데이터가 **잘림** (대기하지 않음)
- `tx_water_mark()`로 버퍼 사용량을 모니터링하여 오버플로우 감지
- 잘림이 자주 발생하면 `STM32ZERO_SIO_TX_SIZE` 증가 필요

```cpp
// Check buffer usage after heavy logging
uint16_t wm = sio::tx_water_mark();
uint16_t size = 4096;  // STM32ZERO_SIO_TX_SIZE
if (wm > size * 80 / 100) {
    // Warning: buffer usage exceeded 80%
}
```

### UART (`stm32zero-uart.hpp`)

추가 시리얼 포트를 위한 다중 인스턴스 UART 지원:

```cpp
#include "stm32zero-uart.hpp"

// 버퍼 크기와 함께 uart 인스턴스 정의
STM32ZERO_DEFINE_UART(gps, huart2, 128, 512);

// MX_USARTx_UART_Init() 이후
STM32ZERO_INIT_UART(gps, huart2);

// 사용
gps.write("$PMTK...\r\n", 10);
gps.readln(buf, sizeof(buf), 1000);
```

### Microsecond Timer (`stm32zero-ustim.hpp`)

캐스케이드 연결된 타이머를 사용한 48비트 lock-free 마이크로초 카운터:

- 2-timer (32+16 또는 16+32) 및 3-timer (16+16+16) 모드 지원
- Lock-free 읽기 (모든 컨텍스트에서 안전)
- 인라인 구현 (함수 호출 오버헤드 제로)
- 범위: 오버플로우까지 약 8.9년

```cpp
#include "stm32zero-ustim.hpp"

stm32zero::ustim::init();

uint64_t start = stm32zero::ustim::get();
// ... 작업 수행 ...
uint64_t elapsed_us = stm32zero::ustim::elapsed(start);
```

### FreeRTOS (`stm32zero-freertos.hpp`)

Zero-overhead FreeRTOS 래퍼:

**Priority Enum** (CMSIS-RTOS v1/v2 호환):
```cpp
using namespace stm32zero::freertos;

// 네이티브 FreeRTOS와 CMSIS-RTOS 모두 호환
task.create(func, "Task", Priority::NORMAL);
xTaskCreate(func, "Task", 128, NULL, +Priority::HIGH, NULL);
```

**Static Task** (힙 할당 제로):
```cpp
StaticTask<256> task;  // 256 words = 1024 bytes (32비트 기준)

task.create(myFunc, "MyTask", Priority::NORMAL);
task.create(myFunc, "MyTask", Priority::HIGH, &userData);
```

**Static Queue**:
```cpp
StaticQueue<sizeof(Message), 10> queue;  // 10개 메시지
queue.create();

Message msg = {1, 42};
queue.send(&msg);
queue.receive(&msg);
queue.send_from_isr(&msg);  // ISR 안전
```

**Static Mutex & Semaphores**:
```cpp
StaticMutex mutex;
mutex.create();
{ MutexLock lock(mutex); /* 보호된 코드 */ }

StaticBinarySemaphore sem;
sem.create();
sem.take(pdMS_TO_TICKS(1000));  // 타임아웃과 함께 대기
sem.give_from_isr();            // ISR에서 시그널
```

## 요구사항

- C++17 이상
- STM32 HAL 드라이버
- ARM Cortex-M 프로세서

## 설치

git 서브모듈로 추가:

```bash
git submodule add https://github.com/textrix/stm32zero.git STM32ZERO
```

Include 경로: `STM32ZERO/include`

## 설정

프로젝트 include 경로에 `stm32zero-conf.h` 생성 (선택사항):

```c
// Serial I/O 모듈 UART 핸들
#define STM32ZERO_SIO_UART      huart3
#define STM32ZERO_SIO_RX_SIZE   256
#define STM32ZERO_SIO_TX_SIZE   4096
#define STM32ZERO_SIO_DMA_SIZE  64

// 마이크로초 타이머 (타이머 번호, 핸들이 아님)
// 2-timer 모드 (32+16 또는 16+32):
#define STM32ZERO_USTIM_LOW     5
#define STM32ZERO_USTIM_HIGH    8
// 또는 3-timer 모드 (16+16+16):
#define STM32ZERO_USTIM_LOW     3
#define STM32ZERO_USTIM_MID     4
#define STM32ZERO_USTIM_HIGH    12

// FreeRTOS 지원
#define STM32ZERO_RTOS_FREERTOS 1

// 캐시 라인 크기 오버라이드 (미정의시 자동 감지)
#define STM32ZERO_CACHE_LINE_SIZE 32
```

## 코드 스타일

- **들여쓰기**: 탭 (8칸 너비)
- **네이밍**: 함수/변수는 `snake_case`, 클래스/enum은 `PascalCase`
- **Enum 값**: 대문자 (`Priority::NORMAL`, `Priority::HIGH`)

## 라이선스

MIT License
