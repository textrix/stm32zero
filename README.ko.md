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

**섹션 배치 매크로:**
- `STM32ZERO_ITCM` - ITCM RAM에 코드 배치 (빠른 실행)
- `STM32ZERO_DTCM` - DTCM RAM에 데이터 배치 (빠른 접근, 캐시 없음)
- `STM32ZERO_DMA` / `STM32ZERO_DMA_TX` / `STM32ZERO_DMA_RX` - DMA 안전 메모리 영역

### Debug (`stm32zero-debug.hpp`)

Lock-free 이중 버퍼링 기반 DMA printf 출력:

- ISR과 Task 모두 안전 (컨텍스트 자동 감지)
- 블로킹 없음 - 데이터는 버퍼에 큐잉
- 콜백 자동 등록

```cpp
#include "stm32zero-debug.hpp"

// MX_USARTx_UART_Init() 이후
stm32zero::debug::init();

// 어디서든 동작 (task 또는 ISR)
printf("Hello, World!\r\n");
```

### Microsecond Timer (`stm32zero-ustim.hpp`)

3개의 16비트 타이머를 캐스케이드 연결한 48비트 lock-free 마이크로초 카운터:

- Lock-free 읽기 (모든 컨텍스트에서 안전)
- 인라인 구현 (함수 호출 오버헤드 제로)
- 범위: 오버플로우까지 약 8.9년

```cpp
#include "stm32zero-ustim.hpp"

stm32zero::ustim::init();

uint64_t start = stm32zero::ustim::get();
// ... 작업 수행 ...
uint64_t elapsed_us = stm32zero::ustim::get() - start;
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
// Debug 모듈 UART 핸들
#define STM32ZERO_DEBUG_UART        huart3
#define STM32ZERO_DEBUG_BUFFER_SIZE 4096

// 마이크로초 타이머 인스턴스
#define STM32ZERO_USTIM_L           TIM3
#define STM32ZERO_USTIM_M           TIM4
#define STM32ZERO_USTIM_H           TIM12

// 캐시 라인 크기 오버라이드 (미정의시 자동 감지)
#define STM32ZERO_CACHE_LINE_SIZE   32
```

## 코드 스타일

- **들여쓰기**: 탭 (8칸 너비)
- **네이밍**: 함수/변수는 `snake_case`, 클래스/enum은 `PascalCase`
- **Enum 값**: 대문자 (`Priority::NORMAL`, `Priority::HIGH`)

## 라이선스

MIT License
