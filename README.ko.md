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

// 포맷 출력 (호출자가 버퍼 제공)
char buf[128];
sio::writef(buf, "Value: %d\r\n", 42);           // 배열 참조 (크기 자동 추론)
sio::writef(buf, sizeof(buf), "Value: %d\r\n", 42);  // 포인터 + 명시적 크기

// 타임아웃 있는 읽기
int n = sio::readln(buf, sizeof(buf), 1000);
```

**설계 노트:**

- **최대 처리량을 위한 DMA**: 논블로킹 고속 전송을 위해 HAL + DMA 사용
- **의도적 printf 제거**: 정적 메모리만 사용하기 위해 제거 (zero-overhead 원칙)
- **newlib hook 미구현**: newlib 의존성 방지를 위해 `_write`/`_read` 의도적 미구현

**로깅 사용시 주의:**

- **논블로킹 쓰기**: TX 버퍼가 가득 차면 데이터가 **잘림** (대기하지 않음)
- `write_peak()`로 버퍼 사용량을 모니터링하여 오버플로우 감지
- 잘림이 자주 발생하면 `STM32ZERO_SIO_TX_SIZE` 증가 필요

```cpp
// Check buffer usage after heavy logging
uint16_t peak = sio::write_peak();
uint16_t size = 4096;  // STM32ZERO_SIO_TX_SIZE
if (peak > size * 80 / 100) {
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

### Assert (`stm32zero-assert.c`)

newlib `assert.h`를 위한 최소 assert 핸들러:

- 직접 UART 레지스터 접근 (HAL 없음, 인터럽트 없음)
- Serial I/O UART로 파일, 라인, 함수명, 표현식 출력
- 디버깅을 위해 무한 루프에서 메시지 반복
- 인터럽트 비활성화 전 적절한 메모리 배리어

```c
#include <assert.h>

// 조건이 false면 assert 핸들러 호출
assert(ptr != NULL);
assert(index < ARRAY_SIZE);
```

**출력 예시:**
```
--------------- ASSERTION FAILED ---------------

src/main.c
42
main
ptr != NULL
```

**요구사항:**
- `stm32zero-conf.h`에 `STM32ZERO_SIO_NUM` 정의 필요
- assert 출력 전 UART가 초기화되어 있어야 함
- 빌드에 `stm32zero-assert.c` 추가 필요

**비활성화:**

STM32ZERO의 assert 핸들러를 비활성화하려면 (예: 다른 라이브러리의 구현 사용 시):

```c
#define STM32ZERO_NO_ASSERT  1
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

### FDCAN (`stm32zero-fdcan.hpp`)

정적 할당 기반 CAN-FD 지원:

```cpp
#include "stm32zero-fdcan.hpp"

STM32ZERO_DEFINE_FDCAN(can1, hfdcan1, 16);

// MX_FDCAN1_Init() 이후
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

// TX/RX 상태 확인
if (can1.writable()) { ... }   // TX FIFO에 빈 슬롯 있음
if (can1.readable()) { ... }   // RX 메시지 있음

// 대기 함수
can1.wait_writable(1000);      // TX 가능할 때까지 대기
can1.wait_readable(1000);      // RX 메시지 올 때까지 대기
can1.flush(1000);              // 모든 TX 완료 대기
can1.purge();                  // RX 버퍼 비우기
```

**H7/H5 호환성:**

기본적으로 Message RAM 설정은 STM32CubeMX 설정을 그대로 사용합니다 (H7과 H5 시리즈 모두 호환). MX 설정을 코드에서 재정의하려면 `FDCAN_OVERRIDE_MX_CONFIG=1`을 정의하세요.

**클럭 설정 (코드 크기 최적화):**

기본적으로 FDCAN 클럭은 `HAL_RCCEx_GetPeriphCLKFreq()`로 자동 감지되며, 이로 인해 ~2-3KB 코드 크기가 증가합니다. 이를 피하려면 `stm32zero-conf.h`에 클럭 주파수를 정의하세요:

```c
// CubeMX .ioc 파일의 RCC.FDCANFreq_Value 값
#define STM32ZERO_FDCAN_CLOCK_HZ  80000000UL
```

Debug 빌드에서는 매크로 값이 실제 클럭과 일치하는지 검증하여 CubeMX 설정 변경을 감지합니다.

**하드웨어 주의사항:**

CAN 트랜시버의 STBY(standby) 핀은 반드시 **LOW**로 설정해야 트랜시버가 활성화됩니다. STBY 핀이 플로팅 상태이거나 HIGH이면 트랜시버가 대기 모드로 진입하여 CAN 통신이 실패합니다 (IR 레지스터에 PEA/ARA 프로토콜 에러 발생).

**저전력 웨이크업 (Remote Wake via RXD):**

MCP2561/2FD 같은 트랜시버는 버스 활동 모니터링이 가능한 저전력 대기 모드를 지원합니다:

- Standby 모드(STBY=HIGH)에서 송신기와 고속 수신기가 비활성화되어 전력 소모 최소화
- 저전력 수신기와 웨이크업 필터는 버스 모니터링을 위해 활성 상태 유지
- RXD는 웨이크업 필터로 인해 CAN 버스의 **지연된** 신호를 출력
- CAN 버스의 Dominant 상태가 RXD 핀에 **하강 에지**를 발생시켜 MCU 인터럽트
- MCU는 고속 통신을 위해 STBY=LOW로 Normal 모드로 복귀해야 함

```cpp
// 예시: RXD 핀 (PD0)을 웨이크업용 EXTI 입력으로 설정
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(EXTI0_IRQn);

// EXTI 콜백에서: 웨이크업 후 트랜시버 활성화
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);  // 활성화
    }
}
```

**STM32 저전력 모드 고려사항:**

- **Stop 모드**: 모든 GPIO를 EXTI로 웨이크업 가능. 진입 전 pending 인터럽트 클리어 필요 (`NVIC->ICPR[n]`), SysTick 정지 (`HAL_SuspendTick()`). PRIMASK (`__disable_irq()`)는 핸들러 실행만 막고 EXTI는 WFI를 깨움.
- **Standby 모드**: 전용 WKUP 핀만 웨이크업 가능 (STM32H7: PA0=WKUP1, PA2=WKUP2 등). 필요시 RXD를 WKUP 핀에 연결.

## 요구사항

- C++17 이상
- STM32 HAL 드라이버
- ARM Cortex-M 프로세서

### C++17 선택 이유

STM32ZERO는 깔끔하고 유지보수하기 쉬운 코드를 위해 C++17 기능을 사용합니다:

- **`if constexpr`**: 템플릿 특수화 없이 컴파일 타임 분기
- **`inline` 변수**: 별도 .cpp 정의 없이 헤더 온리 라이브러리 지원

C++14 대안도 있지만 불필요한 복잡도만 증가합니다. 최신 ARM GCC 툴체인(STM32CubeCLT 포함)은 C++17을 완벽히 지원합니다.

## 설치

git 서브모듈로 추가:

```bash
git submodule add https://github.com/textrix/stm32zero.git STM32ZERO
```

Include 경로: `STM32ZERO/include`

## 설정

프로젝트 include 경로에 `stm32zero-conf.h` 생성 (선택사항):

```c
// Serial I/O 모듈 UART 번호 (예: USART3/huart3의 경우 3)
#define STM32ZERO_SIO_NUM       3
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

// FDCAN 클럭 (CubeMX .ioc의 RCC.FDCANFreq_Value)
// HAL_RCCEx_GetPeriphCLKFreq() 회피로 ~2-3KB 절약
#define STM32ZERO_FDCAN_CLOCK_HZ  80000000UL

// 네임스페이스 별칭 (선택사항)
#define STM32ZERO_NAMESPACE_ALIAS zero
```

네임스페이스 별칭 설정 시 `stm32zero::sio::write()` 대신 `zero::sio::write()`로 사용 가능.

## 코드 스타일

- **들여쓰기**: 탭 (8칸 너비)
- **네이밍**: 함수/변수는 `snake_case`, 클래스/enum은 `PascalCase`
- **Enum 값**: 대문자 (`Priority::NORMAL`, `Priority::HIGH`)

## 라이선스

MIT License
