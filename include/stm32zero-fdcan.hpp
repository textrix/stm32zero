#ifndef __STM32ZERO_FDCAN_HPP__
#define __STM32ZERO_FDCAN_HPP__

/**
 * STM32ZERO FDCAN Module - CAN-FD support with static allocation
 *
 * Features:
 *   - CAN Classic and CAN-FD support (with/without BRS)
 *   - Static allocation for all resources
 *   - HAL_FDCAN_REGISTER_CALLBACKS=1 based callback registration
 *   - FreeRTOS and bare-metal support
 *   - 80MHz clock timing presets (configurable)
 *   - DLC conversion utilities (constexpr)
 *   - Flexible filter configuration (single, dual, range, mask)
 *   - Bus state and error counter monitoring
 *
 * Requirements:
 *   - USE_HAL_FDCAN_REGISTER_CALLBACKS=1 in stm32h7xx_hal_conf.h
 *
 * Usage:
 *   // In source file (.cpp) - define fdcan instance
 *   STM32ZERO_DEFINE_FDCAN(can1, hfdcan1, 16);
 *
 *   // Initialize after MX_FDCAN1_Init()
 *   STM32ZERO_INIT_FDCAN(can1, hfdcan1);
 *
 *   // Configure and open (new API)
 *   can1.set_format(fdcan::FrameFormat::FD_BRS);
 *   can1.set_nominal(fdcan::Bitrate::K500);
 *   can1.set_data(fdcan::Bitrate::M2);
 *   can1.set_filter_range(0x100, 0x1FF);
 *   can1.open();
 *
 *   // Send/Receive
 *   can1.send(0x100, data, 8, 1000);           // Standard ID
 *   can1.send_ext(0x18DAF110, data, 8, 1000);  // Extended ID
 *
 *   fdcan::RxMessage msg;
 *   can1.recv(&msg, 1000);
 *
 *   // Check bus state
 *   if (can1.bus_state() == fdcan::BusState::BUS_OFF) {
 *       // Handle error
 *   }
 */

#include <cstdint>
#include <cstddef>
#include <cstring>

#include "stm32zero.hpp"

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
#include "stm32zero-freertos.hpp"
#endif

namespace stm32zero {
namespace fdcan {

//=============================================================================
// Enumerations
//=============================================================================

/**
 * CAN Data Length Code
 *
 * Maps byte counts to DLC values (0-15).
 * CAN Classic: 0-8 bytes
 * CAN FD: 0-64 bytes (8,12,16,20,24,32,48,64)
 */
enum class Dlc : uint8_t {
	DLC_0	= 0,
	DLC_1	= 1,
	DLC_2	= 2,
	DLC_3	= 3,
	DLC_4	= 4,
	DLC_5	= 5,
	DLC_6	= 6,
	DLC_7	= 7,
	DLC_8	= 8,
	DLC_12	= 9,
	DLC_16	= 10,
	DLC_20	= 11,
	DLC_24	= 12,
	DLC_32	= 13,
	DLC_48	= 14,
	DLC_64	= 15
};

/**
 * CAN Frame Format
 */
enum class FrameFormat : uint8_t {
	CLASSIC,	// CAN 2.0 (up to 8 bytes, max 1Mbps)
	FD_NO_BRS,	// CAN FD without Bit Rate Switch
	FD_BRS		// CAN FD with Bit Rate Switch
};

/**
 * CAN Identifier Type
 */
enum class IdType : uint8_t {
	STANDARD,	// 11-bit identifier (0x000 - 0x7FF)
	EXTENDED	// 29-bit identifier
};

/**
 * CAN Bitrate Presets
 *
 * Based on 80MHz FDCAN clock.
 * For CAN FD with BRS: nominal phase is fixed at 500Kbps,
 * data phase uses the selected bitrate.
 */
enum class Bitrate : uint32_t {
	K33_3	= 33333,
	K50	= 50000,
	K83_3	= 83333,
	K100	= 100000,
	K125	= 125000,
	K250	= 250000,
	K500	= 500000,
	M1	= 1000000,
	M2	= 2000000,
	M4	= 4000000,
	M5	= 5000000
};

/**
 * Operation Status
 */
enum class Status : int8_t {
	OK		= 0,
	ERROR		= -1,
	TIMEOUT		= -2,
	NOT_READY	= -3,
	INVALID_PARAM	= -4
};

/**
 * Sample Point (x10 for precision)
 *
 * Common sample points:
 *   - 75.0%: Used for data phase in CAN-FD
 *   - 80.0%: Standard CAN 2.0 setting
 *   - 85.0%: Conservative for noisy environments
 *   - 87.5%: Common for industrial CAN
 *   - 90.0%: Aggressive setting for short buses
 */
enum class SamplePoint : uint16_t {
	P75_0 = 750,
	P80_0 = 800,
	P85_0 = 850,
	P87_5 = 875,
	P90_0 = 900
};

/**
 * CAN Bus State (from PSR register)
 */
enum class BusState : uint8_t {
	ACTIVE,		// Normal operation
	WARNING,	// Error counter > 96
	PASSIVE,	// Error counter > 127
	BUS_OFF		// Error counter > 255, bus disconnected
};

//=============================================================================
// DLC Conversion (constexpr)
//=============================================================================

/**
 * DLC to byte count lookup table
 */
constexpr uint8_t dlc_to_bytes_table[16] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

/**
 * Convert DLC to byte count
 *
 * @param dlc Data Length Code (0-15)
 * @return Byte count (0-64)
 */
constexpr uint8_t dlc_to_bytes(Dlc dlc)
{
	return dlc_to_bytes_table[static_cast<uint8_t>(dlc)];
}

/**
 * Convert DLC value to byte count
 *
 * @param dlc Data Length Code value (0-15)
 * @return Byte count (0-64)
 */
constexpr uint8_t dlc_to_bytes(uint8_t dlc)
{
	return (dlc <= 15) ? dlc_to_bytes_table[dlc] : 64;
}

/**
 * Bytes to DLC lookup table (rounds up to nearest valid DLC)
 */
constexpr uint8_t bytes_to_dlc_table[65] = {
	// 0-8: direct mapping
	0, 1, 2, 3, 4, 5, 6, 7, 8,
	// 9-12: round up to DLC_12 (9)
	9, 9, 9, 9,
	// 13-16: round up to DLC_16 (10)
	10, 10, 10, 10,
	// 17-20: round up to DLC_20 (11)
	11, 11, 11, 11,
	// 21-24: round up to DLC_24 (12)
	12, 12, 12, 12,
	// 25-32: round up to DLC_32 (13)
	13, 13, 13, 13, 13, 13, 13, 13,
	// 33-48: round up to DLC_48 (14)
	14, 14, 14, 14, 14, 14, 14, 14,
	14, 14, 14, 14, 14, 14, 14, 14,
	// 49-64: round up to DLC_64 (15)
	15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15
};

/**
 * Convert byte count to DLC (rounds up)
 *
 * @param bytes Byte count (0-64)
 * @return DLC enum value
 */
constexpr Dlc bytes_to_dlc(uint8_t bytes)
{
	return static_cast<Dlc>((bytes <= 64) ? bytes_to_dlc_table[bytes] : 15);
}

//=============================================================================
// RxMessage Structure
//=============================================================================

/**
 * Received CAN message structure
 */
struct RxMessage {
	uint32_t id;		// Message identifier
	IdType id_type;		// Standard or Extended
	FrameFormat format;	// Classic, FD_NO_BRS, FD_BRS
	uint8_t dlc;		// Data Length Code (0-15)
	uint8_t length;		// Actual byte count
	uint32_t timestamp;	// RX timestamp (if enabled)
	uint8_t data[64];	// Payload data
};

//=============================================================================
// Timing Configuration
//=============================================================================

/**
 * FDCAN clock frequency (override in stm32zero-conf.h if not 80MHz)
 */
#ifndef STM32ZERO_FDCAN_CLOCK_HZ
#define STM32ZERO_FDCAN_CLOCK_HZ 80000000
#endif

/**
 * Bit timing parameters
 */
struct BitTiming {
	uint16_t prescaler;
	uint8_t sjw;		// Sync Jump Width
	uint8_t seg1;		// Time Segment 1 (before sample point)
	uint8_t seg2;		// Time Segment 2 (after sample point)

	bool is_valid() const {
		return prescaler > 0 && sjw > 0 && seg1 > 0 && seg2 > 0;
	}
};

/**
 * Calculate timing parameters for given bitrate and sample point
 *
 * @param bitrate_hz Desired bitrate in Hz
 * @param sample_point_x10 Sample point * 10 (e.g., 875 for 87.5%)
 * @return BitTiming structure (prescaler=0 if calculation fails)
 */
BitTiming calculate_timing(uint32_t bitrate_hz, uint16_t sample_point_x10);

/**
 * Filter configuration type
 */
enum class FilterType : uint8_t {
	ACCEPT_ALL,	// Accept all messages (no filter)
	SINGLE_ID,	// Match single ID
	DUAL_ID,	// Match either of two IDs
	RANGE,		// Match ID range [min, max]
	MASK		// Classic mask filter (id & mask)
};

/**
 * Filter configuration structure
 */
struct FilterConfig {
	FilterType type = FilterType::ACCEPT_ALL;
	IdType id_type = IdType::STANDARD;
	uint32_t id1 = 0;	// ID / Min ID / Mask ID
	uint32_t id2 = 0;	// Second ID / Max ID / Mask value
};

//=============================================================================
// Fdcan Class
//=============================================================================

/**
 * FDCAN controller wrapper
 *
 * Provides CAN Classic and CAN-FD communication with:
 *   - Static allocation (no heap)
 *   - HAL callback registration
 *   - FreeRTOS queue-based RX (or bare-metal polling)
 *   - Thread-safe TX with mutex (FreeRTOS) or critical section
 *
 * Usage (new API):
 *   can1.set_format(FrameFormat::FD_BRS);
 *   can1.set_nominal(Bitrate::K500);
 *   can1.set_data(Bitrate::M2);
 *   can1.set_filter_range(0x100, 0x1FF);
 *   can1.open();
 *
 *   can1.send(0x100, data, 8, 1000);         // Standard ID
 *   can1.send_ext(0x18DAF110, data, 8, 1000); // Extended ID
 */
class Fdcan {
public:
	using ErrorCallback = void (*)(Fdcan* self, uint32_t error_code);

	Fdcan() = default;

	/**
	 * Initialize FDCAN instance
	 *
	 * @param hfdcan HAL FDCAN handle
	 * @param rx_queue FreeRTOS queue for RX messages (or nullptr for bare-metal)
	 *
	 * Call after MX_FDCANx_Init() and before open().
	 */
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	void init(FDCAN_HandleTypeDef* hfdcan, QueueHandle_t rx_queue);
#else
	void init(FDCAN_HandleTypeDef* hfdcan, RxMessage* rx_buffer, size_t rx_buffer_size);
#endif

	//=========================================================================
	// Configuration API (call before open())
	//=========================================================================

	/**
	 * Set frame format
	 *
	 * @param format CLASSIC, FD_NO_BRS, or FD_BRS
	 *
	 * If not called, uses CubeMX configuration.
	 */
	void set_format(FrameFormat format);

	/**
	 * Set nominal (arbitration) bitrate with preset timing
	 *
	 * @param rate Bitrate preset
	 * @param sp Sample point (default 87.5%)
	 */
	void set_nominal(Bitrate rate, SamplePoint sp = SamplePoint::P87_5);

	/**
	 * Set nominal timing with custom parameters
	 *
	 * @param timing Custom bit timing parameters
	 */
	void set_nominal_ex(const BitTiming& timing);

	/**
	 * Set data phase bitrate (for FD_BRS only)
	 *
	 * @param rate Bitrate preset
	 * @param sp Sample point (default 75.0% for data phase)
	 */
	void set_data(Bitrate rate, SamplePoint sp = SamplePoint::P75_0);

	/**
	 * Set data phase timing with custom parameters
	 *
	 * @param timing Custom bit timing parameters
	 */
	void set_data_ex(const BitTiming& timing);

	//=========================================================================
	// Filter Configuration API (call before open())
	//=========================================================================

	/**
	 * Set filter for single ID
	 *
	 * @param id Message ID to accept
	 * @param id_type Standard (11-bit) or Extended (29-bit)
	 */
	void set_filter_id(uint32_t id, IdType id_type = IdType::STANDARD);

	/**
	 * Set filter for two IDs (dual filter)
	 *
	 * @param id1 First message ID
	 * @param id2 Second message ID
	 * @param id_type Standard or Extended
	 */
	void set_filter_dual(uint32_t id1, uint32_t id2, IdType id_type = IdType::STANDARD);

	/**
	 * Set filter for ID range
	 *
	 * @param min Minimum ID (inclusive)
	 * @param max Maximum ID (inclusive)
	 * @param id_type Standard or Extended
	 */
	void set_filter_range(uint32_t min, uint32_t max, IdType id_type = IdType::STANDARD);

	/**
	 * Set filter with mask
	 *
	 * @param id Base ID
	 * @param mask Mask (1 = must match, 0 = don't care)
	 * @param id_type Standard or Extended
	 */
	void set_filter_mask(uint32_t id, uint32_t mask, IdType id_type = IdType::STANDARD);

	/**
	 * Accept all messages (disable filter)
	 */
	void set_filter_accept_all();

	//=========================================================================
	// Control API
	//=========================================================================

	/**
	 * Open CAN channel (new simplified API)
	 *
	 * Applies pending configuration and starts the FDCAN peripheral.
	 * If no configuration was set, uses CubeMX defaults.
	 *
	 * @return Status::OK on success
	 */
	Status open();

	/**
	 * Close CAN channel
	 */
	Status close();

	/**
	 * Check if channel is open
	 */
	bool is_open() const { return opened_; }

	//=========================================================================
	// Transmit API
	//=========================================================================

	/**
	 * Send CAN message with Standard ID (11-bit)
	 *
	 * @param id Standard message identifier (0x000-0x7FF)
	 * @param data Pointer to data (can be nullptr if len==0)
	 * @param len Data length in bytes (0-8 for Classic, 0-64 for FD)
	 * @param timeout_ms Timeout in milliseconds
	 * @return Status::OK on success
	 */
	Status send(uint16_t id, const uint8_t* data, uint8_t len, uint32_t timeout_ms);

	/**
	 * Send CAN message with Extended ID (29-bit)
	 *
	 * @param id Extended message identifier (0x00000000-0x1FFFFFFF)
	 * @param data Pointer to data (can be nullptr if len==0)
	 * @param len Data length in bytes (0-8 for Classic, 0-64 for FD)
	 * @param timeout_ms Timeout in milliseconds
	 * @return Status::OK on success
	 */
	Status send_ext(uint32_t id, const uint8_t* data, uint8_t len, uint32_t timeout_ms);

	//=========================================================================
	// Receive API
	//=========================================================================

	/**
	 * Receive CAN message
	 *
	 * @param msg Pointer to RxMessage structure
	 * @param timeout_ms Timeout in milliseconds
	 * @return Status::OK on success, Status::TIMEOUT if no message
	 */
	Status recv(RxMessage* msg, uint32_t timeout_ms);

	//=========================================================================
	// Status/Utility API
	//=========================================================================

	/**
	 * Get current bus state (from PSR register)
	 *
	 * @return Current bus state (ACTIVE, WARNING, PASSIVE, BUS_OFF)
	 */
	BusState bus_state() const;

	/**
	 * Get error counters
	 *
	 * @param tx_err Pointer to store TX error counter (can be nullptr)
	 * @param rx_err Pointer to store RX error counter (can be nullptr)
	 */
	void error_counters(uint8_t* tx_err, uint8_t* rx_err) const;

	/**
	 * Get TX FIFO free level
	 *
	 * @return Number of free elements in TX FIFO (0-3)
	 */
	uint8_t tx_fifo_free_level() const;

	/**
	 * Get RX FIFO0 fill level
	 *
	 * @return Number of messages in RX FIFO0
	 */
	uint8_t rx_fifo_fill_level() const;

	/**
	 * Get RX queue handle (FreeRTOS only)
	 */
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	QueueHandle_t rx_queue() const { return rx_queue_; }
#endif

	/**
	 * Get last HAL error code
	 */
	uint32_t error_code() const;

	/**
	 * Set error callback
	 */
	void set_error_callback(ErrorCallback cb) { error_callback_ = cb; }

	/**
	 * Get HAL handle
	 */
	FDCAN_HandleTypeDef* handle() const { return hfdcan_; }

	// ISR callbacks (called internally)
	void rx_fifo0_isr(uint32_t rx_fifo0_its);
	void tx_complete_isr(uint32_t buffer_indexes);
	void error_isr();

private:
	Status register_callbacks();
	Status apply_timing_config();
	Status apply_filter_config();
	Status configure_interrupts();
	Status send_internal(uint32_t id, IdType id_type, const uint8_t* data, uint8_t len, uint32_t timeout_ms);

	FDCAN_HandleTypeDef* hfdcan_ = nullptr;
	FDCAN_TxHeaderTypeDef tx_header_{};
	uint8_t tx_scratch_[64]{};	// For padding data to slot size

	// Current state
	FrameFormat format_ = FrameFormat::CLASSIC;
	bool opened_ = false;

	// Pending configuration (applied on open())
	bool format_changed_ = false;
	bool nominal_changed_ = false;
	bool data_changed_ = false;
	FrameFormat pending_format_ = FrameFormat::CLASSIC;
	BitTiming pending_nominal_{};
	BitTiming pending_data_{};

	// Filter configuration
	FilterConfig filter_config_;

	ErrorCallback error_callback_ = nullptr;

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	QueueHandle_t rx_queue_ = nullptr;
	freertos::StaticMutex tx_mutex_;
	freertos::StaticBinarySemaphore tx_sem_;
#else
	RxMessage* rx_buffer_ = nullptr;
	size_t rx_buffer_size_ = 0;
	volatile size_t rx_head_ = 0;
	volatile size_t rx_tail_ = 0;
	volatile bool tx_complete_ = true;
#endif
};

} // namespace fdcan
} // namespace stm32zero

//=============================================================================
// FDCAN Instance Macros
//=============================================================================

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)

/**
 * Define a FDCAN instance with FreeRTOS queue (use in .cpp file)
 *
 * @param name Instance name
 * @param huart HAL FDCAN handle name (without &)
 * @param queue_len RX queue length
 */
#define STM32ZERO_DEFINE_FDCAN(name, hfdcan, queue_len) \
	static stm32zero::freertos::StaticQueue< \
		sizeof(stm32zero::fdcan::RxMessage), queue_len> name##_rx_queue_; \
	static stm32zero::fdcan::Fdcan name

/**
 * Initialize FDCAN instance (call after MX_FDCANx_Init)
 */
#define STM32ZERO_INIT_FDCAN(name, hfdcan) \
	do { \
		name##_rx_queue_.create(); \
		name.init(&hfdcan, name##_rx_queue_.handle()); \
	} while(0)

#else // Bare-metal

/**
 * Define a FDCAN instance for bare-metal (use in .cpp file)
 *
 * @param name Instance name
 * @param hfdcan HAL FDCAN handle name (without &)
 * @param buffer_len RX buffer length
 */
#define STM32ZERO_DEFINE_FDCAN(name, hfdcan, buffer_len) \
	STM32ZERO_DTCM static stm32zero::fdcan::RxMessage name##_rx_buffer_[buffer_len]; \
	static stm32zero::fdcan::Fdcan name

/**
 * Initialize FDCAN instance for bare-metal
 */
#define STM32ZERO_INIT_FDCAN(name, hfdcan) \
	name.init(&hfdcan, name##_rx_buffer_, sizeof(name##_rx_buffer_) / sizeof(name##_rx_buffer_[0]))

#endif // STM32ZERO_RTOS_FREERTOS

#endif // __STM32ZERO_FDCAN_HPP__
