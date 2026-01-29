/**
 * STM32ZERO FDCAN Module Implementation
 */

#include "main.h"
#include "stm32zero-fdcan.hpp"

#if defined(HAL_FDCAN_MODULE_ENABLED)

namespace stm32zero {
namespace fdcan {

//=============================================================================
// Timing Calculation
//=============================================================================

BitTiming Fdcan::calculate_timing(uint32_t bitrate_hz, uint16_t sample_point_x10) const
{
	const uint32_t clock_hz = clock_hz_;

	// Hardware limits for STM32H7 FDCAN
	constexpr uint16_t max_prescaler = 256;
	constexpr uint16_t min_tq = 4;
	constexpr uint16_t max_tq = 256;
	constexpr uint16_t max_seg1 = 128;
	constexpr uint16_t max_seg2 = 64;
	constexpr uint16_t max_sjw = 64;

	BitTiming best = {0, 0, 0, 0};
	uint32_t best_error = UINT32_MAX;

	for (uint16_t prescaler = 1; prescaler <= max_prescaler; prescaler++) {
		uint32_t tq_total = clock_hz / (prescaler * bitrate_hz);

		if (tq_total < min_tq || tq_total > max_tq) {
			continue;
		}

		uint32_t seg1 = (sample_point_x10 * tq_total) / 1000 - 1;
		uint32_t seg2 = tq_total - 1 - seg1;

		if (seg1 < 1 || seg1 > max_seg1 || seg2 < 1 || seg2 > max_seg2) {
			continue;
		}

		uint32_t actual_bitrate = clock_hz / (prescaler * tq_total);
		uint32_t error = (actual_bitrate > bitrate_hz)
				 ? (actual_bitrate - bitrate_hz)
				 : (bitrate_hz - actual_bitrate);

		if (error < best_error) {
			best_error = error;
			best.prescaler = prescaler;
			best.seg1 = static_cast<uint8_t>(seg1);
			best.seg2 = static_cast<uint8_t>(seg2);
			best.sjw = (seg2 < max_sjw) ? static_cast<uint8_t>(seg2) : max_sjw;
		}

		if (error == 0) {
			break;
		}
	}

	return best;
}

//=============================================================================
// FDCAN Instance Mapping Table
//=============================================================================

static constexpr size_t MAX_FDCAN_INSTANCES = 3;

namespace {
	stm32zero::detail::HandleMap<Fdcan, FDCAN_HandleTypeDef, MAX_FDCAN_INSTANCES> fdcan_map_;
}

Fdcan* find_fdcan(FDCAN_HandleTypeDef* hfdcan)
{
	return stm32zero::detail::map_find(fdcan_map_, hfdcan);
}

static void register_fdcan(FDCAN_HandleTypeDef* hfdcan, Fdcan* fdcan)
{
	stm32zero::detail::map_add(fdcan_map_, hfdcan, fdcan);
}

//=============================================================================
// Common Callback Handlers (used by both registered and weak callbacks)
//=============================================================================

static void fdcan_rx_fifo0_handler(FDCAN_HandleTypeDef* hfdcan, uint32_t its)
{
	auto* self = find_fdcan(hfdcan);
	if (self) self->rx_fifo0_isr(its);
}

static void fdcan_tx_complete_handler(FDCAN_HandleTypeDef* hfdcan, uint32_t indexes)
{
	auto* self = find_fdcan(hfdcan);
	if (self) self->tx_complete_isr(indexes);
}

static void fdcan_tx_fifo_empty_handler(FDCAN_HandleTypeDef* hfdcan)
{
	auto* self = find_fdcan(hfdcan);
	if (self) self->tx_complete_isr(0);
}

static void fdcan_error_handler(FDCAN_HandleTypeDef* hfdcan, uint32_t its)
{
	(void)its;
	auto* self = find_fdcan(hfdcan);
	if (self) self->error_isr();
}

//=============================================================================
// Fdcan Implementation
//=============================================================================

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)

void Fdcan::init(FDCAN_HandleTypeDef* hfdcan, QueueHandle_t rx_queue)
{
	hfdcan_ = hfdcan;
	rx_queue_ = rx_queue;
	opened_ = false;

	// Auto-detect FDCAN clock frequency at runtime
	clock_hz_ = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

	tx_mutex_.create();
	tx_sem_.create(TX_FIFO_SIZE);

	// Register in mapping table
	register_fdcan(hfdcan, this);

	// Register HAL callbacks (State should be READY after MX_FDCANx_Init)
	register_callbacks();
}

#else // Bare-metal

void Fdcan::init(FDCAN_HandleTypeDef* hfdcan, RxMessage* rx_buffer, size_t rx_buffer_size)
{
	hfdcan_ = hfdcan;
	rx_buffer_ = rx_buffer;
	rx_buffer_size_ = rx_buffer_size;
	rx_head_ = 0;
	rx_tail_ = 0;
	opened_ = false;

	// Auto-detect FDCAN clock frequency at runtime
	clock_hz_ = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

	// Register in mapping table
	register_fdcan(hfdcan, this);

	// Register HAL callbacks (State should be READY after MX_FDCANx_Init)
	register_callbacks();
}

#endif

IoResult Fdcan::register_callbacks()
{
#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1
	if (HAL_FDCAN_RegisterRxFifo0Callback(hfdcan_, fdcan_rx_fifo0_handler) != HAL_OK) {
		return {Status::ERROR, 0};
	}
	if (HAL_FDCAN_RegisterTxBufferCompleteCallback(hfdcan_, fdcan_tx_complete_handler) != HAL_OK) {
		return {Status::ERROR, 0};
	}
	if (HAL_FDCAN_RegisterCallback(hfdcan_, HAL_FDCAN_TX_FIFO_EMPTY_CB_ID, fdcan_tx_fifo_empty_handler) != HAL_OK) {
		return {Status::ERROR, 0};
	}
	if (HAL_FDCAN_RegisterErrorStatusCallback(hfdcan_, fdcan_error_handler) != HAL_OK) {
		return {Status::ERROR, 0};
	}
#endif
	// When USE_HAL_FDCAN_REGISTER_CALLBACKS=0, weak callbacks are used
	return {Status::OK, 0};
}

//=============================================================================
// Configuration API Implementation
//=============================================================================

void Fdcan::set_format(FrameFormat format)
{
	pending_format_ = format;
	format_changed_ = true;
}

void Fdcan::set_nominal(Bitrate rate, SamplePoint sp)
{
	pending_nominal_ = calculate_timing(static_cast<uint32_t>(rate),
					    static_cast<uint16_t>(sp));
	nominal_changed_ = true;
}

void Fdcan::set_nominal_ex(const BitTiming& timing)
{
	pending_nominal_ = timing;
	nominal_changed_ = true;
}

void Fdcan::set_data(Bitrate rate, SamplePoint sp)
{
	pending_data_ = calculate_timing(static_cast<uint32_t>(rate),
					 static_cast<uint16_t>(sp));
	data_changed_ = true;
}

void Fdcan::set_data_ex(const BitTiming& timing)
{
	pending_data_ = timing;
	data_changed_ = true;
}

//=============================================================================
// Filter Configuration API Implementation
//=============================================================================

void Fdcan::set_filter_id(uint32_t id, IdType id_type)
{
	filter_config_.type = FilterType::DUAL_ID;
	filter_config_.id_type = id_type;
	filter_config_.id1 = id;
	filter_config_.id2 = id;  // Same ID for both = single ID match
}

void Fdcan::set_filter_dual(uint32_t id1, uint32_t id2, IdType id_type)
{
	filter_config_.type = FilterType::DUAL_ID;
	filter_config_.id_type = id_type;
	filter_config_.id1 = id1;
	filter_config_.id2 = id2;
}

void Fdcan::set_filter_range(uint32_t min, uint32_t max, IdType id_type)
{
	filter_config_.type = FilterType::RANGE;
	filter_config_.id_type = id_type;
	filter_config_.id1 = min;
	filter_config_.id2 = max;
}

void Fdcan::set_filter_mask(uint32_t id, uint32_t mask, IdType id_type)
{
	filter_config_.type = FilterType::MASK;
	filter_config_.id_type = id_type;
	filter_config_.id1 = id;
	filter_config_.id2 = mask;
}

void Fdcan::set_filter_accept_all()
{
	filter_config_.type = FilterType::ACCEPT_ALL;
}

//=============================================================================
// Apply Configuration (internal)
//=============================================================================

IoResult Fdcan::apply_timing_config()
{
	bool need_reinit = format_changed_ || nominal_changed_ || data_changed_;

	if (!need_reinit) {
		// No timing changes, use CubeMX configuration
		// Read current format from HAL
		if (hfdcan_->Init.FrameFormat == FDCAN_FRAME_CLASSIC) {
			format_ = FrameFormat::CLASSIC;
		} else if (hfdcan_->Init.FrameFormat == FDCAN_FRAME_FD_NO_BRS) {
			format_ = FrameFormat::FD_NO_BRS;
		} else {
			format_ = FrameFormat::FD_BRS;
		}
		return {Status::OK, 0};
	}

	// Set frame format
	if (format_changed_) {
		format_ = pending_format_;
		switch (format_) {
		case FrameFormat::CLASSIC:
			hfdcan_->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
			break;
		case FrameFormat::FD_NO_BRS:
			hfdcan_->Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
			break;
		case FrameFormat::FD_BRS:
			hfdcan_->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
			break;
		}
	} else {
		// Read current format from HAL
		if (hfdcan_->Init.FrameFormat == FDCAN_FRAME_CLASSIC) {
			format_ = FrameFormat::CLASSIC;
		} else if (hfdcan_->Init.FrameFormat == FDCAN_FRAME_FD_NO_BRS) {
			format_ = FrameFormat::FD_NO_BRS;
		} else {
			format_ = FrameFormat::FD_BRS;
		}
	}

	// Common settings
	hfdcan_->Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan_->Init.AutoRetransmission = ENABLE;
	hfdcan_->Init.TransmitPause = DISABLE;
	hfdcan_->Init.ProtocolException = DISABLE;

	// Message RAM configuration - use MX settings as-is
	// (H5 series doesn't have these fields, H7 has them but MX handles it)
#if FDCAN_OVERRIDE_MX_CONFIG
	// Message RAM offset
#if defined(FDCAN2)
	hfdcan_->Init.MessageRAMOffset = (hfdcan_->Instance == FDCAN1) ? 0 : 1280;
#else
	hfdcan_->Init.MessageRAMOffset = 0;
#endif

	// Message RAM configuration
	hfdcan_->Init.StdFiltersNbr = 1;
	hfdcan_->Init.ExtFiltersNbr = 1;
	hfdcan_->Init.RxFifo0ElmtsNbr = 2;
	hfdcan_->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan_->Init.RxFifo1ElmtsNbr = 0;
	hfdcan_->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan_->Init.RxBuffersNbr = 0;
	hfdcan_->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hfdcan_->Init.TxEventsNbr = 0;
	hfdcan_->Init.TxBuffersNbr = 0;
	hfdcan_->Init.TxFifoQueueElmtsNbr = 2;
	hfdcan_->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hfdcan_->Init.TxElmtSize = FDCAN_DATA_BYTES_8;
#endif

	// Nominal timing
	if (nominal_changed_) {
		hfdcan_->Init.NominalPrescaler = pending_nominal_.prescaler;
		hfdcan_->Init.NominalSyncJumpWidth = pending_nominal_.sjw;
		hfdcan_->Init.NominalTimeSeg1 = pending_nominal_.seg1;
		hfdcan_->Init.NominalTimeSeg2 = pending_nominal_.seg2;
	}

	// Data timing
	if (data_changed_) {
		hfdcan_->Init.DataPrescaler = pending_data_.prescaler;
		hfdcan_->Init.DataSyncJumpWidth = pending_data_.sjw;
		hfdcan_->Init.DataTimeSeg1 = pending_data_.seg1;
		hfdcan_->Init.DataTimeSeg2 = pending_data_.seg2;
	} else if (nominal_changed_) {
		// If only nominal changed, copy to data phase
		hfdcan_->Init.DataPrescaler = pending_nominal_.prescaler;
		hfdcan_->Init.DataSyncJumpWidth = pending_nominal_.sjw;
		hfdcan_->Init.DataTimeSeg1 = pending_nominal_.seg1;
		hfdcan_->Init.DataTimeSeg2 = pending_nominal_.seg2;
	}

	// Update element sizes for FD - use MX settings as-is
#if FDCAN_OVERRIDE_MX_CONFIG
	if (format_ != FrameFormat::CLASSIC) {
		hfdcan_->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
		hfdcan_->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
		hfdcan_->Init.RxBufferSize = FDCAN_DATA_BYTES_64;
		hfdcan_->Init.TxElmtSize = FDCAN_DATA_BYTES_64;
	}
#endif

	// Re-initialize FDCAN with new settings
	HAL_FDCAN_DeInit(hfdcan_);
	if (HAL_FDCAN_Init(hfdcan_) != HAL_OK) {
		return {Status::ERROR, 0};
	}

	// Re-register callbacks after HAL_FDCAN_Init
	IoResult result = register_callbacks();
	if (!result) {
		return result;
	}

	// Enable TX delay compensation for high-speed FD
	if (format_ != FrameFormat::CLASSIC && data_changed_) {
		uint32_t data_bitrate = clock_hz_ /
					(pending_data_.prescaler * (1 + pending_data_.seg1 + pending_data_.seg2));
		if (data_bitrate >= 2000000) {
			HAL_FDCAN_ConfigTxDelayCompensation(hfdcan_, 2, 0);
			HAL_FDCAN_EnableTxDelayCompensation(hfdcan_);
		}
	}

	// Clear pending flags
	format_changed_ = false;
	nominal_changed_ = false;
	data_changed_ = false;

	return {Status::OK, 0};
}

IoResult Fdcan::apply_filter_config()
{
	FDCAN_FilterTypeDef filter{};

	filter.FilterIndex = 0;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	if (filter_config_.type == FilterType::ACCEPT_ALL) {
		// Accept all messages via global filter
		HAL_FDCAN_ConfigGlobalFilter(hfdcan_,
					     FDCAN_ACCEPT_IN_RX_FIFO0,
					     FDCAN_ACCEPT_IN_RX_FIFO0,
					     FDCAN_REJECT_REMOTE,
					     FDCAN_REJECT_REMOTE);
		return {Status::OK, 0};
	}

	// Configure specific filter
	filter.IdType = (filter_config_.id_type == IdType::EXTENDED)
			? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;

	switch (filter_config_.type) {
	case FilterType::SINGLE_ID:
	case FilterType::DUAL_ID:
		filter.FilterType = FDCAN_FILTER_DUAL;
		filter.FilterID1 = filter_config_.id1;
		filter.FilterID2 = filter_config_.id2;
		break;

	case FilterType::RANGE:
		filter.FilterType = FDCAN_FILTER_RANGE;
		filter.FilterID1 = filter_config_.id1;  // Min
		filter.FilterID2 = filter_config_.id2;  // Max
		break;

	case FilterType::MASK:
		filter.FilterType = FDCAN_FILTER_MASK;
		filter.FilterID1 = filter_config_.id1;  // ID
		filter.FilterID2 = filter_config_.id2;  // Mask
		break;

	default:
		break;
	}

	if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter) != HAL_OK) {
		return {Status::ERROR, 0};
	}

	// Reject non-matching messages
	HAL_FDCAN_ConfigGlobalFilter(hfdcan_,
				     FDCAN_REJECT,
				     FDCAN_REJECT,
				     FDCAN_REJECT_REMOTE,
				     FDCAN_REJECT_REMOTE);

	return {Status::OK, 0};
}


IoResult Fdcan::configure_interrupts()
{
	// Configure FIFO watermark (H5 doesn't have this API)
#if defined(FDCAN_CFG_RX_FIFO0)
	HAL_FDCAN_ConfigFifoWatermark(hfdcan_, FDCAN_CFG_RX_FIFO0, 2);
#endif

	uint32_t active_its =
		FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
		FDCAN_IT_TX_COMPLETE |
		FDCAN_IT_TX_ABORT_COMPLETE |
		FDCAN_IT_TX_FIFO_EMPTY |
		FDCAN_IT_ARB_PROTOCOL_ERROR |
		FDCAN_IT_DATA_PROTOCOL_ERROR |
		FDCAN_IT_TIMEOUT_OCCURRED |
		FDCAN_IT_BUS_OFF;

	if (HAL_FDCAN_ActivateNotification(hfdcan_, active_its, 0) != HAL_OK) {
		return {Status::ERROR, 0};
	}

	return {Status::OK, 0};
}

IoResult Fdcan::open()
{
	if (opened_) {
		return {Status::ERROR, 0};
	}

	// Apply timing configuration (or use CubeMX defaults)
	IoResult result = apply_timing_config();
	if (!result) {
		return result;
	}

	// Prepare TX header template based on format
	tx_header_.TxFrameType = FDCAN_DATA_FRAME;
	tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header_.MessageMarker = 0;
	tx_header_.FDFormat = (format_ == FrameFormat::CLASSIC) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
	tx_header_.BitRateSwitch = (format_ == FrameFormat::FD_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;

	// Apply filter configuration
	result = apply_filter_config();
	if (!result) {
		return result;
	}

	result = configure_interrupts();
	if (!result) {
		return result;
	}

	if (HAL_FDCAN_Start(hfdcan_) != HAL_OK) {
		return {Status::ERROR, 0};
	}

	opened_ = true;
	return {Status::OK, 0};
}

IoResult Fdcan::close()
{
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}

	HAL_FDCAN_Stop(hfdcan_);
	opened_ = false;
	return {Status::OK, 0};
}

IoResult Fdcan::write_internal(uint32_t id, IdType id_type, const uint8_t* data, uint8_t len, uint32_t timeout_ms)
{
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}

	if (len > 64) {
		return {Status::INVALID_PARAM, 0};
	}

	if (format_ == FrameFormat::CLASSIC && len > 8) {
		return {Status::INVALID_PARAM, 0};
	}

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	// Wait for free FIFO slot first
	if (!tx_sem_.take(pdMS_TO_TICKS(timeout_ms))) {
		return {Status::TIMEOUT, 0};
	}

	// Then lock mutex for thread-safe header/scratch access
	freertos::MutexLock lock(tx_mutex_, pdMS_TO_TICKS(timeout_ms));
	if (!lock) {
		tx_sem_.give();  // Return slot on mutex timeout
		return {Status::TIMEOUT, 0};
	}
#else
	// Bare-metal: wait for free FIFO slot
	if (!wait_until([this]{ return tx_fifo_free_level() > 0; }, timeout_ms)) {
		return {Status::TIMEOUT, 0};
	}
#endif

	// Convert byte length to DLC and get slot size
	Dlc dlc = bytes_to_dlc(len);
	uint8_t slot_len = dlc_to_bytes(dlc);

	// Pad data if needed
	const uint8_t* tx_data = data;
	if (len != slot_len && len > 0) {
		std::memcpy(tx_scratch_, data, len);
		std::memset(tx_scratch_ + len, 0, slot_len - len);
		tx_data = tx_scratch_;
	}

	// Set TX header
	tx_header_.Identifier = id;
	tx_header_.IdType = (id_type == IdType::EXTENDED) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	tx_header_.DataLength = static_cast<uint32_t>(dlc);

	// Add message to TX FIFO
	HAL_StatusTypeDef hal_status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header_, tx_data);
	if (hal_status != HAL_OK) {
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
		tx_sem_.give();  // Return slot on HAL error
#endif
		return {Status::ERROR, 0};
	}

	// Return immediately - ISR will give() semaphore when TX completes
	return {Status::OK, len};
}

IoResult Fdcan::write(uint16_t id, const uint8_t* data, uint8_t len, uint32_t timeout_ms)
{
	if (id > 0x7FF) {
		return {Status::INVALID_PARAM, 0};
	}
	return write_internal(id, IdType::STANDARD, data, len, timeout_ms);
}

IoResult Fdcan::write_ext(uint32_t id, const uint8_t* data, uint8_t len, uint32_t timeout_ms)
{
	if (id > 0x1FFFFFFF) {
		return {Status::INVALID_PARAM, 0};
	}
	return write_internal(id, IdType::EXTENDED, data, len, timeout_ms);
}

IoResult Fdcan::read(RxMessage* msg, uint32_t timeout_ms)
{
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}

	if (msg == nullptr) {
		return {Status::INVALID_PARAM, 0};
	}

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	if (xQueueReceive(rx_queue_, msg, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
		return {Status::OK, msg->length};
	}
	return {Status::TIMEOUT, 0};

#else
	// Bare-metal: check ring buffer
	auto has_data = [this]{ return rx_head_ != rx_tail_; };

	if (!wait_until(has_data, timeout_ms)) {
		return {Status::TIMEOUT, 0};
	}

	{
		CriticalSection cs;
		if (rx_head_ != rx_tail_) {
			*msg = rx_buffer_[rx_tail_];
			rx_tail_ = (rx_tail_ + 1) % rx_buffer_size_;
			return {Status::OK, msg->length};
		}
	}
	return {Status::TIMEOUT, 0};
#endif
}

uint32_t Fdcan::error_code() const
{
	return hfdcan_ ? hfdcan_->ErrorCode : 0;
}

IoResult Fdcan::writable() const
{
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}
	uint16_t free_slots = static_cast<uint16_t>(tx_sem_.count());
	return {free_slots > 0 ? Status::OK : Status::BUFFER_FULL, free_slots};
#else
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}
	uint16_t free_slots = tx_fifo_free_level();
	return {free_slots > 0 ? Status::OK : Status::BUFFER_FULL, free_slots};
#endif
}

IoResult Fdcan::wait_writable(uint32_t timeout_ms)
{
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	TickType_t start = xTaskGetTickCount();
	TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

	while (tx_sem_.count() == 0) {
		if ((xTaskGetTickCount() - start) >= timeout_ticks) {
			return {Status::TIMEOUT, 0};
		}
		vTaskDelay(1);
	}
	uint16_t free_slots = static_cast<uint16_t>(tx_sem_.count());
	return {Status::OK, free_slots};
#else
	if (!wait_until([this]{ return tx_fifo_free_level() > 0; }, timeout_ms)) {
		return {Status::TIMEOUT, 0};
	}
	uint16_t free_slots = tx_fifo_free_level();
	return {Status::OK, free_slots};
#endif
}

IoResult Fdcan::flush(uint32_t timeout_ms)
{
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	TickType_t start = xTaskGetTickCount();
	TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

	while (tx_sem_.count() < TX_FIFO_SIZE) {
		if ((xTaskGetTickCount() - start) >= timeout_ticks) {
			return {Status::TIMEOUT, 0};
		}
		vTaskDelay(1);
	}
	return {Status::OK, 0};
#else
	if (!wait_until([this]{ return tx_fifo_free_level() == TX_FIFO_SIZE; }, timeout_ms)) {
		return {Status::TIMEOUT, 0};
	}
	return {Status::OK, 0};
#endif
}

IoResult Fdcan::readable() const
{
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}
	uint16_t pending = static_cast<uint16_t>(uxQueueMessagesWaiting(rx_queue_));
	return {pending > 0 ? Status::OK : Status::BUFFER_EMPTY, pending};
#else
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}
	uint16_t pending = (rx_head_ >= rx_tail_) ?
		(rx_head_ - rx_tail_) :
		(rx_buffer_size_ - rx_tail_ + rx_head_);
	return {pending > 0 ? Status::OK : Status::BUFFER_EMPTY, pending};
#endif
}

IoResult Fdcan::wait_readable(uint32_t timeout_ms)
{
	if (!opened_) {
		return {Status::NOT_READY, 0};
	}

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	// Peek the queue to check if message available
	RxMessage msg;
	if (xQueuePeek(rx_queue_, &msg, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
		uint16_t pending = static_cast<uint16_t>(uxQueueMessagesWaiting(rx_queue_));
		return {Status::OK, pending};
	}
	return {Status::TIMEOUT, 0};
#else
	if (!wait_until([this]{ return rx_head_ != rx_tail_; }, timeout_ms)) {
		return {Status::TIMEOUT, 0};
	}
	uint16_t pending = (rx_head_ >= rx_tail_) ?
		(rx_head_ - rx_tail_) :
		(rx_buffer_size_ - rx_tail_ + rx_head_);
	return {Status::OK, pending};
#endif
}

void Fdcan::purge()
{
#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	xQueueReset(rx_queue_);
#else
	CriticalSection cs;
	rx_head_ = 0;
	rx_tail_ = 0;
#endif
}

//=============================================================================
// Status/Utility API Implementation
//=============================================================================

BusState Fdcan::bus_state() const
{
	if (!hfdcan_) {
		return BusState::BUS_OFF;
	}

	// Read Protocol Status Register (PSR)
	uint32_t psr = hfdcan_->Instance->PSR;

	// Check Bus_Off first (highest priority)
	if (psr & FDCAN_PSR_BO) {
		return BusState::BUS_OFF;
	}

	// Check Error Passive
	if (psr & FDCAN_PSR_EP) {
		return BusState::PASSIVE;
	}

	// Check Error Warning
	if (psr & FDCAN_PSR_EW) {
		return BusState::WARNING;
	}

	return BusState::ACTIVE;
}

void Fdcan::error_counters(uint8_t* tx_err, uint8_t* rx_err) const
{
	if (!hfdcan_) {
		if (tx_err) *tx_err = 0;
		if (rx_err) *rx_err = 0;
		return;
	}

	// Read Error Counter Register (ECR)
	uint32_t ecr = hfdcan_->Instance->ECR;

	if (tx_err) {
		*tx_err = static_cast<uint8_t>((ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);
	}
	if (rx_err) {
		*rx_err = static_cast<uint8_t>((ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);
	}
}

uint8_t Fdcan::tx_fifo_free_level() const
{
	if (!hfdcan_) {
		return 0;
	}

	// Read TX FIFO/Queue Status register
	uint32_t txfqs = hfdcan_->Instance->TXFQS;

	// TFFL: TX FIFO Free Level (bits 2:0)
	return static_cast<uint8_t>(txfqs & FDCAN_TXFQS_TFFL);
}

uint8_t Fdcan::rx_fifo_fill_level() const
{
	if (!hfdcan_) {
		return 0;
	}

	// Read RX FIFO 0 Status register
	uint32_t rxf0s = hfdcan_->Instance->RXF0S;

	// F0FL: RX FIFO 0 Fill Level (bits 6:0)
	return static_cast<uint8_t>((rxf0s & FDCAN_RXF0S_F0FL) >> FDCAN_RXF0S_F0FL_Pos);
}

void Fdcan::rx_fifo0_isr(uint32_t rx_fifo0_its)
{
	(void)rx_fifo0_its;

	// Debug: Check RX FIFO status before reading
	volatile uint32_t rxf0s = hfdcan_->Instance->RXF0S;
	uint32_t fill_level = (rxf0s & FDCAN_RXF0S_F0FL) >> FDCAN_RXF0S_F0FL_Pos;

	// If FIFO is empty, something is wrong
	if (fill_level == 0) {
		// Debug: Check what interrupt actually fired
		volatile uint32_t ir = hfdcan_->Instance->IR;
		volatile uint32_t psr = hfdcan_->Instance->PSR;
		(void)ir;
		(void)psr;
		// Breakpoint here to check ir and psr values
		return;
	}

	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[64];

	if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
		return;
	}

	RxMessage msg{};
	msg.id = rx_header.Identifier;
	msg.id_type = (rx_header.IdType == FDCAN_EXTENDED_ID) ? IdType::EXTENDED : IdType::STANDARD;

	if (rx_header.FDFormat == FDCAN_FD_CAN) {
		msg.format = (rx_header.BitRateSwitch == FDCAN_BRS_ON)
			     ? FrameFormat::FD_BRS : FrameFormat::FD_NO_BRS;
	} else {
		msg.format = FrameFormat::CLASSIC;
	}

	msg.dlc = static_cast<uint8_t>(rx_header.DataLength);
	msg.length = dlc_to_bytes(msg.dlc);
	msg.timestamp = rx_header.RxTimestamp;
	std::memcpy(msg.data, rx_data, msg.length);

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	BaseType_t woken = pdFALSE;
	xQueueSendFromISR(rx_queue_, &msg, &woken);
	portYIELD_FROM_ISR(woken);

#else
	// Bare-metal: store in ring buffer
	size_t next_head = (rx_head_ + 1) % rx_buffer_size_;
	if (next_head != rx_tail_) {
		rx_buffer_[rx_head_] = msg;
		rx_head_ = next_head;
	}
	// else: buffer full, drop message
#endif
}

void Fdcan::tx_complete_isr(uint32_t buffer_indexes)
{
	(void)buffer_indexes;

#if defined(STM32ZERO_RTOS_FREERTOS) && (STM32ZERO_RTOS_FREERTOS == 1)
	tx_sem_.give_from_isr();
#else
	// Bare-metal: no action needed, write_internal polls tx_fifo_free_level()
#endif
}

void Fdcan::error_isr()
{
	// Debug: Capture error status (same as stm32util HAL_FDCAN_ErrorCallback)
	volatile uint32_t ir = hfdcan_->Instance->IR;
	volatile uint32_t ecr = hfdcan_->Instance->ECR;
	volatile uint32_t psr = hfdcan_->Instance->PSR;
	(void)ir;
	(void)ecr;
	(void)psr;
	// Breakpoint here to check error details

	if (error_callback_) {
		error_callback_(this, hfdcan_->ErrorCode);
	}
}

//=============================================================================
// Callback Processing API
//=============================================================================

void process_rx_fifo0(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its)
{
	fdcan_rx_fifo0_handler(hfdcan, rx_fifo0_its);
}

void process_tx_complete(FDCAN_HandleTypeDef* hfdcan, uint32_t buffer_indexes)
{
	fdcan_tx_complete_handler(hfdcan, buffer_indexes);
}

void process_tx_fifo_empty(FDCAN_HandleTypeDef* hfdcan)
{
	fdcan_tx_fifo_empty_handler(hfdcan);
}

void process_error(FDCAN_HandleTypeDef* hfdcan, uint32_t error_status_its)
{
	fdcan_error_handler(hfdcan, error_status_its);
}

} // namespace fdcan
} // namespace stm32zero

//=============================================================================
// Weak HAL Callbacks (for USE_HAL_FDCAN_REGISTER_CALLBACKS=0)
//=============================================================================

#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 0
#if !defined(STM32ZERO_USER_FDCAN_CALLBACKS)

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
	stm32zero::fdcan::process_rx_fifo0(hfdcan, RxFifo0ITs);
}

extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t BufferIndexes)
{
	stm32zero::fdcan::process_tx_complete(hfdcan, BufferIndexes);
}

extern "C" void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef* hfdcan)
{
	stm32zero::fdcan::process_tx_fifo_empty(hfdcan);
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
	stm32zero::fdcan::process_error(hfdcan, ErrorStatusITs);
}

#endif // !STM32ZERO_USER_FDCAN_CALLBACKS
#endif // USE_HAL_FDCAN_REGISTER_CALLBACKS == 0

//=============================================================================
// C API Wrappers
//=============================================================================

extern "C" void stm32zero_fdcan_process_rx_fifo0(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its)
{
	stm32zero::fdcan::process_rx_fifo0(hfdcan, rx_fifo0_its);
}

extern "C" void stm32zero_fdcan_process_tx_complete(FDCAN_HandleTypeDef* hfdcan, uint32_t buffer_indexes)
{
	stm32zero::fdcan::process_tx_complete(hfdcan, buffer_indexes);
}

extern "C" void stm32zero_fdcan_process_tx_fifo_empty(FDCAN_HandleTypeDef* hfdcan)
{
	stm32zero::fdcan::process_tx_fifo_empty(hfdcan);
}

extern "C" void stm32zero_fdcan_process_error(FDCAN_HandleTypeDef* hfdcan, uint32_t error_status_its)
{
	stm32zero::fdcan::process_error(hfdcan, error_status_its);
}

#endif // HAL_FDCAN_MODULE_ENABLED
