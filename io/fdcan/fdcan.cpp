#include "fdcan.hpp"

#include <cstring>

namespace sp
{
FDCAN::FDCAN(FDCAN_HandleTypeDef * hfdcan, FDCANMode mode) : hfdcan_(hfdcan), mode_(mode)
{
  tx_header_.IdType = FDCAN_STANDARD_ID;
  tx_header_.TxFrameType = FDCAN_DATA_FRAME;
  tx_header_.DataLength = FDCAN_DLC_BYTES_8;
  tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header_.MessageMarker = 0x00;
  set_mode(mode);
}

void FDCAN::set_mode(FDCANMode mode)
{
  mode_ = mode;
  switch (mode_) {
    case FDCANMode::CLASSIC:
      tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
      tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
      break;
    case FDCANMode::FD_NO_BRS:
      tx_header_.FDFormat = FDCAN_FD_CAN;
      tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
      break;
    case FDCANMode::FD_BRS:
      tx_header_.FDFormat = FDCAN_FD_CAN;
      tx_header_.BitRateSwitch = FDCAN_BRS_ON;
      break;
  }
}

void FDCAN::start()
{
  HAL_FDCAN_Start(hfdcan_);                                                   // dismiss return
  HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // dismiss return
  HAL_FDCAN_ActivateNotification(
    hfdcan_, FDCAN_IT_BUS_OFF,
    0);  // dismiss return ref: https://community.st.com/t5/stm32-mcus-products/stm32h7-fdcan-has-lost-the-automatic-bus-off-recovery-mechanism/td-p/187400?utm_source=chatgpt.com
}

bool FDCAN::recv()
{
  if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header_, this->rx_data) != HAL_OK) {
    return false;
  }
  this->rx_id = rx_header_.Identifier;
  this->rx_len = dlc_to_bytes(rx_header_.DataLength);
  this->rx_is_fd = (rx_header_.FDFormat == FDCAN_FD_CAN);
  this->rx_brs = (rx_header_.BitRateSwitch == FDCAN_BRS_ON);
  return true;
}

bool FDCAN::send(uint16_t tx_id, uint8_t len) { return send(tx_id, this->tx_data, len, mode_); }

bool FDCAN::send(uint16_t tx_id, const uint8_t * data, uint8_t len)
{
  return send(tx_id, data, len, mode_);
}

bool FDCAN::send(uint16_t tx_id, const uint8_t * data, uint8_t len, FDCANMode mode)
{
  if (len > FDCAN_DATA_LEN || (mode == FDCANMode::CLASSIC && len > 8) || (len > 0 && data == nullptr)) {
    ++tx_drop_cnt;
    return false;
  }

  uint8_t padded_len = 0;
  FDCAN_TxHeaderTypeDef header = tx_header_;
  header.Identifier = tx_id;
  header.DataLength = bytes_to_dlc(len, &padded_len);

  switch (mode) {
    case FDCANMode::CLASSIC:
      header.FDFormat = FDCAN_CLASSIC_CAN;
      header.BitRateSwitch = FDCAN_BRS_OFF;
      break;
    case FDCANMode::FD_NO_BRS:
      header.FDFormat = FDCAN_FD_CAN;
      header.BitRateSwitch = FDCAN_BRS_OFF;
      break;
    case FDCANMode::FD_BRS:
      header.FDFormat = FDCAN_FD_CAN;
      header.BitRateSwitch = FDCAN_BRS_ON;
      break;
  }

  uint8_t padded_buf[FDCAN_DATA_LEN];
  uint8_t * tx_ptr = const_cast<uint8_t *>(data);
  if (padded_len > len) {
    if (len > 0) std::memcpy(padded_buf, data, len);
    std::memset(padded_buf + len, 0, padded_len - len);
    tx_ptr = padded_buf;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &header, tx_ptr) != HAL_OK) {
    ++tx_drop_cnt;
    return false;
  }
  return true;
}

void FDCAN::on_error_status(uint32_t error_status_its)
{
  if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U) {
    ++bus_off_cnt;
    CLEAR_BIT(hfdcan_->Instance->CCCR, FDCAN_CCCR_INIT);
  }
}

void FDCAN::update_error_counters()
{
  FDCAN_ErrorCountersTypeDef counters{};
  if (HAL_FDCAN_GetErrorCounters(hfdcan_, &counters) == HAL_OK) {
    tx_error_cnt = counters.TxErrorCnt;
    rx_error_cnt = counters.RxErrorCnt;
  }
  FDCAN_ProtocolStatusTypeDef status{};
  if (HAL_FDCAN_GetProtocolStatus(hfdcan_, &status) == HAL_OK) {
    if (status.LastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE) {
      last_error_code = status.LastErrorCode;
    }
    if (status.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE) {
      data_last_error_code = status.DataLastErrorCode;
    }
  }
}

uint32_t FDCAN::bytes_to_dlc(uint8_t len, uint8_t * padded_len)
{
  static constexpr uint32_t kSmallDlc[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
  };

  if (len <= 8) {
    if (padded_len) *padded_len = len;
    return kSmallDlc[len];
  }
  if (len <= 12) {
    if (padded_len) *padded_len = 12;
    return FDCAN_DLC_BYTES_12;
  }
  if (len <= 16) {
    if (padded_len) *padded_len = 16;
    return FDCAN_DLC_BYTES_16;
  }
  if (len <= 20) {
    if (padded_len) *padded_len = 20;
    return FDCAN_DLC_BYTES_20;
  }
  if (len <= 24) {
    if (padded_len) *padded_len = 24;
    return FDCAN_DLC_BYTES_24;
  }
  if (len <= 32) {
    if (padded_len) *padded_len = 32;
    return FDCAN_DLC_BYTES_32;
  }
  if (len <= 48) {
    if (padded_len) *padded_len = 48;
    return FDCAN_DLC_BYTES_48;
  }
  if (padded_len) *padded_len = 64;
  return FDCAN_DLC_BYTES_64;
}

uint8_t FDCAN::dlc_to_bytes(uint32_t dlc)
{
  switch (dlc) {
    case FDCAN_DLC_BYTES_0:  return 0;
    case FDCAN_DLC_BYTES_1:  return 1;
    case FDCAN_DLC_BYTES_2:  return 2;
    case FDCAN_DLC_BYTES_3:  return 3;
    case FDCAN_DLC_BYTES_4:  return 4;
    case FDCAN_DLC_BYTES_5:  return 5;
    case FDCAN_DLC_BYTES_6:  return 6;
    case FDCAN_DLC_BYTES_7:  return 7;
    case FDCAN_DLC_BYTES_8:  return 8;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default:                 return 0;
  }
}

}  // namespace sp
