#ifndef SP__FDCAN_HPP
#define SP__FDCAN_HPP

#include <cstddef>
#include <cstdint>

#include "fdcan.h"

namespace sp
{
constexpr size_t FDCAN_DATA_LEN = 64;

enum class FDCANMode
{
  CLASSIC,    // Classic CAN 2.0 (<= 8 bytes, BRS off)
  FD_NO_BRS,  // CAN-FD without bit-rate switching (<= 64 bytes)
  FD_BRS,     // CAN-FD with bit-rate switching (<= 64 bytes)
};

class FDCAN
{
public:
  explicit FDCAN(FDCAN_HandleTypeDef * hfdcan, FDCANMode mode = FDCANMode::CLASSIC);

  uint16_t rx_id = 0;
  uint8_t rx_len = 0;
  bool rx_is_fd = false;
  bool rx_brs = false;
  uint8_t rx_data[FDCAN_DATA_LEN] = {};
  uint8_t tx_data[FDCAN_DATA_LEN] = {};

  uint32_t tx_error_cnt = 0;
  uint32_t rx_error_cnt = 0;
  uint32_t bus_off_cnt = 0;
  uint32_t tx_drop_cnt = 0;
  uint32_t last_error_code = 7;
  uint32_t data_last_error_code = 7;

  void start();
  bool recv();

  bool send(uint16_t tx_id, uint8_t len = 8);
  bool send(uint16_t tx_id, const uint8_t * data, uint8_t len);
  bool send(uint16_t tx_id, const uint8_t * data, uint8_t len, FDCANMode mode);

  void set_mode(FDCANMode mode);
  FDCANMode mode() const { return mode_; }

  void on_error_status(uint32_t error_status_its);
  void update_error_counters();

  static uint32_t bytes_to_dlc(uint8_t len, uint8_t * padded_len = nullptr);
  static uint8_t dlc_to_bytes(uint32_t dlc);

private:
  FDCAN_HandleTypeDef * hfdcan_;
  FDCANMode mode_;
  FDCAN_RxHeaderTypeDef rx_header_{};
  FDCAN_TxHeaderTypeDef tx_header_{};
};

}  // namespace sp

#endif  // SP__FDCAN_HPP
