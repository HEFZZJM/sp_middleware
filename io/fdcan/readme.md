# FDCAN 驱动使用说明 (STM32H7)

`sp::FDCAN` 同时支持 **Classic CAN 2.0** 与 **CAN-FD（最长 64 字节、可选 BRS 变速）**，默认构造为 `CLASSIC` 模式，完全兼容旧代码。

> **总线混挂红线**：只要同一条物理总线上挂了纯 Classic CAN 2.0 设备（如 DJI C620 / C610 / GM6020 / 功率计），**严禁**在该总线上发送任何 CAN-FD 帧，否则经典节点会刷错误帧导致整条总线瘫痪。达妙等支持 CAN-FD 的电机需用上位机统一修改波特率并保存重启后，才能和主控同跑 CAN-FD。

---

## 1. 模式说明

```cpp
enum class FDCANMode
{
  CLASSIC,    // 经典 CAN 2.0 (<= 8 字节, 无 BRS) —— 默认值
  FD_NO_BRS,  // CAN-FD 不变速 (<= 64 字节, 仲裁/数据同速)
  FD_BRS,     // CAN-FD 变速   (<= 64 字节, 数据段切高速)
};
```

- 按总线指定默认模式（推荐）：
  ```cpp
  sp::FDCAN fdcan1(&hfdcan1);                            // 标准can路 CLASSIC (M3508 / GM6020)
  sp::FDCAN fdcan2(&hfdcan2, sp::FDCANMode::FD_BRS);     // 1M/5M CAN-FD (达妙电机)
  sp::FDCAN fdcan3(&hfdcan3, sp::FDCANMode::FD_BRS);     // 1M/5M CAN-FD (双板长帧)
  ```
- 运行时也可以用 `fdcan.set_mode(mode)` 切换默认模式，或用 4 参 `send(id, data, len, mode)` 覆盖单帧模式。

---

## 2. Demo

新建 `applications/fdcan_task.cpp`：

```cpp
#include "cmsis_os.h"
#include "io/fdcan/fdcan.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// CAN1: 经典 CAN 1M (DJI 电机)
sp::FDCAN fdcan1(&hfdcan1);
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020

// CAN2: CAN-FD 1M/5M (达妙电机，上位机需同步设为 5M 并保存重启)
sp::FDCAN fdcan2(&hfdcan2, sp::FDCANMode::FD_BRS);
sp::DM_Motor dm_motor(0x01, 0x11, 3.141593f, 30.0f, 54.0f);

// CAN3: CAN-FD 1M/5M 变长帧 (最多 64 字节，如双板通信)
sp::FDCAN fdcan3(&hfdcan3, sp::FDCANMode::FD_BRS);

extern "C" void fdcan_task()
{
  fdcan1.start();
  fdcan2.start();
  fdcan3.start();

  dm_motor.write_enable(fdcan2.tx_data);
  fdcan2.send(dm_motor.tx_id);  // 默认 8 字节，按 fdcan2 的 FD_BRS 模式发送

  uint8_t payload[24] = {0x01, 0x02, 0x03};

  while (true) {
    // 1) Classic CAN 8 字节发送 (与旧版完全兼容)
    motor6020.cmd(0.0f);  // 调整为 0.1f, 电机会旋转, 注意安全
    motor6020.write(fdcan1.tx_data);
    fdcan1.send(motor6020.tx_id);

    // 2) CAN-FD 达妙 8 字节发送 (DM_Motor 驱动代码无需修改)
    dm_motor.cmd(0.0f);
    dm_motor.write(fdcan2.tx_data);
    fdcan2.send(dm_motor.tx_id);

    // 3) CAN-FD 变长帧发送 (直接传指针，非标准 DLC 长度会自动补零对齐到 12/16/20/24/32/48/64)
    fdcan3.send(0x310, payload, sizeof(payload));

    osDelay(1);
  }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) return;
  auto stamp_ms = osKernelSysTick();

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    if (hfdcan == &hfdcan1 && fdcan1.recv()) {
      if (fdcan1.rx_id == motor6020.rx_id) motor6020.read(fdcan1.rx_data, stamp_ms);
    }
    else if (hfdcan == &hfdcan2 && fdcan2.recv()) {
      if (fdcan2.rx_id == dm_motor.rx_id) dm_motor.read(fdcan2.rx_data, stamp_ms);
    }
    else if (hfdcan == &hfdcan3 && fdcan3.recv()) {
      // 可通过 fdcan3.rx_len / fdcan3.rx_is_fd / fdcan3.rx_brs 获取接收帧长度与属性
    }
  }
}

// 可选但强烈建议：Bus-Off 自动恢复
extern "C" void HAL_FDCAN_ErrorStatusCallback(
  FDCAN_HandleTypeDef * hfdcan, uint32_t ErrorStatusITs)
{
  if (hfdcan == &hfdcan1) fdcan1.on_error_status(ErrorStatusITs);
  else if (hfdcan == &hfdcan2) fdcan2.on_error_status(ErrorStatusITs);
  else if (hfdcan == &hfdcan3) fdcan3.on_error_status(ErrorStatusITs);
}
```

---

## 3. 常用接口与诊断字段

| 接口 / 字段 | 说明 |
|---|---|
| `tx_data[64]` / `rx_data[64]` | 64 字节收发缓冲区 |
| `send(tx_id, len = 8)` | 用 `tx_data` 和总线默认模式发帧；返回 `false` 时累加 `tx_drop_cnt` |
| `send(tx_id, data, len)` | 直接发送指定缓冲区（CAN-FD 非标准长度如 22B 会自动补零到 24B；`CLASSIC` 超过 8B 返回 `false`） |
| `send(tx_id, data, len, mode)` | 单帧临时指定模式 |
| `recv()` | 从 RX FIFO0 取一帧，更新 `rx_id`、`rx_len`（0..64）、`rx_is_fd`、`rx_brs` |
| `on_error_status(ErrorStatusITs)` | 在 `HAL_FDCAN_ErrorStatusCallback` 中调用，Bus-Off 时自动清 `CCCR.INIT` 恢复并累加 `bus_off_cnt` |
| `update_error_counters()` | 更新 `tx_error_cnt`、`rx_error_cnt`、`last_error_code`（仲裁段）、`data_last_error_code`（数据段） |

> `last_error_code` / `data_last_error_code` 编码：`0` 无错、`1` Stuff、`2` Form、`3` Ack（对端无应答）、`4` Bit1、`5` Bit0、`6` CRC、`7` 无变化。

---

## 4. CubeMX 配置要点 (以 120 MHz FDCAN 时钟、1M / 5M 为例)

1. **`Parameter Settings -> Basic Parameters`**：
   - `Frame Format`: `FDCAN_FRAME_FD_BRS`
   - `Auto Retransmission`: `Disable`（按需可开），`Protocol Exception`: `Disable`
   - 仲裁段 1 Mbps (75%)：`NominalPrescaler = 3`, `NominalTimeSeg1 = 29`, `NominalTimeSeg2 = 10`, `NominalSyncJumpWidth = 10`
   - 数据段 5 Mbps (75%)：`DataPrescaler = 1`, `DataTimeSeg1 = 17`, `DataTimeSeg2 = 6`, `DataSyncJumpWidth = 6`
     - 若需先用 2 Mbps (75%) 调试：`DataPrescaler = 3`, `DataTimeSeg1 = 14`, `DataTimeSeg2 = 5`, `DataSyncJumpWidth = 5`（2 Mbps 下无需开 TDC）
   - FIFO 与 Message RAM（三路共用 2560 words，64B 元素每格占 18 words）：
     - `RxFifo0ElmtsNbr = 16`, `RxFifo1ElmtsNbr = 0`, `TxFifoQueueElmtsNbr = 16`, `ElmtSize = 64 bytes`
     - `MessageRAMOffset`：FDCAN1 = `0`，FDCAN2 = `0x350`，FDCAN3 = `0x6A0`
2. **`GPIO Settings`**：所有 FDCAN TX/RX 引脚的 `Maximum output speed` 必须设为 **`Very High`**。
3. **开启 TDC（5 Mbps 必配，CubeMX 界面无此项）**：生成代码后在 `Core/Src/fdcan.c` 对应实例的 `USER CODE BEGIN FDCANx_Init 2` 中加入：
   ```c
   if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, hfdcan2.Init.DataPrescaler * hfdcan2.Init.DataTimeSeg1, 0) != HAL_OK)
   {
     Error_Handler();
   }
   if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2) != HAL_OK)
   {
     Error_Handler();
   }
   ```

---

## 5. `CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/fdcan_task.cpp
    sp_middleware/io/fdcan/fdcan.cpp
    sp_middleware/motor/rm_motor/rm_motor.cpp
    sp_middleware/motor/dm_motor/dm_motor.cpp
    sp_middleware/tools/math_tools/math_tools.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/
)
```
