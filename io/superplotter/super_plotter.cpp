#include "super_plotter.hpp"

#include "cmsis_os.h"

namespace sp
{
namespace
{
// 由构造函数赋值. record() 是静态入口, 拿不到实例, 只能走这个全局指针.
SuperPlotter * g_instance = nullptr;

// 注册在程序生命周期里最多发生 kMaxTopics 次, 所以这里关中断很便宜.
// 保存并恢复 PRIMASK, 以便在别的临界区里安全嵌套.
uint32_t plot_lock()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void plot_unlock(uint32_t primask) { __set_PRIMASK(primask); }
}  // namespace

SuperPlotter::SuperPlotter(
  UART_HandleTypeDef * huart, bool use_dma, uint32_t period_ms, uint32_t matrix_period_ms)
: huart_(huart),
  use_dma_(use_dma),
  period_ms_(period_ms == 0 ? PLOT_PERIOD_MS : period_ms),
  matrix_period_ms_(matrix_period_ms == 0 ? PLOT_MATRIX_PERIOD_MS : matrix_period_ms)
{
  table_.set_lock(&plot_lock, &plot_unlock);
  g_instance = this;
  // 打开 DWT 周期计数器, 用来量一帧的线上时间.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  // pl/tx_age 的起点定在构造时, 否则第一拍会把整段上电时间报成卡顿.
  tx_done_ms_ = HAL_GetTick();
}

void SuperPlotter::record_runtime(const char * name, float value, PlotUnit unit)
{
  if (g_instance == nullptr) return;
  g_instance->write(name, value, unit);
}

void SuperPlotter::record_matrix_runtime(
  const char * name, uint8_t rows, uint8_t cols, const float * values, PlotUnit unit)
{
  if (g_instance == nullptr) return;
  g_instance->table_.record_matrix(name, rows, cols, values, unit);
}

void SuperPlotter::write(const char * name, float value, PlotUnit unit)
{
  table_.record(name, value, unit);
}

PlotPublisher SuperPlotter::publish_runtime(const char * topic, PlotUnit unit)
{
  return table_.publish(topic, unit);
}

MatrixPublisher SuperPlotter::publish_matrix_runtime(
  const char * topic, uint8_t rows, uint8_t cols, PlotUnit unit)
{
  return table_.publish_matrix(topic, rows, cols, unit);
}

void SuperPlotter::poll()
{
  while (ring_r_ != ring_w_) {
    const uint8_t byte = ring_[ring_r_ & 0xff];
    ring_r_ = static_cast<uint16_t>(ring_r_ + 1);
    table_.ingest(&byte, 1);
  }
}

void SuperPlotter::flush()
{
  poll();
  if (huart_ == nullptr) return;

  // 同一时刻只允许一帧在飞.
  if (tx_busy_) {
    // HAL 会在调完成回调之前把 gState 置回 READY, 而回调是中断、任务插不进去,
    // 所以这里看到 READY 说明那次传输走了错误路径、没报完成. 44 字节一帧只要
    // 0.48 ms, 10 ms 还没完就是中断丢了. 两种情况都丢掉这一帧让下一拍重试,
    // 而不是让绘图从此停掉.
    if (huart_->gState == HAL_UART_STATE_READY) {
      tx_busy_ = false;
      tx_announce_ = 0;
      ++tx_fail_;
    }
    else if (DWT->CYCCNT - tx_start_cycles_ > SystemCoreClock / 100u) {
      HAL_UART_AbortTransmit(huart_);
      tx_busy_ = false;
      tx_announce_ = 0;
      ++tx_fail_;
    }
    return;
  }
  if (huart_->gState != HAL_UART_STATE_READY) return;

  // 名字一次只发一个. 之前一次发整批约 180 字节, 在这条链路上被截断过;
  // 单个名字最长 22 字节 (15 字符矩阵路径加行列和单位).
  constexpr size_t kAnnounceBurst = 24;
  // 8 个值 = 44 字节 = 线上 0.48 ms, 1 ms 周期下链路占一半, 帧之间还有余量给
  // 适配器. 16 个值会占到 91%, 链路撑不住.
  constexpr size_t kSampleBurst = 3 + 5 * 8 + 1;

  const size_t announced = table_.pop_announce(tx_, kAnnounceBurst);
  if (announced > 0) {
    // 先存下来再启动传输: 完成中断可能在 send_bytes() 返回前就进来.
    tx_announce_ = announced;
    if (!send_bytes(tx_, static_cast<uint16_t>(announced))) tx_announce_ = 0;
    return;
  }

  // 矩阵走自己的慢节拍. 到期时它占这一拍的帧, 标量等下一拍;
  // 矩阵帧大得多, 混在同一帧里需要更大的缓冲.
  const uint32_t now = osKernelSysTick();
  if (now - last_matrix_ms_ >= matrix_period_ms_) {
    const size_t packed = table_.pop_matrix_sample(tx_, PLOT_TX_SIZE);
    if (packed > 0) {
      last_matrix_ms_ = now;
      send_bytes(tx_, static_cast<uint16_t>(packed));
      return;
    }
  }

  const size_t sampled = table_.pop_sample(tx_, kSampleBurst);
  if (sampled > 0) send_bytes(tx_, static_cast<uint16_t>(sampled));
}

void SuperPlotter::initialize() { request(); }

void SuperPlotter::loop(TickFn record_hook)
{
  // 固定节拍: 从醒来的时刻排下一次, 偶尔超时的钩子不会把采样率推走.
  uint32_t wake = osKernelSysTick();
  uint32_t iterations = 0;

  for (;;) {
    ++iterations;
    for (int i = 0; i < row_count_; ++i) rows_[i].pub.set(rows_[i].read());

    // 矩阵和标量一样每拍采样, 由 flush() 决定多久发一次. 缓冲在循环里声明,
    // 某个 fill() 少写了格子也不会漏进要发的那一帧.
    if (matrix_row_count_ > 0) {
      float cells[PLOT_MATRIX_MAX_CELLS];
      for (int i = 0; i < matrix_row_count_; ++i) {
        matrices_[i].fill(cells);
        matrices_[i].pub.set(cells);
      }
    }

    // 自诊断, 三条一起看: 曲线没了是链路丢帧 (tx_fail)、链路塞满
    // (tx_us 对上一个周期) 还是循环本身落后 (loop).
    record_runtime("pl/tx_fail", static_cast<float>(tx_fail_));
    record_runtime("pl/tx_us", static_cast<float>(last_tx_us_));
    record_runtime("pl/tx_max", static_cast<float>(max_tx_us_));
    record_runtime("pl/loop", static_cast<float>(iterations & 0xFFFFFFu));
    // 已注册槽数和主机勾选数. pl/topics 卡在 PLOT_MAX_TOPICS 说明 insert()
    // 把后面的静默丢了; pl/subs 超出链路能不能带走, 表现为每路刷新变慢.
    record_runtime("pl/topics", static_cast<float>(table_.size()));
    record_runtime("pl/subs", static_cast<float>(table_.subscribed_count()));

    // 距上一帧真正发出去过了多少 ms, 直接量"绘图任务多久没被调度".
    // 尖峰配 pl/tx_fail 不动 = 任务被抢占 (CPU 问题); 配 tx_fail 上涨 = 那一帧
    // 没发完被丢了 (链路问题). 两者都是 0 而上位机还在卡, 就不是绘图器的问题.
    // 分辨率是 1 ms 的 HAL 节拍, 用来抓卡顿而不是看抖动.
    const uint32_t age_ms = HAL_GetTick() - tx_done_ms_;
    if (age_ms > max_tx_age_ms_) max_tx_age_ms_ = age_ms;
    record_runtime("pl/tx_age", static_cast<float>(age_ms), PlotUnit::Millisecond);
    record_runtime("pl/tx_age_max", static_cast<float>(max_tx_age_ms_), PlotUnit::Millisecond);

    if (record_hook != nullptr) record_hook();
    flush();
    osDelayUntil(&wake, period_ms_);
  }
}

void SuperPlotter::request()
{
  if (huart_ == nullptr) return;
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buff_, sizeof rx_buff_);
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(huart_, rx_buff_, sizeof rx_buff_);
  }
}

void SuperPlotter::on_rx(uint16_t size)
{
  if (size > sizeof rx_buff_) size = sizeof rx_buff_;
  for (uint16_t i = 0; i < size; ++i) {
    ring_[ring_w_ & 0xff] = rx_buff_[i];
    ring_w_ = static_cast<uint16_t>(ring_w_ + 1);
  }
}

// 把一帧交给 UART 就返回: 字节在中断里发出, 所以 1 kHz 的循环只花几微秒,
// 而不是一帧 44 字节在线上需要的 0.5 ms. tx_ 一直有效到 on_tx_complete().
bool SuperPlotter::send_bytes(const uint8_t * data, uint16_t size)
{
  if (huart_ == nullptr || data == nullptr || size == 0) return false;
  // 调用之前就置位: 第一个中断可能在调用内部就进来.
  tx_busy_ = true;
  tx_start_cycles_ = DWT->CYCCNT;
  const HAL_StatusTypeDef status =
    use_dma_ ? HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t *>(data), size)
             : HAL_UART_Transmit_IT(huart_, const_cast<uint8_t *>(data), size);
  if (status != HAL_OK) {
    tx_busy_ = false;
    ++tx_fail_;
    return false;
  }
  return true;
}

void SuperPlotter::on_tx_complete()
{
  if (huart_ == nullptr) return;
  if (tx_busy_) {
    // 整个传输期间 CPU 是空闲的, 所以这里量到的是线上时间, 不是绘图的 CPU 开销.
    const uint32_t cycles = DWT->CYCCNT - tx_start_cycles_;
    last_tx_us_ = cycles / (SystemCoreClock / 1000000u);
    if (last_tx_us_ > max_tx_us_) max_tx_us_ = last_tx_us_;
    // pl/tx_age 的时间戳. HAL_GetTick() 只读 SysTick, 在完成中断里是安全的,
    // 而且 49 天才回绕, 不像 DWT 周期计数器 8.9 秒就绕一圈.
    tx_done_ms_ = HAL_GetTick();
    // 名字公告要等字节真的发出去才标记: 中途被打断的帧会重试, 而不是让主机
    // 看到一个残缺的名字.
    if (tx_announce_ != 0) {
      table_.commit_announce(tx_announce_);
      tx_announce_ = 0;
    }
    tx_busy_ = false;
  }
}

}  // namespace sp
