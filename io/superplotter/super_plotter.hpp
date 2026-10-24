#ifndef SP__SUPER_PLOTTER_HPP
#define SP__SUPER_PLOTTER_HPP

#include <type_traits>

#include "plot_link.hpp"
#include "usart.h"

namespace sp
{
// 帧缓冲大小. 一帧标量是 3 + 5*k 字节, 所以它决定一帧能带几个值;
// 订阅数超出一帧时分多帧轮流发, 不丢数据.
constexpr int PLOT_TX_SIZE = 192;
static_assert(PLOT_TX_SIZE >= 3 + 5 * 4, "tx_ must carry at least four values");
static_assert(PLOT_MAX_TOPICS <= PLOT_MAX_ID, "topic id is one byte on the wire");

// loop() 的采样周期, 单位 ms. 发送是中断驱动的, 所以这是采样节拍, 不是帧在线上的时间.
// 只是默认值, 构造函数可以改: 订阅集超出链路带宽时把周期调大即可.
constexpr uint32_t PLOT_PERIOD_MS = 1;

// 矩阵帧比标量帧重得多 (3x1 当标量只要 5 字节, 当矩阵帧要 19 字节, 4x4 要 71 字节),
// 所以单独走一档慢节拍. 同样是默认值, 构造函数可以改.
constexpr uint32_t PLOT_MATRIX_PERIOD_MS = 10;

// 命名主题 + 主机订阅的绘图器.
//
// 和旧的 Plotter (固定 AA BB | size | float[18], 没有名字) 的区别: 主机从 AA BC
// 公告里学到所有主题名, 再按用户勾选发 AA BD 只订阅要看的那些. 一帧因此是每个
// 主题 5 字节, 而不是永远发 18 个 float.
//
// 常用写法是静态的 record(), 任意任务随时可调, 第一次调用时注册主题:
//
//   sp::SuperPlotter::record("arm/j0/pos", arm_j0.pos, sp::PlotUnit::Radian);
//
// 单位只是主机坐标轴的标签, 不做换算, 且由第一次注册它的调用固定下来.
//
// 注册很便宜 (一个槽位), 路径可以永久留在代码里, 只有被勾选的主题占串口带宽.
// 同一个量在循环里高频写时, 用 publish() 拿句柄可以省掉每次查表.
//
// 发送是中断驱动的: flush() 把一帧交给 UART 就返回, 耗时几微秒, 1 kHz 调用也不占
// 帧在线上的时间. 上一帧还没发完时, 这一拍直接跳过, 等发送完成中断释放缓冲.
//
// 只用来打点的任务写成两句, record() 放在循环钩子里:
//
//   super_plotter.initialize();
//   super_plotter.loop([]{ sp::SuperPlotter::record("arm/j0/pos", arm_j0.pos); });
class SuperPlotter
{
public:
  // period_ms: loop() 的节拍, 也是最多多久交一帧给 UART. 1 = 原来的 1 kHz,
  // 调大则牺牲分辨率换链路余量. 传 0 按 PLOT_PERIOD_MS 处理.
  //
  // matrix_period_ms: 矩阵帧的发送节拍, 和标量采样无关 (矩阵帧大得多, 塞不进同样的
  // 节奏). 传 0 按 PLOT_MATRIX_PERIOD_MS 处理.
  SuperPlotter(
    UART_HandleTypeDef * huart, bool use_dma = true, uint32_t period_ms = PLOT_PERIOD_MS,
    uint32_t matrix_period_ms = PLOT_MATRIX_PERIOD_MS);

  UART_HandleTypeDef * huart() const { return huart_; }

  /** loop() 的节拍, 单位 ms. */
  uint32_t period_ms() const { return period_ms_; }

  /** 矩阵帧的发送节拍, 单位 ms. */
  uint32_t matrix_period_ms() const { return matrix_period_ms_; }

  // 写一个值, 名字第一次出现时注册主题.
  // 转给 plot_task.cpp 里创建的实例, 所以不需要到处传对象, 包含头文件直接调即可.
  //
  // 路径必须是字面量, 长度才能编译期确定; 超过 PLOT_MAX_NAME 是编译错误,
  // 不会截断 (截断会把两个不同路径并成一个字段). 单位只是主机坐标轴的标签,
  // 数值始终按记录时的单位原样发送.
  template <size_t N, typename T>
  static std::enable_if_t<std::is_arithmetic<T>::value, void> record(const char (&name)[N], T value)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    record_runtime(name, static_cast<float>(value));
  }

  template <size_t N, typename T>
  static std::enable_if_t<std::is_arithmetic<T>::value, void> record(
    const char (&name)[N], T value, PlotUnit unit)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    record_runtime(name, static_cast<float>(value), unit);
  }

  // 内部入口, 给已经自己检查过字面量的表用. 这里没有编译期长度检查,
  // 但 insert() 会拒收超长名字, 所以仍然不会截断.
  static void record_runtime(const char * name, float value, PlotUnit unit = PlotUnit::None);

  // 句柄形式: 注册一次, 之后 set() 只是一次 float 写入.
  template <size_t N>
  PlotPublisher publish(const char (&name)[N])
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    return publish_runtime(name);
  }

  template <size_t N>
  PlotPublisher publish(const char (&name)[N], PlotUnit unit)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    return publish_runtime(name, unit);
  }

  // publish() 的内部版本, 规则同 record_runtime().
  PlotPublisher publish_runtime(const char * topic, PlotUnit unit = PlotUnit::None);

  // 矩阵形式: values 里是 rows*cols 个 float, 行优先, 会被拷进槽位, 所以传临时
  // 缓冲也可以. 行列由第一次注册固定, 之后用不同尺寸调用会被忽略, 不会错误解读.
  //
  // 3D 向量、点、小变换用这个; 更大的数据应拆成几个字段.
  template <size_t N>
  static void record_matrix(
    const char (&name)[N], uint8_t rows, uint8_t cols, const float * values,
    PlotUnit unit = PlotUnit::None)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    record_matrix_runtime(name, rows, cols, values, unit);
  }

  static void record_matrix_runtime(
    const char * name, uint8_t rows, uint8_t cols, const float * values,
    PlotUnit unit = PlotUnit::None);

  template <size_t N>
  MatrixPublisher publish_matrix(
    const char (&name)[N], uint8_t rows, uint8_t cols, PlotUnit unit = PlotUnit::None)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    return publish_matrix_runtime(name, rows, cols, unit);
  }

  MatrixPublisher publish_matrix_runtime(
    const char * topic, uint8_t rows, uint8_t cols, PlotUnit unit = PlotUnit::None);

  // 在串口上开接收. 在 plot 任务里调一次, 放在 loop() 之前, 否则收不到订阅帧.
  void initialize();

  // 同上, 并把整张轮询表注册一遍; 之后 loop() 每拍采样表中的每一行.
  // N 从数组本身推出来, 调用方不用传数量.
  template <size_t N>
  void initialize(PolledRow (&rows)[N])
  {
    initialize();
    for (size_t i = 0; i < N; ++i) rows[i].pub = publish_runtime(rows[i].name, rows[i].unit);
    rows_ = rows;
    row_count_ = static_cast<int>(N);
  }

  // 矩阵的轮询表, 在 initialize() 之后调. 每拍采样, flush() 按 matrix_period_ms 发送.
  template <size_t N>
  void initialize_matrices(PolledMatrix (&matrices)[N])
  {
    for (size_t i = 0; i < N; ++i) {
      matrices[i].pub = publish_matrix_runtime(matrices[i].name, matrices[i].rows, matrices[i].cols, matrices[i].unit);
    }
    matrices_ = matrices;
    matrix_row_count_ = static_cast<int>(N);
  }

  // 每拍的用户代码. 无捕获的 lambda 能直接转成函数指针, 所以 record() 可以留在
  // 它描述的那些值旁边. 可选: 只采样轮询表的任务直接调 loop() 就行.
  using TickFn = void (*)();

  // 按 period_ms 节拍一直跑: 采样轮询表, 调 record_hook, 再交最多一帧给 UART.
  // 不返回; 只打点的任务就是 initialize() + loop() 两句.
  void loop(TickFn record_hook = nullptr);

  void poll();
  void flush();
  void request();
  void on_rx(uint16_t size);

  // 在 HAL_UART_TxCpltCallback() 里调, 一帧真正发完了才进来.
  // 把这一帧里的公告标记为已发, 并释放缓冲.
  void on_tx_complete();

  // 被 UART 拒收, 或者因为完成中断没来而丢掉的帧数. 非 0 说明是链路在丢数据,
  // 不是主机解析慢.
  uint32_t tx_fail() const { return tx_fail_; }

  // 上一帧在线上的时间, 单位 us. 期间 CPU 是空闲的, 所以这是
  // 字节数 * 10 / 波特率, 不是绘图的 CPU 开销; 和帧周期比就是链路占用率.
  uint32_t last_tx_us() const { return last_tx_us_; }
  uint32_t max_tx_us() const { return max_tx_us_; }

  // 距上一帧发完过了多少 ms, 以及上电以来最坏的一次. 和 pl/tx_age /
  // pl/tx_age_max 是同一组数, 用来在没有链路时通过调试器直接读.
  uint32_t tx_age_ms() const { return HAL_GetTick() - tx_done_ms_; }
  uint32_t max_tx_age_ms() const { return max_tx_age_ms_; }

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;
  // loop() 节拍, 单位 ms. 不会是 0, 构造时已兜底.
  uint32_t period_ms_;
  // 矩阵发送节拍, 单位 ms. 同样不会是 0.
  uint32_t matrix_period_ms_;
  TopicTable table_;
  // initialize() 交进来的轮询表, 由 loop() 采样.
  PolledRow * rows_ = nullptr;
  int row_count_ = 0;
  PolledMatrix * matrices_ = nullptr;
  int matrix_row_count_ = 0;
  uint32_t last_matrix_ms_ = 0;
  uint8_t rx_buff_[128]{};
  uint8_t ring_[256]{};
  volatile uint16_t ring_w_ = 0;
  volatile uint16_t ring_r_ = 0;
  uint8_t tx_[PLOT_TX_SIZE]{};
  // 同一时刻只有一帧在飞: tx_ 交给 UART 后, 完成中断释放之前不能再写.
  volatile bool tx_busy_ = false;
  // 在飞的那一帧里包含多少字节的公告. 由 on_tx_complete() 标记为已发,
  // 这样没发出去的帧会被重试而不是丢掉.
  volatile size_t tx_announce_ = 0;
  uint32_t tx_start_cycles_ = 0;
  volatile uint32_t tx_fail_ = 0;
  volatile uint32_t last_tx_us_ = 0;
  volatile uint32_t max_tx_us_ = 0;
  // 上一帧发完的时刻, 以及上电以来最大的间隔. 对应 pl/tx_age / pl/tx_age_max.
  volatile uint32_t tx_done_ms_ = 0;
  volatile uint32_t max_tx_age_ms_ = 0;

  void write(const char * name, float value, PlotUnit unit);
  bool send_bytes(const uint8_t * data, uint16_t size);
};

}  // namespace sp

#endif  // SP__SUPER_PLOTTER_HPP
