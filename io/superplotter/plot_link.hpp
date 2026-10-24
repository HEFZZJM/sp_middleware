#ifndef SP__PLOT_LINK_HPP
#define SP__PLOT_LINK_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

// sp::SuperPlotter 的线上格式: 命名主题, 只发被订阅的. (不是 NetworkTables)
//
// 板 -> 主机  AA BC | id u8 | name, NUL | unit u8                  每个主题在首次 flush 时发一次
// 主机 -> 板  AA BD | op u8 | name, NUL                           1 订阅 2 退订 3 全订阅 4 清空
// 板 -> 主机  AA BB | count u8 | (id u8, f32le)* | xor u8          标量样本
//                   xor 覆盖前面所有字节, 校验失败的帧整帧丢弃.
//
// 矩阵是第二组帧: 24 字节的标量槽位放不下可变数量的格子.
//
// 板 -> 主机  AA BE | id u8 | rows u8 | cols u8 | name, NUL | unit u8
// 板 -> 主机  AA BF | count u8 | (id u8, rows u8, cols u8, f32le[rows*cols])* | xor u8
//
// 格子按行优先. rows/cols 在每个样本帧里重复, 这样漏了公告的主机也能解析.
// 订阅用的还是同一套 AA BD 名字, 一个名字只属于两张表之一.
//
// unit 是 PlotUnit 的一个字节, 紧跟名字发送: 只是主机坐标轴的标签, 不做换算,
// 数值始终按记录时的单位发出. 每个主题只多花一个字节, 而且只发一次.
//
// "注册的主题数"和"发送的值数"是两个独立的限制. 一个主题在主机订阅之前只占 RAM;
// 帧由被订阅的子集拼成, 装不下就拆成多帧, 所以注册数只受 RAM 和 u8 id 空间限制.
//
// 路径最长 15 字符 (PLOT_MAX_NAME = 16, 含结尾 NUL). SuperPlotter::record()
// 只接受字面量, 超长的在编译期就报错, 运行期永不截断.

#if defined(__GNUC__)
#define SP_PLOT_BARRIER() __asm volatile("" ::: "memory")
#else
#define SP_PLOT_BARRIER() ((void)0)
#endif

// 最多注册多少个主题. 受 u8 id 空间 (255) 和 RAM 限制: 每槽 24 字节, 128 槽约 3 KB.
// 板子需要别的数时用 -DSP_PLOT_MAX_TOPICS=<n> 覆盖.
#ifndef SP_PLOT_MAX_TOPICS
#define SP_PLOT_MAX_TOPICS 128
#endif

// 矩阵和标量分开存: 矩阵槽位装整块格子, 所以 PLOT_MATRIX_CELLS 直接乘 RAM.
// 8 槽 × 32 格约 1.2 KB. 一帧最多 32 个 float (135 字节), 放得进 PLOT_TX_SIZE.
// 板子扛得住就用 -DSP_PLOT_MAX_MATRICES=<n> / -DSP_PLOT_MATRIX_CELLS=<n> 覆盖.
#ifndef SP_PLOT_MAX_MATRICES
#define SP_PLOT_MAX_MATRICES 8
#endif
#ifndef SP_PLOT_MATRIX_CELLS
#define SP_PLOT_MATRIX_CELLS 32
#endif

namespace sp
{
constexpr uint8_t PLOT_OP_SUB = 1;
constexpr uint8_t PLOT_OP_UNSUB = 2;
constexpr uint8_t PLOT_OP_ALL = 3;
constexpr uint8_t PLOT_OP_CLEAR = 4;

constexpr int PLOT_MAX_TOPICS = SP_PLOT_MAX_TOPICS;
constexpr int PLOT_MAX_NAME = 16;  // 15 个字符加结尾 NUL
constexpr int PLOT_MAX_ID = 255;

// 公告里带的单位标签. 线上一个字节、槽位里也一个字节, 主题仍然只占 24 字节.
// id 是协议的一部分: 主机用的是同一张表, 所以已有的编号绝不能改, 新单位只能往后加.
//
// 只列上位机真能换算的单位. 它没有对应单位组的量 (力矩 N·m、质量 kg、功率 W ...)
// 请用 PlotUnit::None: 挂一个主机解析不了的标签比没有标签更糟, 因为看起来像是
// 轴已经处理过了.
//
// None 表示"未知或无量纲", 主机会直接不显示单位, 而不是从路径里猜一个.
enum class PlotUnit : uint8_t
{
  None = 0,

  // 长度
  Meter = 1,
  Millimeter = 2,
  Centimeter = 3,
  Inch = 4,
  Foot = 5,

  // 角度
  Radian = 6,
  Degree = 7,
  Revolution = 8,

  // 时间
  Second = 9,
  Millisecond = 10,
  Microsecond = 11,
  Minute = 12,

  // 线速度/加速度
  MeterPerSecond = 13,
  MeterPerSecondSquared = 14,

  // 角速度/角加速度
  RadianPerSecond = 15,
  DegreePerSecond = 16,
  RadianPerSecondSquared = 17,

  // 电学
  Ampere = 18,
  Volt = 19,
  Joule = 20,

  // 温度
  Celsius = 21,
  Kelvin = 22,

  // 其他
  Hertz = 23,
  RevolutionPerMinute = 24,
};

constexpr int PLOT_UNIT_COUNT = 25;

constexpr int PLOT_MAX_MATRICES = SP_PLOT_MAX_MATRICES;
constexpr int PLOT_MATRIX_MAX_CELLS = SP_PLOT_MATRIX_CELLS;

// 矩阵槽位自己持有格子: 发布时拷贝, 所以调用方可以传一个马上失效的栈缓冲.
// 行列和单位一样, 由第一次注册固定.
struct PlotMatrixSlot
{
  char name[PLOT_MAX_NAME];
  uint16_t hash;
  uint8_t rows;
  uint8_t cols;
  PlotUnit unit;
  uint8_t flags;
  float cells[PLOT_MATRIX_MAX_CELLS];
};

class MatrixPublisher
{
public:
  // values 必须是 rows*cols 个 float, 行优先.
  void set(const float * values) const
  {
    if (slot_ == nullptr || values == nullptr) return;
    const size_t count = static_cast<size_t>(slot_->rows) * slot_->cols;
    std::memcpy(slot_->cells, values, count * sizeof(float));
  }

  uint8_t rows() const { return slot_ == nullptr ? 0 : slot_->rows; }
  uint8_t cols() const { return slot_ == nullptr ? 0 : slot_->cols; }

  explicit operator bool() const { return slot_ != nullptr; }

private:
  friend class TopicTable;
  PlotMatrixSlot * slot_ = nullptr;
};

// 按固定节拍轮询的矩阵表中的一行. fill 把 rows*cols 个 float 按行优先写进缓冲.
struct PolledMatrix
{
  const char * name;
  uint8_t rows;
  uint8_t cols;
  PlotUnit unit;
  void (*fill)(float * out);
  MatrixPublisher pub;
};

// 字面量路径能放进 PlotSlot::name 时返回 true (N 含 NUL).
// 15 字符以内的路径会绑定到 SuperPlotter::record() / publish() 的
// const char (&)[N] 重载, 那里 static_assert 这个条件. 运行期名字只有内部表会用,
// 那些表的字面量已经检查过, 而且 insert() 会拒收超长的而不是截断.
template <size_t N>
constexpr bool plot_name_fits(const char (&)[N])
{
  return N <= PLOT_MAX_NAME;
}

// 不小于 2*topics 的最小 2 的幂: TopicTable 的开放寻址索引按它开, 保证装载率
// 不超过 1/2, 探测序列一定能撞到空槽.
constexpr int plot_index_size(int topics, int size = 1)
{
  return size >= 2 * topics ? size : plot_index_size(topics, size * 2);
}

struct PlotSlot
{
  char name[PLOT_MAX_NAME];
  float value;
  uint16_t hash;
  PlotUnit unit;
  // bit0 已订阅, bit1 已公告. 这样打包整个槽位正好 24 字节: 16 + 4 + 2 + 1 + 1.
  uint8_t flags;
};

// 单位那一字节不能涨 RAM: 槽位必须还是 24 字节.
static_assert(sizeof(PlotSlot) == 24, "PlotSlot grew; check the field packing");

constexpr uint8_t PLOT_SLOT_SUBSCRIBED = 1u << 0;
constexpr uint8_t PLOT_SLOT_ANNOUNCED = 1u << 1;

class PlotPublisher
{
public:
  void set(float value) const
  {
    if (slot_ != nullptr) slot_->value = value;
  }

  explicit operator bool() const { return slot_ != nullptr; }

private:
  friend class TopicTable;
  PlotSlot * slot_ = nullptr;
};

// 按固定节拍轮询的标量表中的一行: 路径、读值的函数、注册好的发布句柄.
// 放在这里而不是 super_plotter.hpp, 是为了让主机侧的桥接程序能共用表类型
// 又不必引入 HAL.
struct PolledRow
{
  const char * name;
  float (*read)();
  PlotUnit unit;
  PlotPublisher pub;
};

class TopicTable
{
public:
  static constexpr int kMaxTopics = PLOT_MAX_TOPICS;

  // 只在注册主题时调用, 所以已知主题上的 record() 是无锁的.
  // 单线程构建可以不设. lock 返回一个不透明状态, 由 unlock 还原.
  using LockFn = uint32_t (*)();
  using UnlockFn = void (*)(uint32_t);

  void set_lock(LockFn lock, UnlockFn unlock)
  {
    lock_ = lock;
    unlock_ = unlock;
  }

  PlotPublisher publish(const char * name, PlotUnit unit = PlotUnit::None)
  {
    PlotPublisher out;
    if (name == nullptr || name[0] == '\0') return out;
    const uint16_t hash = name_hash(name);
    PlotSlot * slot = lookup(name, hash);
    if (slot == nullptr) {
      const uint32_t state = lock();
      slot = lookup(name, hash);
      if (slot == nullptr) slot = insert(name, hash, unit);
      unlock(state);
    }
    out.slot_ = slot;
    return out;
  }

  // 写一个值, 名字第一次出现时注册主题. 任意任务可调: 冷路径串行化,
  // 热路径只是一次哈希比对加一次 float 存储.
  //
  // 单位只在注册那一次有效, 之后用不同单位调用会被忽略, 所以一个主题
  // 不会在运行中途换标签.
  void record(const char * name, float value, PlotUnit unit = PlotUnit::None)
  {
    if (name == nullptr || name[0] == '\0') return;
    const uint16_t hash = name_hash(name);
    PlotSlot * slot = lookup(name, hash);
    if (slot != nullptr) {
      slot->value = value;
      return;
    }
    const uint32_t state = lock();
    slot = lookup(name, hash);
    if (slot == nullptr) slot = insert(name, hash, unit);
    if (slot != nullptr) slot->value = value;
    unlock(state);
  }

  // 注册一个矩阵主题并返回句柄. 发布时拷贝格子, 所以调用方可以复用缓冲.
  // 行列都必须非 0, 且 rows*cols 不能超过 PLOT_MATRIX_MAX_CELLS.
  MatrixPublisher publish_matrix(
    const char * name, uint8_t rows, uint8_t cols, PlotUnit unit = PlotUnit::None)
  {
    MatrixPublisher out;
    if (name == nullptr || name[0] == '\0' || !matrix_dims_ok(rows, cols)) return out;
    const uint16_t hash = name_hash(name);
    PlotMatrixSlot * slot = lookup_matrix(name, hash);
    if (slot == nullptr) {
      const uint32_t state = lock();
      slot = lookup_matrix(name, hash);
      if (slot == nullptr) slot = insert_matrix(name, hash, rows, cols, unit);
      unlock(state);
    }
    out.slot_ = slot;
    return out;
  }

  // 第一次调用时注册, 之后拷格子. 已存在的主题保持原形状: 用不同尺寸调用会被
  // 忽略, 而不是写越界或者悄悄换一种解读.
  void record_matrix(
    const char * name, uint8_t rows, uint8_t cols, const float * values,
    PlotUnit unit = PlotUnit::None)
  {
    MatrixPublisher pub = publish_matrix(name, rows, cols, unit);
    if (pub.slot_ != nullptr && pub.rows() == rows && pub.cols() == cols) pub.set(values);
  }

  void ingest(const uint8_t * data, size_t n)
  {
    for (size_t i = 0; i < n; ++i) {
      if (acc_len_ < static_cast<int>(sizeof acc_)) acc_[acc_len_++] = data[i];
      pump();
    }
  }

  // 把待发的公告尽量塞进 dst. 不标记已发: 等字节真的出去了再由
  // commit_announce() 标记, 这样被截断的帧会重试而不是遗忘.
  size_t pop_announce(uint8_t * dst, size_t cap)
  {
    if (dst == nullptr) return 0;
    size_t total = 0;
    const int n = count_;
    for (int i = 0; i < n; ++i) {
      if (topics_[i].name[0] == '\0' || (topics_[i].flags & PLOT_SLOT_ANNOUNCED)) continue;
      const size_t name_n = std::strlen(topics_[i].name);
      const size_t need = announce_size(topics_[i]);
      if (total + need > cap || i > PLOT_MAX_ID) break;
      dst[total + 0] = 0xAA;
      dst[total + 1] = 0xBC;
      dst[total + 2] = static_cast<uint8_t>(i);
      std::memcpy(dst + total + 3, topics_[i].name, name_n + 1);
      dst[total + 3 + name_n + 1] = static_cast<uint8_t>(topics_[i].unit);
      total += need;
    }

    // 矩阵公告排在标量后面. 线上两套 id 空间是独立的, 所以槽位下标直接用.
    const int matrix_n = matrix_count_;
    for (int i = 0; i < matrix_n; ++i) {
      if (matrices_[i].name[0] == '\0' || (matrices_[i].flags & PLOT_SLOT_ANNOUNCED)) continue;
      const size_t need = matrix_announce_size(matrices_[i]);
      if (total + need > cap) break;
      dst[total + 0] = 0xAA;
      dst[total + 1] = 0xBE;
      dst[total + 2] = static_cast<uint8_t>(i);
      dst[total + 3] = matrices_[i].rows;
      dst[total + 4] = matrices_[i].cols;
      const size_t name_n = std::strlen(matrices_[i].name);
      std::memcpy(dst + total + 5, matrices_[i].name, name_n + 1);
      dst[total + 5 + name_n + 1] = static_cast<uint8_t>(matrices_[i].unit);
      total += need;
    }
    return total;
  }

  // 把上一次 pop_announce() 打包的 n 个字节标记为已发.
  // n 必须是那次写出的前缀长度.
  void commit_announce(size_t n)
  {
    size_t total = 0;
    const int count = count_;
    for (int i = 0; i < count && total < n; ++i) {
      if (topics_[i].name[0] == '\0' || (topics_[i].flags & PLOT_SLOT_ANNOUNCED)) continue;
      const size_t need = announce_size(topics_[i]);
      if (total + need > n || i > PLOT_MAX_ID) break;
      topics_[i].flags |= PLOT_SLOT_ANNOUNCED;
      total += need;
    }

    const int matrix_n = matrix_count_;
    for (int i = 0; i < matrix_n && total < n; ++i) {
      if (matrices_[i].name[0] == '\0' || (matrices_[i].flags & PLOT_SLOT_ANNOUNCED)) continue;
      const size_t need = matrix_announce_size(matrices_[i]);
      if (total + need > n) break;
      matrices_[i].flags |= PLOT_SLOT_ANNOUNCED;
      total += need;
    }
  }

  // 把已订阅的标量拼成一帧, 从上次的游标继续, 所以超过一帧的集合会分几次发
  // 而不是被丢掉.
  size_t pop_sample(uint8_t * dst, size_t cap)
  {
    if (dst == nullptr || cap < 9) return 0;
    const int n = count_;
    if (n == 0) return 0;

    size_t total = 3;
    int packed = 0;
    int i = sample_cursor_;
    if (i >= n) i = 0;
    for (int scanned = 0; scanned < n; ++scanned) {
      if (topics_[i].flags & PLOT_SLOT_SUBSCRIBED) {
        if (total + 5 + 1 > cap) break;
        dst[total++] = static_cast<uint8_t>(i);
        uint32_t bits = 0;
        std::memcpy(&bits, &topics_[i].value, sizeof bits);
        dst[total++] = static_cast<uint8_t>(bits);
        dst[total++] = static_cast<uint8_t>(bits >> 8);
        dst[total++] = static_cast<uint8_t>(bits >> 16);
        dst[total++] = static_cast<uint8_t>(bits >> 24);
        ++packed;
      }
      if (++i >= n) i = 0;
    }
    sample_cursor_ = i;
    if (packed == 0) return 0;
    dst[0] = 0xAA;
    dst[1] = 0xBB;
    dst[2] = static_cast<uint8_t>(packed);
    uint8_t sum = 0;
    for (size_t k = 0; k < total; ++k) sum ^= dst[k];
    dst[total++] = sum;
    return total;
  }

  // 把已订阅的矩阵拼成一帧 AA BF. 每个条目重复 rows/cols: 漏了公告的主机也能
  // 解析, 重新注册成新尺寸的主题也不会让解析器错位. 游标让装不下的集合分多帧发.
  size_t pop_matrix_sample(uint8_t * dst, size_t cap)
  {
    if (dst == nullptr || cap < 11) return 0;
    const int n = matrix_count_;
    if (n == 0) return 0;

    size_t total = 3;
    int packed = 0;
    int i = matrix_cursor_;
    if (i >= n) i = 0;
    for (int scanned = 0; scanned < n; ++scanned) {
      if (matrices_[i].flags & PLOT_SLOT_SUBSCRIBED) {
        const size_t cells = static_cast<size_t>(matrices_[i].rows) * matrices_[i].cols;
        if (total + 3 + cells * 4 + 1 > cap) break;
        dst[total++] = static_cast<uint8_t>(i);
        dst[total++] = matrices_[i].rows;
        dst[total++] = matrices_[i].cols;
        for (size_t c = 0; c < cells; ++c) {
          uint32_t bits = 0;
          std::memcpy(&bits, &matrices_[i].cells[c], sizeof bits);
          dst[total++] = static_cast<uint8_t>(bits);
          dst[total++] = static_cast<uint8_t>(bits >> 8);
          dst[total++] = static_cast<uint8_t>(bits >> 16);
          dst[total++] = static_cast<uint8_t>(bits >> 24);
        }
        ++packed;
      }
      if (++i >= n) i = 0;
    }
    matrix_cursor_ = i;
    if (packed == 0) return 0;
    dst[0] = 0xAA;
    dst[1] = 0xBF;
    dst[2] = static_cast<uint8_t>(packed);
    uint8_t sum = 0;
    for (size_t k = 0; k < total; ++k) sum ^= dst[k];
    dst[total++] = sum;
    return total;
  }

  int size() const { return count_; }
  int matrix_size() const { return matrix_count_; }
  int subscribed_count() const
  {
    int c = 0;
    const int n = count_;
    for (int i = 0; i < n; ++i) {
      if (topics_[i].flags & PLOT_SLOT_SUBSCRIBED) ++c;
    }
    return c;
  }

private:
  // topics_ 上的开放寻址索引: 存槽位下标 + 1, 0 表示空. 按名字查时走这里而不是
  // 扫遍所有槽位, 所以 record() 是 O(1) 而不是 O(注册数) —— 之前 1 kHz 任务
  // 写约 100 个主题时, 大部分时间都花在扫描上.
  //
  // 按槽位数的两倍开, 保证装载率不超过 1/2、探测序列一定撞到空槽.
  // 主题只追加不删除, 所以不需要墓碑标记.
  static constexpr int kIndexSize = plot_index_size(kMaxTopics);
  static_assert(kMaxTopics <= 255, "slot index + 1 must fit in the u8 index entry");

  PlotSlot topics_[kMaxTopics]{};
  uint8_t index_[kIndexSize]{};
  volatile int count_ = 0;
  int sample_cursor_ = 0;
  PlotMatrixSlot matrices_[PLOT_MAX_MATRICES]{};
  volatile int matrix_count_ = 0;
  int matrix_cursor_ = 0;
  uint8_t acc_[64]{};
  int acc_len_ = 0;
  LockFn lock_ = nullptr;
  UnlockFn unlock_ = nullptr;

  static uint16_t name_hash(const char * s)
  {
    uint32_t h = 2166136261u;
    while (*s != '\0') {
      h ^= static_cast<uint8_t>(*s++);
      h *= 16777619u;
    }
    return static_cast<uint16_t>(h ^ (h >> 16));
  }

  uint32_t lock() const { return lock_ == nullptr ? 0 : lock_(); }
  void unlock(uint32_t state) const
  {
    if (unlock_ != nullptr) unlock_(state);
  }

  // 从哈希位置沿探测序列找. 表最多半满, 所以一定停在空槽上.
  // 注册在锁里进行, 先写槽位再发布索引项, 所以读到非空索引项的线程看到的
  // 一定是名字已经写完整的槽位.
  PlotSlot * lookup(const char * name, uint16_t hash)
  {
    if (name == nullptr) return nullptr;
    int i = hash & (kIndexSize - 1);
    for (;;) {
      const uint8_t entry = index_[i];
      if (entry == 0) return nullptr;
      PlotSlot & slot = topics_[entry - 1];
      if (slot.hash == hash && std::strcmp(slot.name, name) == 0) return &slot;
      i = (i + 1) & (kIndexSize - 1);
    }
  }

  // 追加一个槽位并挂进索引. 名字写完才让 count_ 和索引可见, 所以读者要么没看到
  // 这个主题, 要么看到的是完整的. 放不下的名字直接拒收而不是截断 ——
  // 截断会把两个不同的路径并成同一个字段.
  PlotSlot * insert(const char * name, uint16_t hash, PlotUnit unit)
  {
    if (name == nullptr || name[0] == '\0' || count_ >= kMaxTopics) return nullptr;
    const size_t len = std::strlen(name);
    if (len >= static_cast<size_t>(PLOT_MAX_NAME)) return nullptr;

    int pos = hash & (kIndexSize - 1);
    while (index_[pos] != 0) pos = (pos + 1) & (kIndexSize - 1);

    const int id = count_;
    PlotSlot & slot = topics_[id];
    std::memset(&slot, 0, sizeof slot);
    std::memcpy(slot.name, name, len + 1);
    slot.hash = hash;
    slot.unit = unit;
    SP_PLOT_BARRIER();
    count_ = id + 1;
    SP_PLOT_BARRIER();
    // 最后发布: 这一步之后这个主题才能按名字查到.
    index_[pos] = static_cast<uint8_t>(id + 1);
    return &slot;
  }

  // 3 字节帧头 + 带 NUL 的名字 + 单位字节.
  static size_t announce_size(const PlotSlot & slot)
  {
    return 3 + std::strlen(slot.name) + 1 + 1;
  }

  static bool matrix_dims_ok(uint8_t rows, uint8_t cols)
  {
    return rows > 0 && cols > 0 &&
           static_cast<int>(rows) * static_cast<int>(cols) <= PLOT_MATRIX_MAX_CELLS;
  }

  PlotMatrixSlot * lookup_matrix(const char * name, uint16_t hash)
  {
    if (name == nullptr) return nullptr;
    const int n = matrix_count_;
    for (int i = 0; i < n; ++i) {
      if (matrices_[i].hash == hash && std::strcmp(matrices_[i].name, name) == 0) return &matrices_[i];
    }
    return nullptr;
  }

  PlotMatrixSlot * insert_matrix(
    const char * name, uint16_t hash, uint8_t rows, uint8_t cols, PlotUnit unit)
  {
    if (name == nullptr || name[0] == '\0' || matrix_count_ >= PLOT_MAX_MATRICES) return nullptr;
    if (!matrix_dims_ok(rows, cols)) return nullptr;
    const size_t len = std::strlen(name);
    if (len >= static_cast<size_t>(PLOT_MAX_NAME)) return nullptr;
    PlotMatrixSlot & slot = matrices_[matrix_count_];
    std::memset(&slot, 0, sizeof slot);
    std::memcpy(slot.name, name, len + 1);
    slot.hash = hash;
    slot.rows = rows;
    slot.cols = cols;
    slot.unit = unit;
    SP_PLOT_BARRIER();
    matrix_count_ = matrix_count_ + 1;
    return &slot;
  }

  // 3 字节帧头 + rows + cols + 带 NUL 的名字 + 单位字节.
  static size_t matrix_announce_size(const PlotMatrixSlot & slot)
  {
    return 7 + std::strlen(slot.name);
  }

  int find_matrix(const char * name) const
  {
    const uint16_t hash = name_hash(name);
    for (int i = 0; i < matrix_count_; ++i) {
      if (matrices_[i].hash == hash && std::strcmp(matrices_[i].name, name) == 0) return i;
    }
    return -1;
  }

  int find(const char * name)
  {
    const uint16_t hash = name_hash(name);
    const PlotSlot * slot = lookup(name, hash);
    // 槽位顺序就是线上 id, 所以下标就是相对基址的距离.
    return slot == nullptr ? -1 : static_cast<int>(slot - topics_);
  }

  void apply(uint8_t op, const char * name)
  {
    if (op == PLOT_OP_ALL || op == PLOT_OP_CLEAR) {
      const uint8_t on = op == PLOT_OP_ALL ? PLOT_SLOT_SUBSCRIBED : 0;
      for (int i = 0; i < count_; ++i) {
        // 强制重新公告, 好让刚连上的主机也拿到单位.
        topics_[i].flags = on;
      }
      for (int i = 0; i < matrix_count_; ++i) {
        matrices_[i].flags = on;
      }
      sample_cursor_ = 0;
      matrix_cursor_ = 0;
      return;
    }
    const int id = find(name);
    if (id >= 0) {
      if (op == PLOT_OP_SUB) topics_[id].flags |= PLOT_SLOT_SUBSCRIBED;
      else if (op == PLOT_OP_UNSUB) topics_[id].flags &= static_cast<uint8_t>(~PLOT_SLOT_SUBSCRIBED);
      return;
    }
    // 标量和矩阵共用订阅用的名字空间, 但不在同一张表里.
    const int matrix_id = find_matrix(name);
    if (matrix_id < 0) return;
    if (op == PLOT_OP_SUB) matrices_[matrix_id].flags |= PLOT_SLOT_SUBSCRIBED;
    else if (op == PLOT_OP_UNSUB) matrices_[matrix_id].flags &= static_cast<uint8_t>(~PLOT_SLOT_SUBSCRIBED);
  }

  void pump()
  {
    while (acc_len_ >= 3) {
      int start = -1;
      for (int i = 0; i + 1 < acc_len_; ++i) {
        if (acc_[i] == 0xAA && acc_[i + 1] == 0xBD) {
          start = i;
          break;
        }
      }
      if (start < 0) {
        acc_len_ = acc_[acc_len_ - 1] == 0xAA ? 1 : 0;
        if (acc_len_ == 1) acc_[0] = 0xAA;
        return;
      }
      if (start > 0) {
        std::memmove(acc_, acc_ + start, static_cast<size_t>(acc_len_ - start));
        acc_len_ -= start;
      }
      const uint8_t op = acc_[2];
      int end = 3;
      if (op == PLOT_OP_SUB || op == PLOT_OP_UNSUB) {
        while (end < acc_len_ && acc_[end] != 0) ++end;
        if (end >= acc_len_) return;
      }
      char name[PLOT_MAX_NAME];
      name[0] = '\0';
      if (op == PLOT_OP_SUB || op == PLOT_OP_UNSUB) {
        const int c = end - 3;
        // 放不下的名字不可能匹配任何已注册主题 (它们都更短), 所以直接丢掉,
        // 而不是截断成一个错误的名字.
        if (c <= 0 || c >= PLOT_MAX_NAME) {
          ++end;
          std::memmove(acc_, acc_ + end, static_cast<size_t>(acc_len_ - end));
          acc_len_ -= end;
          continue;
        }
        std::memcpy(name, acc_ + 3, static_cast<size_t>(c));
        name[c] = '\0';
        ++end;
      }
      apply(op, name);
      std::memmove(acc_, acc_ + end, static_cast<size_t>(acc_len_ - end));
      acc_len_ -= end;
    }
  }
};

}  // namespace sp

#endif  // SP__PLOT_LINK_HPP
