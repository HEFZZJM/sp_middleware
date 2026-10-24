# SuperPlotter

一条只发"被订阅的变量"的单向链路，配套上位机是 SuperScope（AdvantageScope 的改造版）。
和 `io/plotter` 里原来的 `Plotter` 互不影响，两者可以同时使用。

- **命名主题**：路径是一个字符串，不是数组下标，上位机按名字勾选。
- **订阅制**：板上记录所有路径，但**只发送上位机勾选的那些**，串口带宽只取决于勾了几条。
- **中断发送**：`flush()` 把一帧交给 UART 就返回，耗时几微秒。

## 用法

```cpp
#include "io/superplotter/super_plotter.hpp"

sp::SuperPlotter::record("arm/j0/pos", arm_j0.pos);              // 自动注册路径
sp::SuperPlotter::record("arm/j0/vel", arm_j0.vel, sp::PlotUnit::RadianPerSecond);
```

任意任务、任意位置都能调用：第一次调用时注册路径，之后每次只写一个 float，
按 16 位哈希定位槽位，无锁、无内存分配。

同一个量在热点循环里高频写时，用句柄形式省掉查表：

```cpp
sp::PlotPublisher j0 = plotter.publish("arm/j0/pos");   // 注册一次，返回句柄
j0.set(arm_j0.pos);                                     // 之后是一次 float 写入
```

> `record()` 是静态函数，任意任务可调；`publish()` 是成员函数，需要实例。

矩阵（3D 向量、位姿）用另一套帧，单位是"行×列个 float"：

```cpp
float xyz[3] = {x, y, z};
sp::SuperPlotter::record_matrix("chassis/pos", 1, 3, xyz, sp::PlotUnit::Meter);
```

## 约束

| 项目 | 值 | 说明 |
| --- | --- | --- |
| 路径长度 | ≤ 15 字符 | 超长**编译报错**，不会截断（截断会把两个路径并成一个字段） |
| 路径形式 | 字面量 | 运行期 `const char *` 不接受，长度必须编译期已知 |
| 注册上限 | 128 个 | 每槽 24 字节，共约 3 KB；可用 `-DSP_PLOT_MAX_TOPICS=<n>` 改 |
| 矩阵上限 | 8 个 × 32 格 | 可用 `-DSP_PLOT_MAX_MATRICES` / `-DSP_PLOT_MATRIX_CELLS` 改 |
| 单帧标量 | 8 个值（44 字节） | 订阅数更多时分多帧轮流发，不丢数据，只是每路刷新变慢 |
| 单位 | `sp::PlotUnit` 枚举 | 1 字节，只作标签，不做换算 |

注册和发送是两个独立的量：注册只占 RAM，**只有上位机勾选的路径才会被发送**。
所以路径可以放心留在代码里。`record()` 超过注册上限会**静默丢弃**——名字照样编译过，
只是永远不出现在上位机列表里，用 `pl/topics` 判断是否卡到上限了。

每路的实际刷新率取决于订阅数和采样周期：

```
每路更新率 = 1000 / period_ms * 8 / N        (N = 勾选的通道数)
```

勾 8 路在 1 ms 周期下就是 1 kHz，勾 40 路约 200 Hz。链路本身 921600 baud 约 92 KB/s，
一帧 44 字节在 1 ms 节拍下只占 48%，所以加变量不会丢数据，只是被拉稀。

## 协议

```
板 -> 上位机  AA BC | id u8 | name, NUL | unit u8        首次 flush 时为每个标量主题发一次
板 -> 上位机  AA BE | id u8 | rows u8 | cols u8 | name, NUL | unit u8    矩阵主题公告
上位机 -> 板  AA BD | op u8 | name, NUL                  1 订阅 2 取消 3 全订阅 4 清空
板 -> 上位机  AA BB | count u8 | (id u8, f32le)* | xor u8                标量样本
板 -> 上位机  AA BF | count u8 | (id u8, rows, cols, f32le[r*c])* | xor u8   矩阵样本
```

`xor` 覆盖前面所有字节，校验失败的帧整帧丢弃而不是画出一个毛刺。

`unit` 是 `sp::PlotUnit` 的 id（0 = 无单位），一个字节，随名字一起只发一次。
它只是给上位机坐标轴用的标签，**数值始终按记录时的单位原样发送**，不做换算。
枚举放在 `plot_link.hpp`，id 是协议的一部分：新增只能往后加，不能改号。

只列了 SuperScope 能真正换算的单位。力矩（N·m）、质量（kg）、功率（W）这类
它没有对应单位组的量，请用 `PlotUnit::None`——挂一个上位机解析不了的标签，
比没有标签更容易让人误以为轴已经处理好了。

## 接入

在某个任务里创建实例并开跑。构造函数四个参数都有默认值，通常只需要前两个：

```cpp
// 串口, 是否用 DMA, 标量采样周期 ms, 矩阵发送周期 ms
sp::SuperPlotter plotter(&huart7, false, 1, 10);

extern "C" void plot_task()
{
  plotter.initialize();     // 启动接收, 这样上位机才能发 AA BD 订阅帧
  plotter.loop();           // 不返回: 每 period_ms 采样一次并发送
}
```

`initialize()` 只做初始化，`loop()` 不会返回。想自己控制节奏就手写循环调 `flush()`：
`record()` 是静态的，在哪个任务里调都可以。

UART 中断里把收到的字节交给 `on_rx()` 即可：

```cpp
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  if (huart == plotter.huart()) {
    plotter.on_rx(size);
    plotter.request();      // 重新挂接收, 否则只收到第一批
  }
}
```

用中断发送时还需要把完成回调接上，否则下一帧永远发不出去：

```cpp
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart == plotter.huart()) plotter.on_tx_complete();
}
```

`initialize()` 还有一种接受轮询表的写法（`PolledRow` 数组），表里一行一个路径，
由 `loop()` 按固定节拍采样。多数情况直接用 `record()` 更省事，不需要这张表。

## 自诊断

绘图器自己会记录几个量，曲线不对时先看它们：

| 路径 | 含义 |
| --- | --- |
| `pl/topics` | 已注册的槽数，卡在 128 = 槽用完了，后面的 `record()` 被丢弃 |
| `pl/subs` | 上位机勾了几路，乘以上面的公式就是每路更新率 |
| `pl/tx_fail` | 非 0 且持续涨 = 链路/UART 在丢帧，不是上位机解析慢 |
| `pl/tx_us` | 最近一帧的线上时间，接近 `period_ms * 1000` = 线速顶满 |
| `pl/tx_max` | 上电以来最长的一帧 |
| `pl/tx_age` | 距上一帧发完过了多少 ms，尖峰 = 打印线程被停了这么久 |
| `pl/tx_age_max` | 上电以来最坏的那次，已经过去的卡顿事后也看得到 |
| `pl/loop` | `loop()` 的迭代计数，不涨 = 任务根本没在跑 |

`pl/tx_age` 出现尖峰时，`pl/tx_fail` 平 = 任务被抢占（CPU 问题）；
`pl/tx_fail` 同时在涨 = 那一帧没发完被丢了（链路问题）。

## 上位机

配套的 SuperScope 在 `sp_plotter` 仓库，是 AdvantageScope 的改造版。
连上后它先收公告建字段列表，再按界面上勾选的曲线发订阅帧，取消勾选就发退订帧。
