## 简介
同济大学SuperPower战队25赛季电控中间件

命名空间`sp::`

文件结构:
- io: 常用硬件(led, 蜂鸣器, 遥控器, 上位机曲线...)
- motor: 电机相关
- referee: 裁判系统相关
- tools: 常用工具和算法

## 安装
```bash
git submodule add https://github.com/TongjiSuperPower/sp_middleware.git
```

## 上位机曲线 SuperPlotter

`io/superplotter` 提供一条只发"被订阅的变量"的单向链路，配套上位机是 SuperScope
（AdvantageScope 的改造版）。和 `io/plotter` 里原来的 `Plotter` 互不影响，两者可以同时使用。

```cpp
#include "io/superplotter/super_plotter.hpp"

sp::SuperPlotter::record("arm/j0/pos", arm_j0.pos, sp::PlotUnit::Radian);
```

用法、约束、线上协议和接入代码见 [`io/superplotter/readme.md`](io/superplotter/readme.md)。

## 参考
TODO
