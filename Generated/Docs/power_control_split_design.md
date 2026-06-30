# Power Control 拆分设计

更新时间：2026-06-30

## 为什么需要拆分

`GD32G553/Driver/power_control.c` 同时包含 HRTIMER/PWM 初始化、占空比映射、软启动、预连接、AFE handover、输入保护、OVP/OCP、PI 快环和对外控制 API，单文件已经超过规范建议的 600 行拆分触发线。继续在一个文件里堆控制细节，会让保护逻辑和模式映射互相缠在一起，后续排查硬件问题时不容易确认改动边界。

## 为什么这样设计

本轮沿用 `charge_manager` 的内部 `.inc` 分片方式：`power_control.c` 仍是唯一编译单元，静态状态和 `static` 函数可见性保持不变，外部头文件 `power_control.h` 不新增接口。这样可以先把文件按职责拆开，又不引入新的链接单元、头文件依赖或运行时行为变化。

## 分片边界

- `power_control_pwm.inc`：HRTIMER/PWM 计数、死区、互补输出、故障输入、采样点等待和输出开关。
- `power_control_safety.inc`：控制环复位、停滞恢复、输出 OVP/OCP、软启动电流和采样电流选择。
- `power_control_limits.inc`：轻载、预连接、AFE handover、电池升压占空比限制和输入保护降占空比。
- `power_control_mode.inc`：电压环选择、异步 Boost 整流判断、故障快照和 Buck/Boost/Buck-Boost 映射。
- `power_control_api.inc`：对外 API、故障闭锁入口、目标设置、反馈设置、快环主流程和状态读取。

## 能做到什么

- 降低 `power_control.c` 主文件长度，让入口状态、宏和分片顺序更清晰。
- 保持 PWM 初始化顺序、PI 参数、保护阈值、占空比限制和外部 API 不变。
- 让后续硬件问题能按 PWM、保护、限制、模式映射、API/快环分区定位。

## 做不到什么

- 本轮不改变控制算法、保护阈值、PWM 占空比公式或 AFE handover 行为。
- 本轮不把静态函数提升为独立 `.c/.h` 模块。
- 本轮不新增硬件抽象层，也不重命名外部接口。

## 如何测试

运行：

```powershell
python Generated\Tests\run_tests.py
```

测试会检查 power control 内部分片存在、`power_control.c` 按固定顺序包含这些分片、Keil 工程路径仍然有效。最终仍需要在 Keil GUI 中执行 Rebuild，确认固件编译通过。
