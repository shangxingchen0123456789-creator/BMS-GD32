# Power PWM 模块化设计

更新时间：2026-06-30

## 为什么需要这一步

上一轮把 `power_control.c` 拆成内部 `.inc` 片段，已经降低了长文件维护成本。但 `.inc` 仍然依赖 `power_control.c` 的静态变量和 include 顺序，不是真正独立模块。PWM/HRTIMER 输出层边界最清楚，适合作为第一块升级为 `.c/.h` 的内部驱动模块。

## 设计选择

新增 `power_pwm.c/.h`，只封装 HRTIMER 初始化、占空比写入、输出开关、FAULT 恢复、ADC 同步采样点等待和硬件状态读取。`power_control` 继续负责控制策略、保护判断和 Buck/Boost 模式决策，并通过显式参数把当前输出上下文传给 PWM 模块。

## 接口边界

- `Power_Pwm_Init()`：初始化 GPIO、HRTIMER、FAULT 和互补输出定时器。
- `Power_Pwm_Apply()`：写入 Buck 半桥和 Boost 半桥占空比。
- `Power_Pwm_Outputs_Enable()`：根据上下文打开或关闭实际输出通道。
- `Power_Pwm_Outputs_Disable()`：关闭 PWM 输出通道。
- `Power_Pwm_Wait_Adc_Sample_Point()`：等待同步采样比较点。
- `Power_Pwm_Get_State()`：返回 ready、outputsOn 和 periodTicks。

## 能做到什么

- 让 PWM/HRTIMER 细节从 `power_control` 控制策略中独立出来。
- 不改变 PWM 频率、死区、比较值计算、输出通道选择和 FAULT 行为。
- 为后续把 ADC 同步采样、硬件 FAULT 处理继续下沉到 BSP/Driver 层留出接口。

## 做不到什么

- 本轮不修改 PI 控制、软启动、OVP/OCP、预连接、AFE handover 或模式映射。
- 本轮不抽象所有硬件外设，也不引入新的回调或 RTOS 通信机制。
- 本轮不改变 `power_control.h` 对外 API。

## 如何测试

运行：

```powershell
python Generated\Tests\run_tests.py
```

## Boundary Update

`power_pwm.c` deliberately does not include `power_control.h`. The control
layer converts its internal stage enum into `boostStageActive` before calling
`Power_Pwm_Outputs_Enable()`, so the hardware output module only sees the
small context needed to select HRTIMER channels.

The structure test rejects reintroducing `power_control_pwm.inc` and checks
that `power_pwm.c` is compiled by the Keil project.

并使用 Keil UV4 命令行 Rebuild，确认新增 `power_pwm.c` 作为独立编译单元后链接通过。
