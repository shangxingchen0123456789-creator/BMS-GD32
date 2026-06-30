# GD32G553 固件架构

更新时间：2026-06-30

## 为什么调整

之前工程把大多数 App、Service、Module、BSP 和 Driver 代码都放在 `GD32G553/Driver`，另有一个 `Comm` 目录。这样可以编译，但职责边界不清：任务编排、协议处理、状态机、硬件驱动和通用工具看起来都像驱动。新目录遵循 `code.md`，让后续功能可以落在正确层级。

## 设计选择

本次优先做目录和构建归属调整。公共头文件名和 C 函数名保持不变，避免改变固件行为和协议接口。Keil include path 已加入各层目录，因此现有 `#include "module_name.h"` 风格仍可工作。

## 分层映射

| 目录 | 模块 |
|---|---|
| `GD32G553/App` | `app_tasks.*` |
| `GD32G553/BSP` | `board_support.*`, `bms_uart.*` |
| `GD32G553/Common` | `bms_types.h`, `pi_controller.*`, `battery_model.*` |
| `GD32G553/Config` | `bms_board_config.h` |
| `GD32G553/Driver` | `adc_manager.*`, `afe_gd30bm2016.*`, `flash_storage.*`, `power_control.*` |
| `GD32G553/Module` | `charge_manager.*`, `power_manager.*`, `power_path_manager.*`, `balance_manager.*`, `soc_estimator.*` |
| `GD32G553/Service` | `bms_comm_service.*`, `bms_protocol.*`, `bms_state.*`, `fault_log.*`, `param_storage.*`, `safety_manager.*` |

## 能做到什么

- 新功能更容易按职责放置，不再继续膨胀 `Driver`。
- 硬件访问与任务调度、协议逻辑分开。
- 主机侧测试可以优先覆盖不依赖 MCU 寄存器的 Common 和 Module 代码。
- 当前已验证的充电行为保持不变。

## 做不到什么

- 暂不拆分 `power_control.c`、`charge_manager.c`、`afe_gd30bm2016.c` 等长文件内部逻辑。
- 不引入新的 RTOS 抽象、日志框架或测试框架。
- 不改变 UART 协议、充电状态机行为、PWM 映射、ADC 映射或 AFE 时序。

## 模块交互

`User/main.c` 启动板级支持和 `App_Tasks_Start()`。`App` 负责控制、快环、安全、通信和心跳任务调度。`Module` 负责充电策略和状态机。`Service` 负责通信、快照、持久化、故障日志和安全锁存。`Driver` 是直接访问 GD32 外设 API 或外部芯片事务的层。

## 测试

运行：

```powershell
python Tests/run_tests.py
```

测试入口会检查：

- 必要分层目录和移动后的文件是否存在；
- 旧源码路径是否已经从 Keil 工程中移除；
- Keil 工程每个 `FilePath` 是否能解析到真实文件；
- include path 是否包含新的核心层级；
- 如果主机有 C 编译器，则编译并运行 PI、电池模型、均衡逻辑单元测试。
