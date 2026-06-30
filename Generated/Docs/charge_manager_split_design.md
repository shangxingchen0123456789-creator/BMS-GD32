# Charge Manager 拆分设计

更新时间：2026-06-30

## 为什么需要拆分

`GD32G553/Module/charge_manager.c` 集中了参数校验、命令处理、故障收集、预连接路径、状态机更新和状态上报，单文件超过 1600 行。它虽然能工作，但后续继续改充电策略时，容易把无关逻辑卷在一起，审查和回退成本都很高。

## 为什么这样设计

本轮采用内部 `.inc` 分片方式，把静态函数按职责拆到多个文件中，再由 `charge_manager.c` 按固定顺序包含。这样仍然保持一个 C 编译单元，原有 `static` 变量、临界区和函数可见性不变，避免为了拆文件而把大量内部状态暴露到头文件。

## 优点

- 外部 API 不变，`charge_manager.h` 不新增内部接口。
- Keil 工程只需要继续编译 `charge_manager.c`，不会增加链接单元。
- 适合硬件控制代码的第一步拆分，风险低、容易回退。
- 后续如果某个分片边界稳定，再升级为独立 `.c/.h` 模块。

## 模块能做到什么

- `charge_manager_params.inc`：命令回复、默认参数、参数范围校验。
- `charge_manager_faults.inc`：故障采集、OCP/OVP 确认、数字电源故障过滤。
- `charge_manager_path.inc`：预连接目标、路径 settle、手动 FET 状态辅助函数。
- `charge_manager_commands.inc`：普通命令、数字电源命令和手动 FET 命令处理。
- `charge_manager_control.inc`：状态对应目标电流、控制模式和 CV 完成条件。

## 模块做不到什么

- 本轮不改变充电状态机行为。
- 本轮不拆 `Charge_Manager_Update()` 主状态机。
- 本轮不重命名外部函数、不改通信协议、不改充电参数上下限。

## 与其他模块如何交互

`charge_manager.c` 仍然是唯一外观入口，继续调用 `Power_Control`、`Power_Path_Manager`、`Safety_Manager`、`Afe_Gd30bm2016` 和 `Param_Storage`。新增 `.inc` 文件只作为 `charge_manager.c` 的内部实现片段，不允许被其他模块直接包含。

## 如何测试

运行：

```powershell
python Generated\Tests\run_tests.py
```

测试会检查内部分片文件存在、`charge_manager.c` 包含这些分片、Keil 工程路径仍然有效。若本机有 C 编译器，还会运行已有 Common/Module 主机侧单元测试。
