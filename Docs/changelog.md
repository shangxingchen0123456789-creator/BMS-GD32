# 变更记录

## 2026-06-30

- 拆分 `Module/charge_manager.c` 内部实现，新增 `charge_manager_params.inc`、`charge_manager_path.inc`、`charge_manager_faults.inc`、`charge_manager_control.inc`、`charge_manager_commands.inc`。
- 保留 `charge_manager.c` 作为外观入口和主状态机文件，外部 API 不变。
- 新增 `Docs/charge_manager_split_design.md`，记录拆分原因、边界和测试方式。
- 更新 Keil 工程 Module 分组，使内部实现片段可见但不作为独立编译单元。
- 更新 `Tests/run_tests.py`，检查 charge manager 内部分片存在且包含顺序正确。
- 新增 `Docs`、`Tests`、`App`、`BSP`、`Common`、`Config`、`Module`、`Service` 目录。
- 将任务编排从 `Driver` 移到 `App`。
- 将板级支持和 UART BSP 代码从 `Driver`/`Comm` 移到 `BSP`。
- 将共享类型、PI 控制器和电池模型移到 `Common`。
- 将板级映射和标定常量移到 `Config`。
- 将充电、电源路径、功率限制、SOC 和均衡逻辑移到 `Module`。
- 将协议、通信服务、状态快照、参数存储、故障日志和安全服务移到 `Service`。
- 更新 Keil 工程源码路径和 include path，使其匹配新目录。
- 新增主机侧工程结构检查和可选 C 单元测试。
