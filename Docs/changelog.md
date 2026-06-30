# 变更记录

## 2026-06-30

- 新增 `Docs`、`Tests`、`App`、`BSP`、`Common`、`Config`、`Module`、`Service` 目录。
- 将任务编排从 `Driver` 移到 `App`。
- 将板级支持和 UART BSP 代码从 `Driver`/`Comm` 移到 `BSP`。
- 将共享类型、PI 控制器和电池模型移到 `Common`。
- 将板级映射和标定常量移到 `Config`。
- 将充电、电源路径、功率限制、SOC 和均衡逻辑移到 `Module`。
- 将协议、通信服务、状态快照、参数存储、故障日志和安全服务移到 `Service`。
- 更新 Keil 工程源码路径和 include path，使其匹配新目录。
- 新增主机侧工程结构检查和可选 C 单元测试。
