# Generated Files

更新时间：2026-06-30

本目录集中存放重构过程中生成的资料文件，避免工程根目录和固件源码目录继续散落文档、测试脚本和文本日志。

## 目录

| 目录 | 内容 |
|---|---|
| `Docs` | 架构说明、编码规范落地说明、模块拆分设计、变更记录和测试报告 |
| `Docs/Archive` | 早期联调、修复和参数整理记录 |
| `Tests` | 主机侧工程结构检查和可选 C 单元测试 |
| `BuildLogs` | 已生成的文本构建日志 |

## 当前验证

- Keil GUI Rebuild 已通过：`0 Error(s), 0 Warning(s)`。
- 主机侧检查入口：`python Generated\Tests\run_tests.py`。
- 固件源码、Keil 工程文件和编译输出仍保留在 `GD32G553` 下。
