# 测试报告

更新时间：2026-06-30

## 范围

本报告覆盖按 `code.md` 做的分层重构和 `charge_manager` 内部拆分。主要风险是文件移动后 Keil 构建引用失效，以及内部 `.inc` 分片包含顺序错误；硬件行为预期保持不变。

## 检查项

- `python Tests/run_tests.py`
- 测试脚本内置 Keil 工程引用检查
- 测试脚本检查 `charge_manager.c` 内部分片存在且按固定顺序包含
- 当 `gcc`、`clang` 或 `cl` 可用时，运行可选主机侧 C 单元测试
- 如果本机有 Keil 命令行工具，则运行固件构建

## 结果

`python Tests/run_tests.py` passed on 2026-06-30.

```text
PASS layer files exist
PASS old moved paths removed
PASS Keil FilePath entries resolve
PASS Keil include paths use layers
PASS charge manager fragments included
SKIP optional C unit tests: no gcc, clang, or cl found on PATH
```

Keil 命令行构建未运行，因为本机 `PATH` 中没有 `UV4`/`armclang`。
