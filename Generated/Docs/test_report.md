# 测试报告

更新时间：2026-06-30

## 范围

本报告覆盖按 `code.md` 做的分层重构和 `charge_manager` 内部拆分。主要风险是文件移动后 Keil 构建引用失效，以及内部 `.inc` 分片包含顺序错误；硬件行为预期保持不变。

## 检查项

- `python Generated\Tests\run_tests.py`
- 测试脚本内置 Keil 工程引用检查
- 测试脚本检查 `charge_manager.c` 内部分片存在且按固定顺序包含
- 测试脚本检查生成资料已集中到 `Generated` 目录
- 当 `gcc`、`clang` 或 `cl` 可用时，运行可选主机侧 C 单元测试
- 如果本机有 Keil 命令行工具，则运行固件构建

## 结果

`python Generated\Tests\run_tests.py` passed on 2026-06-30.

```text
PASS layer files exist
PASS old moved paths removed
PASS Keil FilePath entries resolve
PASS Keil include paths use layers
PASS charge manager fragments included
PASS generated files grouped
SKIP optional C unit tests: no gcc, clang, or cl found on PATH
```

Keil GUI Rebuild 已通过 on 2026-06-30:

```text
*** Using Compiler 'V6.19', folder: 'D:\Keil_v5\ARM\ARMCLANG\Bin'
Rebuild target 'Target 1'
Program Size: Code=76832 RO-data=1896 RW-data=8 ZI-data=24600
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:06
```

Keil 命令行构建未运行，因为本机 `PATH` 中没有 `UV4`/`armclang`；本次采用用户在 Keil GUI 中完成的 Rebuild 结果作为固件编译验证记录。
