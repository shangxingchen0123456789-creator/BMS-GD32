# 测试报告

更新时间：2026-06-30

## 范围

本报告覆盖按 `code.md` 做的分层重构、`charge_manager` 内部拆分和 `power_control` 内部拆分。主要风险是文件移动后 Keil 构建引用失效，以及内部 `.inc` 分片包含顺序错误；硬件行为预期保持不变。

## 检查项

- `python Generated\Tests\run_tests.py`
- 测试脚本内置 Keil 工程引用检查
- 测试脚本检查 `charge_manager.c` 内部分片存在且按固定顺序包含
- 测试脚本检查 `power_control.c` 内部分片存在且按固定顺序包含
- 测试脚本检查生成资料已集中到 `Generated` 目录
- 展开 `power_control.c` 的内部 include 后，与拆分前 `HEAD` 内容逐字节一致
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
PASS power control fragments included
PASS generated files grouped
SKIP optional C unit tests: no gcc, clang, or cl found on PATH
```

`power_control.c` 展开等价性检查 passed on 2026-06-30:

```text
PASS expanded power_control matches HEAD content
```

Keil GUI Rebuild 已通过 on 2026-06-30:

```text
*** Using Compiler 'V6.19', folder: 'D:\Keil_v5\ARM\ARMCLANG\Bin'
Rebuild target 'Target 1'
Program Size: Code=76832 RO-data=1896 RW-data=8 ZI-data=24600
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:06
```

Keil UV4 command-line build passed on 2026-06-30:

```text
*** Using Compiler 'V6.19', folder: 'D:\Keil_v5\ARM\ARMCLANG\Bin'
Build target 'Target 1'
compiling power_control.c...
linking...
Program Size: Code=76832 RO-data=1896 RW-data=8 ZI-data=24600
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:02
```

## Power PWM Module Follow-up

This round promotes PWM/HRTIMER output code from `power_control_pwm.inc` into
`power_pwm.c/.h`. The validation checks that the new module exists, is
referenced by Keil, is no longer included as a `.inc` fragment by
`power_control.c`, and does not depend on `power_control.h`.

`python Generated\Tests\run_tests.py` passed for this follow-up on 2026-06-30:

```text
PASS layer files exist
PASS old moved paths removed
PASS Keil FilePath entries resolve
PASS Keil include paths use layers
PASS charge manager fragments included
PASS power control fragments included
PASS power pwm module extracted
PASS generated files grouped
SKIP optional C unit tests: no gcc, clang, or cl found on PATH
```

Keil UV4 command-line rebuild passed for this follow-up on 2026-06-30:

```text
compiling power_pwm.c...
compiling power_control.c...
Program Size: Code=76980 RO-data=1896 RW-data=8 ZI-data=24600
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:02
```

## Remove INC Modules Follow-up

This round converts every remaining project-owned `.inc` file into a real
`.c` module and adds internal context headers for `power_control` and
`charge_manager`.

`python Generated\Tests\run_tests.py` passed for this follow-up on 2026-06-30:

```text
PASS layer files exist
PASS old moved paths removed
PASS Keil FilePath entries resolve
PASS Keil include paths use layers
PASS no inc files remain
PASS real internal modules referenced
PASS power pwm module extracted
PASS generated files grouped
SKIP optional C unit tests: no gcc, clang, or cl found on PATH
```

Keil UV4 command-line build passed for this follow-up on 2026-06-30:

```text
compiling power_control_mode.c...
compiling power_control_limits.c...
compiling power_control_api.c...
compiling power_control_safety.c...
compiling charge_manager_params.c...
compiling charge_manager_control.c...
compiling charge_manager_faults.c...
compiling charge_manager_path.c...
compiling charge_manager_commands.c...
Program Size: Code=76340 RO-data=1896 RW-data=8 ZI-data=24592
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:06
```

## Internal Context Access Follow-up

This round removes the transitional `s_xxx` macro aliases from the internal
headers and updates the converted modules to access `g_power_control` and
`g_charge_manager` fields directly.

`python Generated\Tests\run_tests.py` passed for this follow-up on 2026-06-30:

```text
PASS layer files exist
PASS old moved paths removed
PASS Keil FilePath entries resolve
PASS Keil include paths use layers
PASS no inc files remain
PASS real internal modules referenced
PASS internal context access is explicit
PASS power pwm module extracted
PASS generated files grouped
SKIP optional C unit tests: no gcc, clang, or cl found on PATH
```

Keil UV4 command-line rebuild passed for this follow-up on 2026-06-30:

```text
compiling power_control_safety.c...
compiling power_control_api.c...
compiling power_control_mode.c...
compiling power_control_limits.c...
compiling charge_manager.c...
compiling charge_manager_params.c...
compiling charge_manager_path.c...
compiling charge_manager_faults.c...
compiling charge_manager_commands.c...
compiling charge_manager_control.c...
Program Size: Code=76340 RO-data=1896 RW-data=8 ZI-data=24592
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:06
```

## Internal Context Split Follow-up

This round groups the private `power_control` and `charge_manager` contexts by
responsibility. The public APIs and Keil compile-unit layout remain unchanged.

`python Generated\Tests\run_tests.py` passed for this follow-up on 2026-06-30:

```text
PASS layer files exist
PASS old moved paths removed
PASS Keil FilePath entries resolve
PASS Keil include paths use layers
PASS no inc files remain
PASS real internal modules referenced
PASS internal context access is explicit
PASS internal contexts are grouped
PASS power pwm module extracted
PASS generated files grouped
SKIP optional C unit tests: no gcc, clang, or cl found on PATH
```

Keil UV4 command-line rebuild passed for this follow-up on 2026-06-30:

```text
compiling power_control_api.c...
compiling power_control_safety.c...
compiling charge_manager.c...
compiling power_control_mode.c...
compiling charge_manager_params.c...
compiling power_control_limits.c...
compiling charge_manager_path.c...
compiling charge_manager_control.c...
compiling charge_manager_commands.c...
compiling charge_manager_faults.c...
Program Size: Code=76340 RO-data=1896 RW-data=8 ZI-data=24592
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:07
```

## Charge Manager Update Helpers Follow-up

This round keeps `Charge_Manager_Update` as the public update entry point but
splits its internal work into private helpers for snapshotting, dispatch,
normal charging, path ownership, status output, and context write-back.

`python Generated\Tests\run_tests.py` passed for this follow-up on 2026-06-30:

```text
PASS layer files exist
PASS old moved paths removed
PASS Keil FilePath entries resolve
PASS Keil include paths use layers
PASS no inc files remain
PASS real internal modules referenced
PASS internal context access is explicit
PASS internal contexts are grouped
PASS power pwm module extracted
PASS charge update uses helpers
PASS generated files grouped
SKIP optional C unit tests: no gcc, clang, or cl found on PATH
```

Keil UV4 command-line rebuild passed for this follow-up on 2026-06-30:

```text
compiling charge_manager_path.c...
compiling charge_manager.c...
compiling power_manager.c...
compiling charge_manager_faults.c...
compiling charge_manager_control.c...
compiling charge_manager_commands.c...
Program Size: Code=76724 RO-data=1896 RW-data=8 ZI-data=24592
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:06
```
