#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GENERATED = ROOT / "Generated"
TEST_ROOT = GENERATED / "Tests"
FW = ROOT / "GD32G553"
PROJECT_FILE = FW / "Project" / "GD32G553_CHG.uvprojx"

EXPECTED_FILES = [
    "App/app_tasks.c",
    "App/app_tasks.h",
    "BSP/board_support.c",
    "BSP/board_support.h",
    "BSP/bms_uart.c",
    "BSP/bms_uart.h",
    "Common/bms_types.h",
    "Common/pi_controller.c",
    "Common/pi_controller.h",
    "Common/battery_model.c",
    "Common/battery_model.h",
    "Config/bms_board_config.h",
    "Driver/adc_manager.c",
    "Driver/afe_gd30bm2016.c",
    "Driver/flash_storage.c",
    "Driver/power_control.c",
    "Driver/power_control_api.inc",
    "Driver/power_control_limits.inc",
    "Driver/power_control_mode.inc",
    "Driver/power_control_pwm.inc",
    "Driver/power_control_safety.inc",
    "Module/balance_manager.c",
    "Module/charge_manager.c",
    "Module/charge_manager_commands.inc",
    "Module/charge_manager_control.inc",
    "Module/charge_manager_faults.inc",
    "Module/charge_manager_params.inc",
    "Module/charge_manager_path.inc",
    "Module/power_manager.c",
    "Module/power_path_manager.c",
    "Module/soc_estimator.c",
    "Service/bms_comm_service.c",
    "Service/bms_protocol.c",
    "Service/bms_state.c",
    "Service/fault_log.c",
    "Service/param_storage.c",
    "Service/safety_manager.c",
]

CHARGE_MANAGER_FRAGMENTS = [
    'charge_manager_params.inc',
    'charge_manager_path.inc',
    'charge_manager_faults.inc',
    'charge_manager_control.inc',
    'charge_manager_commands.inc',
]

POWER_CONTROL_FRAGMENTS = [
    'power_control_pwm.inc',
    'power_control_safety.inc',
    'power_control_limits.inc',
    'power_control_mode.inc',
    'power_control_api.inc',
]

GENERATED_EXPECTED_FILES = [
    "README.md",
    "Docs/architecture.md",
    "Docs/changelog.md",
    "Docs/charge_manager_split_design.md",
    "Docs/coding_standard.md",
    "Docs/power_control_split_design.md",
    "Docs/test_report.md",
    "Tests/run_tests.py",
    "Tests/unit/test_common_logic.c",
    "BuildLogs/codex_build_async_fix.log",
    "BuildLogs/uv4_power_control_split_rebuild_20260630.log",
]

OLD_GENERATED_PATHS = [
    "Docs",
    "Tests",
    "GD32G553_Code_Overview.md",
]

MOVED_OLD_PATHS = [
    "Driver/app_tasks.c",
    "Driver/app_tasks.h",
    "Driver/balance_manager.c",
    "Driver/balance_manager.h",
    "Driver/battery_model.c",
    "Driver/battery_model.h",
    "Driver/bms_board_config.h",
    "Driver/bms_state.c",
    "Driver/bms_state.h",
    "Driver/bms_types.h",
    "Driver/board_support.c",
    "Driver/board_support.h",
    "Driver/charge_manager.c",
    "Driver/charge_manager.h",
    "Driver/fault_log.c",
    "Driver/fault_log.h",
    "Driver/param_storage.c",
    "Driver/param_storage.h",
    "Driver/pi_controller.c",
    "Driver/pi_controller.h",
    "Driver/power_manager.c",
    "Driver/power_manager.h",
    "Driver/power_path_manager.c",
    "Driver/power_path_manager.h",
    "Driver/safety_manager.c",
    "Driver/safety_manager.h",
    "Driver/soc_estimator.c",
    "Driver/soc_estimator.h",
    "Comm/bms_comm_service.c",
    "Comm/bms_comm_service.h",
    "Comm/bms_protocol.c",
    "Comm/bms_protocol.h",
    "Comm/bms_uart.c",
    "Comm/bms_uart.h",
]


def fail(message: str) -> None:
    raise AssertionError(message)


def assert_exists() -> None:
    for rel in EXPECTED_FILES:
        path = FW / rel
        if not path.exists():
            fail(f"missing expected file: {path}")


def assert_old_paths_not_present() -> None:
    for rel in MOVED_OLD_PATHS:
        path = FW / rel
        if path.exists():
            fail(f"old moved path still exists: {path}")


def project_file_paths() -> list[str]:
    tree = ET.parse(PROJECT_FILE)
    paths: list[str] = []
    for node in tree.findall(".//FilePath"):
        if node.text:
            paths.append(node.text.strip())
    return paths


def assert_keil_paths_resolve() -> None:
    paths = project_file_paths()
    project_dir = PROJECT_FILE.parent
    for rel in paths:
        normalized = rel.replace("\\", os.sep)
        resolved = (project_dir / normalized).resolve()
        if not resolved.exists():
            fail(f"Keil FilePath does not exist: {rel} -> {resolved}")


def assert_keil_uses_layers() -> None:
    text = PROJECT_FILE.read_text(encoding="utf-8")
    include_line = next(
        (line for line in text.splitlines() if "<IncludePath>" in line and "..\\Library" in line),
        "",
    )
    for layer in ["App", "BSP", "Common", "Config", "Driver", "Module", "Service"]:
        token = f"..\\{layer}"
        if token not in include_line:
            fail(f"Keil include path missing {token}")

    for old_path in MOVED_OLD_PATHS:
        token = "..\\" + old_path.replace("/", "\\")
        if token in text:
            fail(f"Keil project still references old moved path: {token}")


def assert_charge_manager_fragments() -> None:
    source = (FW / "Module" / "charge_manager.c").read_text(encoding="utf-8")
    last_index = -1
    for fragment in CHARGE_MANAGER_FRAGMENTS:
        token = f'#include "{fragment}"'
        index = source.find(token)
        if index < 0:
            fail(f"charge_manager.c missing internal fragment include: {fragment}")
        if index <= last_index:
            fail(f"charge_manager internal fragment order is wrong near: {fragment}")
        last_index = index


def assert_power_control_fragments() -> None:
    source = (FW / "Driver" / "power_control.c").read_text(encoding="utf-8")
    last_index = -1
    for fragment in POWER_CONTROL_FRAGMENTS:
        token = f'#include "{fragment}"'
        index = source.find(token)
        if index < 0:
            fail(f"power_control.c missing internal fragment include: {fragment}")
        if index <= last_index:
            fail(f"power_control internal fragment order is wrong near: {fragment}")
        last_index = index


def assert_generated_files_grouped() -> None:
    for rel in GENERATED_EXPECTED_FILES:
        path = GENERATED / rel
        if not path.exists():
            fail(f"missing generated file: {path}")

    for rel in OLD_GENERATED_PATHS:
        path = ROOT / rel
        if path.exists():
            fail(f"generated file or folder still at repository root: {path}")


def find_c_compiler() -> str | None:
    for compiler in ("gcc", "clang", "cl"):
        found = shutil.which(compiler)
        if found:
            return compiler
    return None


def run_optional_c_unit_tests() -> None:
    compiler = find_c_compiler()
    if compiler is None:
        print("SKIP optional C unit tests: no gcc, clang, or cl found on PATH")
        return

    build_dir = TEST_ROOT / "build"
    build_dir.mkdir(exist_ok=True)
    exe = build_dir / ("test_common_logic.exe" if os.name == "nt" else "test_common_logic")
    sources = [
        TEST_ROOT / "unit" / "test_common_logic.c",
        FW / "Common" / "pi_controller.c",
        FW / "Common" / "battery_model.c",
        FW / "Module" / "balance_manager.c",
    ]
    include_args = [
        str(FW / "Common"),
        str(FW / "Module"),
    ]

    if compiler == "cl":
        cmd = [compiler, "/nologo", "/W4"]
        cmd.extend(f"/I{path}" for path in include_args)
        cmd.extend(str(path) for path in sources)
        cmd.append(f"/Fe:{exe}")
    else:
        cmd = [compiler, "-std=c99", "-Wall", "-Wextra"]
        cmd.extend(f"-I{path}" for path in include_args)
        cmd.extend(str(path) for path in sources)
        cmd.extend(["-o", str(exe)])

    subprocess.run(cmd, cwd=ROOT, check=True)
    subprocess.run([str(exe)], cwd=ROOT, check=True)
    print("PASS optional C unit tests")


def main() -> int:
    checks = [
        ("layer files exist", assert_exists),
        ("old moved paths removed", assert_old_paths_not_present),
        ("Keil FilePath entries resolve", assert_keil_paths_resolve),
        ("Keil include paths use layers", assert_keil_uses_layers),
        ("charge manager fragments included", assert_charge_manager_fragments),
        ("power control fragments included", assert_power_control_fragments),
        ("generated files grouped", assert_generated_files_grouped),
    ]

    for name, check in checks:
        check()
        print(f"PASS {name}")

    run_optional_c_unit_tests()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"FAIL {exc}", file=sys.stderr)
        raise SystemExit(1)
