#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
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
    "Module/balance_manager.c",
    "Module/charge_manager.c",
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

    build_dir = ROOT / "Tests" / "build"
    build_dir.mkdir(exist_ok=True)
    exe = build_dir / ("test_common_logic.exe" if os.name == "nt" else "test_common_logic")
    sources = [
        ROOT / "Tests" / "unit" / "test_common_logic.c",
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
