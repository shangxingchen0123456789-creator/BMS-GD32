#!/usr/bin/env python3
import os
import re
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
    "Driver/afe_gd30bm2016_transport.c",
    "Driver/afe_gd30bm2016_transport.h",
    "Driver/flash_storage.c",
    "Driver/power_control.c",
    "Driver/power_control_api.c",
    "Driver/power_control_internal.h",
    "Driver/power_control_limits.c",
    "Driver/power_control_mode.c",
    "Driver/power_control_safety.c",
    "Driver/power_pwm.c",
    "Driver/power_pwm.h",
    "Module/balance_manager.c",
    "Module/charge_manager.c",
    "Module/charge_manager_commands.c",
    "Module/charge_manager_control.c",
    "Module/charge_manager_faults.c",
    "Module/charge_manager_internal.h",
    "Module/charge_manager_params.c",
    "Module/charge_manager_path.c",
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

CHARGE_MANAGER_MODULES = [
    "Module/charge_manager_params.c",
    "Module/charge_manager_path.c",
    "Module/charge_manager_faults.c",
    "Module/charge_manager_control.c",
    "Module/charge_manager_commands.c",
    "Module/charge_manager_internal.h",
]

POWER_CONTROL_MODULES = [
    "Driver/power_control_safety.c",
    "Driver/power_control_limits.c",
    "Driver/power_control_mode.c",
    "Driver/power_control_api.c",
    "Driver/power_control_internal.h",
]

GENERATED_EXPECTED_FILES = [
    "README.md",
    "Docs/architecture.md",
    "Docs/afe_transport_module_design.md",
    "Docs/changelog.md",
    "Docs/charge_manager_update_helpers_design.md",
    "Docs/charge_manager_split_design.md",
    "Docs/coding_standard.md",
    "Docs/internal_context_access_design.md",
    "Docs/internal_context_split_design.md",
    "Docs/power_fast_loop_helpers_design.md",
    "Docs/power_control_split_design.md",
    "Docs/power_pwm_module_design.md",
    "Docs/remove_inc_modules_design.md",
    "Docs/test_report.md",
    "Tests/run_tests.py",
    "Tests/unit/test_common_logic.c",
    "BuildLogs/codex_build_async_fix.log",
    "BuildLogs/uv4_power_control_split_rebuild_20260630.log",
    "BuildLogs/uv4_power_pwm_module_rebuild_20260630.log",
    "BuildLogs/uv4_remove_inc_modules_rebuild_20260630.log",
    "BuildLogs/uv4_internal_context_access_rebuild_20260630.log",
    "BuildLogs/uv4_internal_context_split_rebuild_20260630.log",
    "BuildLogs/uv4_charge_update_helpers_rebuild_20260630.log",
    "BuildLogs/uv4_power_fast_loop_helpers_rebuild_20260630.log",
    "BuildLogs/uv4_afe_transport_module_rebuild_20260630.log",
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


def assert_no_inc_files() -> None:
    inc_files = list(FW.rglob("*.inc"))
    if inc_files:
        fail("project still contains .inc files: " + ", ".join(str(path) for path in inc_files))


def assert_real_internal_modules() -> None:
    paths = project_file_paths()
    project_tokens = {path.replace("/", "\\") for path in paths}
    project_text = PROJECT_FILE.read_text(encoding="utf-8")

    if ".inc" in project_text:
        fail("Keil project still references .inc files")

    for rel in POWER_CONTROL_MODULES + CHARGE_MANAGER_MODULES:
        project_rel = "..\\" + rel.replace("/", "\\")
        if project_rel not in project_tokens:
            fail(f"Keil project missing module: {project_rel}")

    for source_path in list((FW / "Driver").glob("power_control*.c")) + \
                       list((FW / "Module").glob("charge_manager*.c")):
        source = source_path.read_text(encoding="utf-8")
        if ".inc" in source:
            fail(f"source still references .inc: {source_path}")


def assert_internal_context_access_explicit() -> None:
    alias_pattern = re.compile(r"\bs_[A-Za-z0-9_]+")
    target_files = [
        FW / "Driver" / "power_control_internal.h",
        FW / "Module" / "charge_manager_internal.h",
    ]
    target_files.extend((FW / "Driver").glob("power_control*.c"))
    target_files.extend((FW / "Module").glob("charge_manager*.c"))

    for source_path in target_files:
        source = source_path.read_text(encoding="utf-8")
        if "#define s_" in source:
            fail(f"internal context alias macro remains: {source_path}")

        aliases = sorted(set(alias_pattern.findall(source)))
        if aliases:
            fail(f"internal context aliases remain in {source_path}: {', '.join(aliases)}")


def assert_internal_contexts_are_grouped() -> None:
    power_header = (FW / "Driver" / "power_control_internal.h").read_text(encoding="utf-8")
    charge_header = (FW / "Module" / "charge_manager_internal.h").read_text(encoding="utf-8")

    for token in [
        "power_control_loop_context_t loop",
        "power_control_fault_context_t fault",
        "power_control_transition_context_t transition",
        "power_control_feedback_context_t feedback",
    ]:
        if token not in power_header:
            fail(f"power_control_context_t missing grouped field: {token}")

    for token in [
        "charge_manager_config_context_t config",
        "charge_manager_control_context_t control",
        "charge_manager_digital_context_t digital",
        "charge_manager_fault_context_t fault",
        "charge_manager_manual_fet_context_t manualFet",
        "charge_manager_path_context_t path",
    ]:
        if token not in charge_header:
            fail(f"charge_manager_context_t missing grouped field: {token}")

    target_sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in list((FW / "Driver").glob("power_control*.c")) +
                   list((FW / "Module").glob("charge_manager*.c"))
    )
    for token in [
        "g_power_control.loop.",
        "g_power_control.fault.",
        "g_power_control.transition.",
        "g_power_control.feedback.",
        "g_charge_manager.config.",
        "g_charge_manager.control.",
        "g_charge_manager.digital.",
        "g_charge_manager.fault.",
        "g_charge_manager.manualFet.",
        "g_charge_manager.path.",
    ]:
        if token not in target_sources:
            fail(f"grouped context access missing from modules: {token}")


def assert_power_pwm_module() -> None:
    power_control = (FW / "Driver" / "power_control.c").read_text(encoding="utf-8")
    power_control_internal = (FW / "Driver" / "power_control_internal.h").read_text(encoding="utf-8")
    power_pwm = (FW / "Driver" / "power_pwm.c").read_text(encoding="utf-8")
    project_paths = project_file_paths()

    if ".inc" in power_control:
        fail("power_control.c still includes an internal fragment")
    if '#include "power_pwm.h"' not in power_control_internal:
        fail("power_control_internal.h does not include power_pwm.h")
    if '#include "power_control.h"' in power_pwm:
        fail("power_pwm.c must not depend on power_control.h")
    if "..\\Driver\\power_pwm.c" not in project_paths:
        fail("Keil project does not compile power_pwm.c")
    if "..\\Driver\\power_pwm.h" not in project_paths:
        fail("Keil project does not list power_pwm.h")


def assert_charge_update_uses_helpers() -> None:
    source = (FW / "Module" / "charge_manager.c").read_text(encoding="utf-8")
    required_helpers = [
        "Charge_Manager_Load_Update_Context",
        "Charge_Manager_Collect_Update_Faults",
        "Charge_Manager_Dispatch_Update_State",
        "Charge_Manager_Handle_Normal_Charge",
        "Charge_Manager_Apply_Final_Path_State",
        "Charge_Manager_Fill_Status",
        "Charge_Manager_Store_Update_Context",
    ]

    for helper in required_helpers:
        if source.count(helper) < 2:
            fail(f"Charge_Manager_Update helper is not defined and used: {helper}")

    marker = "void Charge_Manager_Update(uint32_t period_ms,"
    update_start = source.find(marker)
    if update_start < 0:
        fail("Charge_Manager_Update implementation not found")
    update_body_lines = source[update_start:].splitlines()
    if len(update_body_lines) > 40:
        fail("Charge_Manager_Update orchestration body grew too long")


def assert_power_fast_loop_uses_helpers() -> None:
    source = (FW / "Driver" / "power_control_api.c").read_text(encoding="utf-8")
    required_helpers = [
        "Power_Control_Fast_Loop_Precheck",
        "Power_Control_Fast_Loop_Prepare_Current",
        "Power_Control_Fast_Loop_Update_Pi",
        "Power_Control_Fast_Loop_Update_Duty",
        "Power_Control_Fast_Loop_Map_Duty",
        "Power_Control_Fast_Loop_Finalize",
    ]

    for helper in required_helpers:
        if source.count(helper) < 2:
            fail(f"Power_Control_Fast_Loop helper is not defined and used: {helper}")

    marker = "void Power_Control_Fast_Loop(const bms_power_sample_t *sample)"
    apply_marker = "void Power_Control_Apply(const bms_power_sample_t *sample)"
    update_start = source.find(marker)
    update_end = source.find(apply_marker, update_start)
    if update_start < 0 or update_end < 0:
        fail("Power_Control_Fast_Loop implementation not found")
    update_body_lines = source[update_start:update_end].splitlines()
    if len(update_body_lines) > 35:
        fail("Power_Control_Fast_Loop orchestration body grew too long")


def assert_afe_transport_module() -> None:
    driver_source = (FW / "Driver" / "afe_gd30bm2016.c").read_text(encoding="utf-8")
    transport_source = (FW / "Driver" / "afe_gd30bm2016_transport.c").read_text(encoding="utf-8")
    project_paths = project_file_paths()

    for token in [
        "Afe_I2c_Write_Byte",
        "Afe_I2c_Read_Byte",
        "Afe_I2c_Start",
        "Afe_I2c_Stop",
        "s_i2c_mutex",
    ]:
        if token in driver_source:
            fail(f"AFE main driver still owns transport detail: {token}")
        if token not in transport_source:
            fail(f"AFE transport module missing expected detail: {token}")

    for token in [
        "Afe_Gd30bm2016_Transport_Write_Raw",
        "Afe_Gd30bm2016_Transport_Read_Raw",
        "Afe_Gd30bm2016_Transport_Lock",
        "Afe_Gd30bm2016_Transport_Delay_Ms",
    ]:
        if token not in driver_source:
            fail(f"AFE main driver does not use transport API: {token}")

    for rel in [
        "..\\Driver\\afe_gd30bm2016_transport.c",
        "..\\Driver\\afe_gd30bm2016_transport.h",
    ]:
        if rel not in project_paths:
            fail(f"Keil project missing AFE transport file: {rel}")


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
        ("no inc files remain", assert_no_inc_files),
        ("real internal modules referenced", assert_real_internal_modules),
        ("internal context access is explicit", assert_internal_context_access_explicit),
        ("internal contexts are grouped", assert_internal_contexts_are_grouped),
        ("power pwm module extracted", assert_power_pwm_module),
        ("charge update uses helpers", assert_charge_update_uses_helpers),
        ("power fast loop uses helpers", assert_power_fast_loop_uses_helpers),
        ("afe transport module extracted", assert_afe_transport_module),
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
