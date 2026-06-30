# Remove INC Modules Design

Update date: 2026-06-30

## Purpose

The previous refactor used `.inc` fragments to split long files with low
behavior risk. This round removes that intermediate form and converts every
project-owned `.inc` file into a real `.c` compilation unit.

## Design

`power_control` and `charge_manager` both had fragments that depended on
static state from the facade file. To keep behavior unchanged while making
those fragments real modules, each group now owns one internal context:

- `power_control_internal.h` declares `power_control_context_t` and internal
  helper prototypes.
- `charge_manager_internal.h` declares `charge_manager_context_t` and internal
  helper prototypes.

The public headers remain unchanged. Internal modules include only their
matching internal header and keep the existing control logic, limits, fault
rules, and command behavior.

## Boundaries

- This round does not change PWM timing, charge thresholds, fault masks,
  current limits, or state-machine decisions.
- The internal headers are not public application APIs. They are only for
  cooperation among the implementation files of one module group.
- Keil compiles each former fragment as an independent `.c` file.

## Validation

- `Generated/Tests/run_tests.py` checks that no `.inc` files remain and that
  Keil references the new module files.
- `git diff --check` is run before commit.
- Keil UV4 command-line build is run and its log is stored in
  `Generated/BuildLogs`.
