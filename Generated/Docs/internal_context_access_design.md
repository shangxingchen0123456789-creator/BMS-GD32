# Internal Context Access Design

Update date: 2026-06-30

## Purpose

The previous round replaced all `.inc` fragments with real `.c` modules. To
keep that migration low risk, `power_control_internal.h` and
`charge_manager_internal.h` temporarily mapped old `s_xxx` names to fields in
the new internal contexts.

This round removes those transition macros. Internal modules now access
`g_power_control` and `g_charge_manager` fields explicitly.

## Design

- Keep the public `power_control.h` and `charge_manager.h` APIs unchanged.
- Keep one private context per module group.
- Replace macro aliases such as `s_power` and `s_params` with explicit context
  field access.
- Keep behavior, constants, fault thresholds, and state transitions unchanged.

## Benefits

- Internal state ownership is visible at the call site.
- New modules no longer hide shared mutable state behind preprocessor aliases.
- Future cleanup can decide which fields should move into smaller private
  sub-contexts without fighting macro indirection.

## Validation

- `Generated/Tests/run_tests.py` rejects `#define s_` state aliases.
- `git diff --check` is run before commit.
- Keil UV4 rebuild is run and the log is stored in `Generated/BuildLogs`.
