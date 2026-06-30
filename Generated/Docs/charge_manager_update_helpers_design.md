# Charge Manager Update Helpers Design

Update date: 2026-06-30

## Purpose

`Charge_Manager_Update` now owns several responsibilities in one long function:
context snapshotting, fault handling, mode dispatch, charge-state transition,
power-path driving, status filling, and context write-back. The behavior is
working, but the function is hard to audit.

This round extracts local helper functions inside `charge_manager.c` so the
main update path reads as a short orchestration flow.

## Design

- Keep `charge_manager.c` as the facade and main state-machine file.
- Add small private structs for a per-update runtime snapshot and path result.
- Extract helpers for:
  - loading and storing the internal context snapshot.
  - collecting faults for BMS or digital-power operation.
  - handling fault/manual/digital/idle dispatch before normal charging.
  - advancing the normal charge state and driving the power path.
  - applying final path ownership rules.
  - filling `bms_status_t`.
- Keep helper functions `static` and private to `charge_manager.c`.

## Benefits

- The public `Charge_Manager_Update` function becomes easier to review.
- State writes remain centralized, which reduces accidental partial updates.
- Future tests can target helper boundaries without changing public APIs.

## Boundaries

- No public header, command ID, parameter range, or Keil project change.
- No new task, queue, allocation, or RTOS synchronization primitive.
- No state-machine behavior change is intended.

## Validation

- `Generated/Tests/run_tests.py` checks that `Charge_Manager_Update` uses the
  extracted helper names.
- Existing structure tests still check module layout, no `.inc` fragments, and
  grouped internal context access.
- Keil UV4 rebuild is run and the log is stored in `Generated/BuildLogs`.
