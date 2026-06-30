# Internal Context Split Design

Update date: 2026-06-30

## Purpose

The previous round removed `s_xxx` transition aliases and made internal state
access explicit. The remaining issue is that each module group still owns one
large private context with unrelated fields next to each other.

This round splits those private contexts by responsibility while keeping public
APIs, Keil project structure, constants, and behavior unchanged.

## Design

- `power_control_context_t` is split into:
  - `loop`: public state snapshot, PI controllers, duty values, soft-start
    current, and async boost state.
  - `fault`: latched fault status and confirmation counters.
  - `transition`: preconnect and AFE handover state.
  - `feedback`: battery current and voltage feedback cached from the charge
    manager.
- `charge_manager_context_t` is split into:
  - `config`: stored charge parameters.
  - `control`: charge state, run request, charge mode, work mode, and CV done
    counter.
  - `digital`: digital power enable and target setpoints.
  - `fault`: present/latched faults and fault confirmation counters.
  - `path`: path settle and preconnect target state.
  - `manualFet`: manual FET takeover state.

## Benefits

- The owner of each mutable field is easier to identify at the call site.
- Future extraction can move a sub-context with its module without first
  untangling a flat structure.
- The split documents which fields are control-loop state, fault state, path
  state, or command/debug state.

## Boundaries

- This round does not change public headers or command behavior.
- This round does not add new runtime allocation or synchronization.
- This round does not change Keil groups or compile units.
- This round keeps all sub-contexts private to the same module groups.

## Validation

- `Generated/Tests/run_tests.py` checks for the expected sub-context names.
- Existing checks continue to reject `.inc` fragments and `s_xxx` aliases.
- `git diff --check` is run before commit.
- Keil UV4 rebuild is run and the log is stored in `Generated/BuildLogs`.
