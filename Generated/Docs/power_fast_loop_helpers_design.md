# Power Fast Loop Helpers Design

Update date: 2026-06-30

## Purpose

`Power_Control_Fast_Loop` handles fast safety checks, current ramping, PI
selection, duty limiting, PWM mapping, and recovery service in one long
function. The behavior is validated, but the function is difficult to audit
because protection exits and control-loop math are interleaved.

This round extracts private helpers inside `power_control_api.c` so the fast
loop becomes a short orchestration path.

## Design

- Keep `Power_Control_Fast_Loop` and all public `power_control.h` APIs
  unchanged.
- Add a private per-loop context for current reference, measured current,
  output current, light-load guard, and calculated duty.
- Extract helpers for:
  - pre-control protection and early exits.
  - current feedback/ramp calculation and OCP check.
  - coast handling before closed-loop duty calculation.
  - PI step calculation.
  - duty limiting and PWM mapping.
  - final PWM apply/recovery service.
- Keep all helpers `static` and private to `power_control_api.c`.

## Benefits

- Safety checks remain ordered and easier to review.
- PI and duty-limit math is separated from fault early exits.
- Future regression tests can check helper presence without changing public
  interfaces.

## Boundaries

- No threshold, trip reason, PI constant, or PWM mapping behavior change.
- No public header or Keil project change.
- No new allocation, task, interrupt, or synchronization primitive.

## Validation

- `Generated/Tests/run_tests.py` checks that `Power_Control_Fast_Loop` uses the
  extracted helper names and remains a short orchestration function.
- Existing tests continue to reject `.inc` files and internal context aliases.
- Keil UV4 rebuild is run and the log is stored in `Generated/BuildLogs`.
