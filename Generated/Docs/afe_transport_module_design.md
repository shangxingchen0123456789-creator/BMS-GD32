# AFE Transport Module Design

Update date: 2026-06-30

## Purpose

`afe_gd30bm2016.c` currently mixes BM2016 register protocol, configuration,
sampling, FET control, and bit-banged I2C transport in one large driver file.
The transport code is hardware-facing and stable, while the BM2016 protocol
logic changes more often during bring-up.

This round extracts the GPIO/I2C transport layer into a private driver module.

## Design

- Keep the public `afe_gd30bm2016.h` API unchanged.
- Add `afe_gd30bm2016_transport.c/.h` under `GD32G553/Driver`.
- Move the bit-banged I2C GPIO setup, start/stop, byte read/write, bus
  recovery, raw register read/write, millisecond delay, and I2C mutex into the
  transport module.
- Keep BM2016 address selection, CRC framing, direct register access, data
  memory commands, protection recovery, FET control, and debug state in
  `afe_gd30bm2016.c`.

## Benefits

- The large AFE driver loses the board-level I2C details and becomes easier to
  inspect as a chip protocol driver.
- The transport module owns the synchronization primitive that protects the
  physical I2C bus.
- Future board-port work can swap or tune the transport implementation without
  changing BM2016 register logic.

## Boundaries

- No BM2016 register address, threshold, CRC polynomial, configuration table,
  protection behavior, FET command sequence, or public API changes.
- No new task, queue, interrupt, heap allocation pattern, or higher-level
  service dependency.
- The module does not decide whether the AFE link is ready; readiness and
  recovery cadence remain in the main BM2016 driver.

## Validation

- `Generated/Tests/run_tests.py` checks that the transport source/header exist,
  are referenced by Keil, and the main driver calls the transport API instead
  of keeping raw I2C byte helpers locally.
- Existing generated tests continue to reject `.inc` files and stale layer
  paths.
- Keil UV4 rebuild is run and the log is stored in `Generated/BuildLogs`.
