# op-mazda-testkit

Mazda longitudinal reverse-engineering notes, helper scripts, and experimental Panda runners extracted from active work on a `2022 Mazda CX-5`.

## Contents

- [`docs/MAZDA_LONGITUDINAL_DISCORD_SUMMARY.md`](https://github.com/yummydirtx/op-mazda-testkit/blob/main/docs/MAZDA_LONGITUDINAL_DISCORD_SUMMARY.md)
  Shorter shareable status summary for Discord/forums.
- [`docs/MAZDA_LONGITUDINAL_RESEARCH.md`](https://github.com/yummydirtx/op-mazda-testkit/blob/main/docs/MAZDA_LONGITUDINAL_RESEARCH.md)
  Full research notes, message findings, and implementation history.
- [`examples/`](https://github.com/yummydirtx/op-mazda-testkit/tree/main/examples)
  Offline analyzers, replay extractors, UDS probes, and Panda test runners.
- [`opendbc/car/mazda/longitudinal_experimental.py`](https://github.com/yummydirtx/op-mazda-testkit/blob/main/opendbc/car/mazda/longitudinal_experimental.py)
  Mazda longitudinal helper module used by the test scripts.

## What This Repo Currently Shows

- Stock Mazda MRCC longitudinal commands are present on CAN.
- `0x764` on bus `0` is radar-related on the tested `2022 CX-5`.
- `COMMUNICATION_CONTROL (0x28)` was not accepted there on the tested car.
- Holding `0x764` in `programming` session suppresses:
  - `0x21b`
  - `0x21c`
  - radar tracks `0x361` to `0x366`
- Replacing only `0x21b + 0x21c` was enough to pass parked and forward-creep neutral tests while the radar session was actively held open.

## Scope

This is a focused artifact repo, not a standalone fork of openpilot or opendbc.

Included:

- Mazda longitudinal documentation
- Mazda reverse-engineering scripts
- Mazda experimental Panda runners
- Mazda helper module used by those scripts

Not included:

- full upstream source tree
- raw `rlog.zst` captures
- device-specific secrets or configs

## Usage Notes

Most scripts expect an existing openpilot/opendbc environment plus Panda access.

For offline log work, a typical invocation looks like:

```bash
cd /path/to/op-mazda-testkit
PYTHONPATH=/path/to/op-mazda-testkit \
python examples/analyze_mazda_acc_rlog.py /path/to/segment.rlog.zst
```

For on-car Panda tests, stop `pandad` first and read the docs before sending anything.

## Safety

These experiments can suppress or disturb radar-based systems.

- Assume high-speed radar-based AEB / SBS may be unavailable during radar-session tests.
- Use a closed area only.
- Keep a foot over the brake.
- Do not road-test unfinished longitudinal control.
