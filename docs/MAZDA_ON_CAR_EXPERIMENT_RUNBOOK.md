# Mazda On-Car Experiment Runbook

This runbook is only for the upstream-preferred path documented in this repo:

1. Hold the radar owner off through the working software-only session path.
2. Replace the minimum Mazda longitudinal frames.
3. Prove real longitudinal authority.
4. Move the proven message set into product code later.

It does not cover alternate architectures.

## Prerequisites

- `pandad` stopped on the device
- closed area for low-speed state and warning work
- separate safe venue for above-threshold authority tests
- a replay window JSON produced by `extract_mazda_longitudinal_replay.py`

Example:

```bash
cd /data/op-mazda-testkit
PYTHONPATH=/data/op-mazda-testkit \
python examples/extract_mazda_longitudinal_replay.py \
  /data/media/0/realdata/<segment>/rlog.zst \
  --profile engaged_cruise \
  --frames 16 \
  --output /data/mazda/engaged_cruise_stream.json
```

## Recommended Ladder

Run these in order. Do not skip ahead unless a rung is clearly falsified.

1. `session_only`
2. `pair`
3. `pair_events_replay`
4. `pair_events_patch`

The wrapper stores each run in a timestamped experiment directory with:

- pre-run DTC snapshot
- session-only suppression probe log
- replacement-run log
- post-run DTC clear and re-read log
- `summary.json`

## Wrapper Usage

Run from `op-mazda-testkit`:

```bash
cd /data/op-mazda-testkit
PYTHONPATH=/data/op-mazda-testkit \
python examples/run_mazda_upstream_experiment.py \
  /data/mazda/engaged_cruise_stream.json \
  --experiment pair \
  --engage-button set-minus \
  --info-accel-cmd 0
```

Ten-minute radar-suppression soak with only the session held open:

```bash
cd /data/op-mazda-testkit
PYTHONPATH=/data/op-mazda-testkit \
python examples/run_mazda_upstream_experiment.py \
  /data/mazda/engaged_cruise_stream.json \
  --experiment session_only \
  --probe-before-seconds 5 \
  --probe-during-seconds 600 \
  --probe-progress-interval 60 \
  --skip-after-clear
```

Replay captured `0x21f` with the pair:

```bash
cd /data/op-mazda-testkit
PYTHONPATH=/data/op-mazda-testkit \
python examples/run_mazda_upstream_experiment.py \
  /data/mazda/engaged_cruise_stream.json \
  --experiment pair_events_replay \
  --engage-button set-minus \
  --info-accel-cmd 0
```

Approximate-patch `0x21f` when overriding accel:

```bash
cd /data/op-mazda-testkit
PYTHONPATH=/data/op-mazda-testkit \
python examples/run_mazda_upstream_experiment.py \
  /data/mazda/engaged_cruise_stream.json \
  --experiment pair_events_patch \
  --engage-button set-minus \
  --info-accel-cmd 160
```

Above-min-speed authority check on a safe venue:

```bash
cd /data/op-mazda-testkit
PYTHONPATH=/data/op-mazda-testkit \
python examples/run_mazda_upstream_experiment.py \
  /data/mazda/engaged_cruise_stream.json \
  --experiment pair_events_patch \
  --engage-button set-minus \
  --target-speed-mph 20 \
  --info-accel-cmd 160
```

## What Each Rung Must Prove

### `session_only`

Must prove the current suppression fact pattern still holds on the car under test:

- `0x21b` disappears
- `0x21c` disappears
- radar tracks disappear
- `0x21f`, `0xfd`, and `0x167` remain

If this changes, stop and debug the suppression path first.

### `pair`

Must prove:

- `0x21b + 0x21c` replacement is accepted
- the car stays calm enough during the active window
- warnings and DTC behavior are reproducible

This rung is for state and fault behavior, not final authority proof.

### `pair_events_replay`

Must prove whether replaying captured `0x21f` changes anything meaningful versus `pair`.

Compare directly against the previous rung. Do not change anything else.

### `pair_events_patch`

Must prove whether approximate `0x21f` patching changes authority or fault behavior relative to replay-only.

This is the rung that should answer whether `0x21f` is a real ownership gate or just a companion.

## Venue Split

Use low-speed lots only for:

- suppression validation
- set-speed latch behavior
- fault behavior
- teardown and handoff behavior

Use a safe above-threshold venue only for:

- positive and negative accel authority
- real ownership proof
- comparisons between replayed and patched `0x21f`

## Interpreting Artifacts

Check `summary.json` first, then:

- `02_probe_session_only.log`: frame disappearance ratios
- `03_replacement.log`: decoded RX and TX state, button pulses, set-speed latch, handoff behavior
- `04_dtc_after_clear.log`: which ECUs latched faults during the run

The key question for the ladder is simple:

- did the added rung change authority or only change warnings?

If a rung only changes warnings, move to the next documented rung.
If a rung changes authority, freeze the ladder there and investigate only that delta.
