# Mazda CX-5 Longitudinal Reverse-Engineering Status

This document is a shareable status update for Mazda longitudinal-control reverse engineering on a `2022 Mazda CX-5` using `comma 4`, direct Panda scripts, and the Mazda tooling added in this repo.

It is intended to answer four questions:

1. What is already known about Mazda MRCC longitudinal on CAN?
2. What was actually tested on-car?
3. What is now proven versus still hypothetical?
4. What should be tested next?

## Executive Summary

The most important result is:

- `0x764` on bus `0` is definitely part of the forward-radar path on this car.
- `COMMUNICATION_CONTROL (0x28)` at `0x764` does **not** work on this `2022 CX-5`; it returns NRC `0x11` (`service not supported`) in every tested session.
- However, entering `programming` session on `0x764` and holding it open with raw `0x3E 0x80` tester-present **does** suppress:
  - `0x21b` / `CRZ_INFO`
  - `0x21c` / `CRZ_CTRL`
  - radar tracks `0x361` to `0x366`
- During that same active session, these messages **continue** at stock rate:
  - `0x21f` / `CRZ_EVENTS`
  - `0xfd` / `GAS`
  - `0x167` / `MORE_GAS`
- Replacing only `0x21b + 0x21c` while holding the radar in `programming` session is enough to:
  - keep the car calm in `Park`
  - allow a forward idle-creep test in `Drive`
  - prevent the loud front-radar fault from appearing during the active test window
- Faults still appear when the script ends, and camera/SBS/SCBS faults can appear during the active window, so `0x21b + 0x21c` are necessary but not yet sufficient for a clean full replacement.

The project is therefore no longer blocked at "can we suppress the stock owner at all?".

The current state is:

- suppression path found
- minimum accepted replacement path partially found
- actual longitudinal actuation still not proven yet

## Vehicle and Test Context

Target vehicle:

- `MAZDA_CX5_2022`

Test setup:

- comma 4 installed on the car
- direct Panda scripts run on-device
- `pandad` stopped during direct tests
- camera harness path in use, not OBD-only sniffing
- tests performed in a closed area

Important safety context:

- These tests can disturb or suppress radar functionality.
- High-speed radar-based AEB / SBS should be assumed unavailable during radar-suppression experiments.
- Camera/SCBS-related warnings were observed during active testing.

## What Was Already Known From Offline Log Analysis

Before the on-car Panda tests, offline `rlog.zst` analysis had already established:

- Mazda stock MRCC already publishes a signed longitudinal command family on CAN.
- The primary messages are:
  - `0x21b` / `CRZ_INFO`
  - `0x21c` / `CRZ_CTRL`
  - `0x21f` / `CRZ_EVENTS`
  - `0xfd` / `GAS`
  - `0x167` / `MORE_GAS`
- The strongest signed accel/brake request is `CRZ_INFO.ACCEL_CMD`.
- `CRZ_EVENTS.ACCEL_CMD` tracks the same command at lower resolution.
- `GAS.GAS_CMD` is real but appears secondary.

Other important offline findings:

- MRCC standby keeps the command family alive, but `CRZ_INFO.ACCEL_CMD` sits at an invalid/inactive sentinel and is not directly usable.
- `CRZ_CTRL` is the state-gating message. It is not optional metadata.
- The message family is phase-split:
  - phase A: `0xfd`, `0x167`, `0x21f`
  - phase B: `0x21b`, `0x21c`
- Historical Mazda behavior strongly suggested that `0x21b + 0x21c` might be the minimum "keep the car logically happy" set.

## Useful Discord Findings

The most useful Discord statements, in hindsight, were:

- older Mazda testing claimed radar disable only worked in `programming` session
- older Mazda testing claimed the car mainly cared about `0x21b` and `0x21c`
- newer-gen users reported `0x764` experiments, but those reports were not enough by themselves to prove successful disable

One important caveat:

- the generic `disable_ecu()` helper can log `ecu disabled` even when the ECU simply stops responding to the request path being used
- that means old Discord logs showing `ecu disabled` are weaker evidence than a direct UDS probe that checks actual message disappearance

## Scripts Added / Used

Offline analysis:

- `examples/analyze_mazda_acc_rlog.py`
- `examples/compare_mazda_acc_rlogs.py`
- `examples/prepare_mazda_longitudinal_templates.py`
- `examples/extract_mazda_longitudinal_replay.py`

On-car Panda probing:

- `examples/test_mazda_radar_disable_with_panda.py`
- `examples/run_mazda_longitudinal_replay_with_panda.py`
- `examples/run_mazda_crz_replacement_with_panda.py`

Mazda helper module:

- `opendbc/car/mazda/longitudinal_experimental.py`

## On-Car UDS Findings

### 1. `0x764` is definitely a real radar-related endpoint on this car

Direct Panda UDS testing showed:

- `0x764` responds to diagnostic session control
- the car can show a real `Front Radar Sensor System Malfunction` while probing this path
- therefore `0x764` is not a random or irrelevant ECU for this platform

### 2. `COMMUNICATION_CONTROL (0x28)` does not work on this `2022 CX-5`

Tested result:

- `0x28` returns NRC `0x11` (`service not supported`)

This was observed in:

- `default`
- `extended`
- `programming`
- `safety system diagnostic` was not accepted as a usable session either

So the original simple theory:

- "enter session"
- "send `0x28`"
- "radar goes silent"

is **not** valid for this specific vehicle as-tested.

### 3. `programming` session alone is enough to suppress part of the radar output

This was the breakthrough.

Using a `session-only` test on `0x764` in `programming` session while sending raw tester-present `0x3E 0x80`, the observed CAN counts over a 5-second window changed as follows:

Before active session:

- `0x21b`: `250` (`50 Hz`)
- `0x21c`: `250` (`50 Hz`)
- `0x21f`: `250` (`50 Hz`)
- `0xfd`: `250` (`50 Hz`)
- `0x167`: `250` (`50 Hz`)
- `0x361` to `0x366`: `50` each (`10 Hz`)

During active `programming` session:

- `0x21b`: `1`
- `0x21c`: `1`
- `0x21f`: `250`
- `0xfd`: `249`
- `0x167`: `250`
- `0x361` to `0x366`: `0`

What that proves:

- `programming` session at `0x764` suppresses `CRZ_INFO`, `CRZ_CTRL`, and radar tracks
- it does **not** suppress `CRZ_EVENTS`, `GAS`, or `MORE_GAS`

This is the first hard on-car proof that the stock owner can be partially displaced without a hardware radar interceptor.

## On-Car Replacement Findings

### 1. Replacing only `0x21b + 0x21c` is enough to keep the car calm temporarily

A dedicated runner was used to:

- enter `programming` session on `0x764`
- keep the session alive with raw `0x3E 0x80`
- replay only `0x21b` and `0x21c` at `50 Hz`

Initial `Park` test with:

- engaged-cruise replay window
- `info_accel_cmd = 0.0`

Result:

- the script ran cleanly
- the loud `Front Radar Sensor System Malfunction` did **not** occur during the active 5-second window
- the radar fault appeared immediately after the script ended

Interpretation:

- the active replacement of `0x21b + 0x21c` is doing real work
- the fault on exit is consistent with the stock suppressed frames disappearing again once the session/replacement stops

### 2. Additional camera/SBS warnings still appear

During active replacement testing, the following warnings were reported:

- `Front Camera Sensor System Malfunction`
- `Smart Brake System Malfunction`
- `Forward Smart City Brake Support Malfunction`

These appeared without the same loud failure behavior as the main radar fault.

Interpretation:

- `0x21b + 0x21c` are enough to hold the basic cruise-command path together temporarily
- they are not enough to keep all radar/camera/SBS dependencies happy
- this lines up with the expectation that radar-track disappearance affects additional systems beyond the cruise command path itself

### 3. Exit handoff did not eliminate stored faults

An exit handoff was added to keep sending `0x21b + 0x21c` briefly after stopping tester-present, while waiting for radar tracks to return.

Observed result:

- quiet camera/SBS warnings still appeared during the active run
- the loud front-radar warning still fired once the run ended

Interpretation:

- teardown timing is not the only issue
- real faults are still being set during the active replacement window

### 4. DTC reads confirm real radar/camera/DSC faults

Post-test DTC reads showed:

- radar module at `0x764`:
  - `U023A00`
  - `U030109`
  - `U031609`
- forward camera at `0x706`:
  - multiple confirmed CAN communication and invalid-data faults
- DSC/ABS at `0x760`:
  - `U023A00`
  - `U040500`
  - `U010400`

The most important practical conclusion is:

- the warnings are not just cosmetic cluster behavior
- the car is storing real faults in the radar, camera, and DSC path
- the DSC fault aligns with the observed degradation of ACC behavior at low speed after testing

This points to two likely root causes:

1. The replacement `0x21b` / `0x21c` pair is accepted well enough to function temporarily, but message integrity is not yet good enough to avoid module complaints.
2. Suppressing radar tracks without replacing them is creating secondary camera/SBS/DSC failures even if the core cruise-command path survives.

### 5. Forward idle-creep test passed

A low-risk forward-creep test in `Drive` with:

- `info_accel_cmd = 0.0`
- same `programming` session hold
- same `0x21b + 0x21c` replacement

Result:

- pass
- no loud radar fault during the active test window
- car behaved normally enough to perform a forward idle-creep test

This is the strongest practical result so far because it shows the suppression-plus-replacement path works while the vehicle is actually rolling.

## What Is Proven Now

The following statements are now supported by direct evidence on this `2022 CX-5`:

1. `0x764` on bus `0` is tied to the forward-radar / MRCC path.
2. `COMMUNICATION_CONTROL (0x28)` is not supported there on this vehicle.
3. `programming` session at `0x764` suppresses:
   - `0x21b`
   - `0x21c`
   - `0x361` to `0x366`
4. `programming` session at `0x764` does not suppress:
   - `0x21f`
   - `0xfd`
   - `0x167`
5. Replacing `0x21b + 0x21c` alone is enough to temporarily satisfy the core cruise-control path better than doing nothing.
6. That partial replacement can survive at least:
   - parked neutral test
   - forward idle-creep test
7. The current implementation still sets real radar/camera/DSC DTCs and cannot yet be treated as a clean handoff.

## What Is Still Not Proven

These questions remain open:

1. Does changing `CRZ_INFO.ACCEL_CMD` actually produce controllable acceleration/braking while `0x21b + 0x21c` are being replaced?
2. Is `0x21b + 0x21c` enough for actual longitudinal actuation, or are `0x21f`, `0xfd`, or `0x167` also required companions?
3. Are the camera/SBS/SCBS warnings harmless side effects for early testing, or do they indicate another required replacement path?
4. Can the system be kept alive cleanly for long enough without post-run faults?
5. Is there a cleaner exit strategy that hands control back without immediate radar faults?
6. Are radar-track replays needed to keep the camera/SBS path happy even if longitudinal itself only needs `0x21b + 0x21c`?
7. Is the current `CRZ_INFO` patch/checksum logic accurate enough, or is imperfect `0x21b` fidelity causing part of the DSC complaint?

## Current Working Hypothesis

The current best model for this `2022 CX-5` is:

1. `0x764` owns or gates the stock radar longitudinal path.
2. Putting it into `programming` session suppresses the most important cruise-control outputs:
   - `CRZ_INFO`
   - `CRZ_CTRL`
   - radar tracks
3. The car will temporarily accept externally supplied `0x21b + 0x21c` in place of the suppressed originals.
4. `0x21f`, `0xfd`, and `0x167` may not be required to get basic longitudinal actuation started, because the stock versions remain alive during the active session.
5. Separate non-longitudinal faults are likely caused by the disappearance of radar tracks and/or related perception-side state.

If this model holds, the simplest viable Mazda-long path on this car may be:

- hold `0x764` in `programming`
- transmit replacement `0x21b + 0x21c`
- leave `0x21f`, `0xfd`, `0x167` stock for now

That is a much narrower problem than "fully emulate the entire radar ECU immediately".

## Immediate Next Tests

The next priority is not more disable work. It is first isolating message-fidelity faults, then proving actual longitudinal influence.

### Step 1: Replay captured `CRZ_INFO` unchanged

Before more accel testing, run the same replacement path with captured `CRZ_INFO` replayed unchanged.

Goal:

- determine whether the current faults are caused mainly by:
  - patched `0x21b` / checksum behavior
  - or missing radar-track / perception-side messages

### Step 2: Tiny positive accel pulses

Test:

- same `programming` session hold
- same `0x21b + 0x21c` replacement
- change only `CRZ_INFO.ACCEL_CMD`

Planned values:

- `40`
- `80`

Goal:

- determine whether the vehicle responds beyond normal idle creep

### Step 3: Determine whether `0x21b` alone is enough for actuation

If tiny positive pulses do nothing:

- compare `0x21b + 0x21c` replacement against a version that also begins replacing other companions
- likely order:
  1. keep current `0x21b + 0x21c`
  2. add controlled `0x21f` handling
  3. then revisit `0xfd`
  4. then `0x167` if needed

### Step 4: Separate longitudinal acceptance from radar/camera fault cleanup

Do not block on solving every warning first.

Right now the main question is:

- can the car be influenced longitudinally through this partial replacement path?

Only after that is answered does it make sense to invest more time in:

- radar-track replay
- camera/SBS fault cleanup
- clean exit / handoff behavior

## Practical Conclusion

Mazda longitudinal on this `2022 CX-5` is no longer a theoretical CAN-decoding exercise.

The project now has:

- a proven suppression mechanism for the key stock cruise-control frames
- a proven partial replacement mechanism for the minimum historical command pair
- a rolling test that stayed stable enough to pass a forward idle-creep check

The remaining question is no longer:

- "Can we get rid of the stock owner at all?"

It is now:

- "Can `CRZ_INFO.ACCEL_CMD` through the `programming-session + 0x21b/0x21c replacement` path actually move the car under control?"

That is a much better place to be than before these tests.

## Repo Artifacts

Main reference doc:

- [MAZDA_LONGITUDINAL_RESEARCH.md](https://github.com/yummydirtx/op-mazda-testkit/blob/main/docs/MAZDA_LONGITUDINAL_RESEARCH.md)

Key scripts:

- [analyze_mazda_acc_rlog.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/analyze_mazda_acc_rlog.py)
- [compare_mazda_acc_rlogs.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/compare_mazda_acc_rlogs.py)
- [extract_mazda_longitudinal_replay.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/extract_mazda_longitudinal_replay.py)
- [test_mazda_radar_disable_with_panda.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/test_mazda_radar_disable_with_panda.py)
- [run_mazda_crz_replacement_with_panda.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/run_mazda_crz_replacement_with_panda.py)
- [longitudinal_experimental.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/opendbc/car/mazda/longitudinal_experimental.py)
