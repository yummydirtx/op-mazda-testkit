# Mazda CX-5 Longitudinal Research Notes

## Scope

This document captures the current state of Mazda longitudinal-control research for `MAZDA_CX5_2022` in this repository, the evidence gathered so far, the open questions, and the recommended next steps.

The target is software-only longitudinal control for Mazda MRCC-equipped vehicles using the existing Mazda openpilot/opendbc integration as the base.

## Current Goal

The goal is to move from:

- Mazda lateral control working
- radar tracks available
- BSM available
- stock MRCC still controlling gas and brake

to:

- openpilot/sunnypilot generating the Mazda longitudinal command stream
- stock longitudinal controller disabled or superseded cleanly
- safety and tuning implemented in a way that is mergeable and defensible

## Current Repo State

The most relevant existing Mazda work in this repo is already in place for lateral and perception:

- `opendbc/car/mazda/interface.py`
- `opendbc/car/mazda/carcontroller.py`
- `opendbc/car/mazda/radar_interface.py`
- `opendbc/car/mazda/carstate.py`
- `opendbc/car/mazda/mazdacan.py`
- `opendbc/safety/modes/mazda.h`

The important state today is:

- Mazda longitudinal is not implemented.
- `openpilotLongitudinalControl` is still false for Mazda.
- `pcmCruise` is still true for Mazda.
- The Mazda controller only sends steering, HUD, and cruise-button traffic.
- Panda safety only whitelists Mazda LKAS, LKAS HUD, and cruise-button TX.
- Radar parsing exists for `0x361` to `0x366`, but tracks `0x365` and `0x366` are still treated as unusable because `RELV_OBJ` is not decoded reliably there.

In short: perception support exists, but the actuation path for gas/brake does not.

## Related Community Context

The most relevant community thread is:

- `https://community.sunnypilot.ai/t/longitudinal-for-mazda/1482`

The useful conclusions from that discussion are:

- The preferred mergeable path is software-only.
- The likely Mazda longitudinal owner is the MRCC radar/controller path on CAN.
- A likely implementation path is:
  - identify the stock longitudinal ECU
  - disable or silence its longitudinal TX using UDS
  - replace those messages from openpilot/sunnypilot
- A fallback idea discussed in the thread is to keep MRCC armed but not set, then inject the engaged-style longitudinal messages while stock ACC is in standby.
- A major caveat is the AEB/radar-safety interaction. Disabling radar TX may remove or alter stock AEB behavior unless it is emulated or otherwise preserved.

The thread was directionally useful, but it did not establish which exact Mazda messages must be replaced, nor whether the standby-injection path is actually viable.

## Later Discord Revelations

Additional Mazda-specific discussion from late 2025 in the comma.ai Discord materially sharpened the implementation picture.

The most useful takeaways were:

- Multiple people independently repeated that the OBD port on newer Mazdas appears firewalled for the messages needed for deeper reverse engineering, while the camera-side harness splice exposes the needed traffic.
- The most consistent software-only longitudinal path discussed was:
  - UDS communication-control disable of the radar at `0x764` on bus `0`
  - keep the radar disabled with tester-present traffic if needed
  - replace the factory longitudinal messages from openpilot/sunnypilot
- A specific disable attempt was reported using:
  - `addr = 0x764`
  - `bus = 0`
  - `com_request = b'\x28\x03\x01'`
  - with the note that `b'\x28\x01\x01'` may be a better choice if RX should remain enabled
- One developer reported being able to send fields on `CRZ_CTRL` and see visible cluster/HUD effects, which supports the idea that at least parts of this message family are writable and accepted by the car.
- A particularly important historical note was that older Mazda VCM-based implementations only emitted `0x21b` and `0x21c`, and newer radar units appear to have taken over the same role/addressing. This raises the possibility that the minimum viable replacement set may be smaller than the full observed radar message family.
- Another hypothesis raised in Discord was that regular non-MRCC cruise may not be a usable path for openpilot longitudinal because the PCM appears to control throttle directly rather than via the same CAN command family used by MRCC.
- Several Mazda users also repeated the operational safety implication that disabling radar is expected to remove high-speed radar-based AEB (`SBS`), while camera-based low-speed braking (`SCBS`) may still remain. This is useful context, but still needs direct verification on the target vehicle.

## Research Questions

The work was driven by four concrete questions:

1. Which Mazda CAN messages carry the stock MRCC longitudinal request?
2. Are those messages still present when MRCC is only in standby?
3. Which message appears to be the true signed accel/brake request, and which ones are companion or secondary fields?
4. What is the likely minimum message set that must be emulated for openpilot longitudinal to work?

## Tools Added During This Work

Two Mazda-specific offline analysis tools were added to this repo:

- [examples/analyze_mazda_acc_rlog.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/analyze_mazda_acc_rlog.py)
- [examples/compare_mazda_acc_rlogs.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/compare_mazda_acc_rlogs.py)
- [examples/prepare_mazda_longitudinal_templates.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/prepare_mazda_longitudinal_templates.py)
- [examples/extract_mazda_longitudinal_replay.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/extract_mazda_longitudinal_replay.py)
- [examples/run_mazda_longitudinal_replay_with_panda.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/run_mazda_longitudinal_replay_with_panda.py)
- [examples/test_mazda_radar_disable_with_panda.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/test_mazda_radar_disable_with_panda.py)
- [opendbc/car/mazda/longitudinal_experimental.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/opendbc/car/mazda/longitudinal_experimental.py)

### `analyze_mazda_acc_rlog.py`

Purpose:

- summarize one Mazda `rlog.zst`
- rank candidate longitudinal signals by correlation with measured longitudinal acceleration
- print raw payload snapshots for strongest accel and strongest decel moments

This script is CAN-only and uses:

- `ENGINE_DATA.SPEED`
- `BRAKE.VEHICLE_ACC_X`
- `CRZ_CTRL`
- `CRZ_INFO`
- `CRZ_EVENTS`
- `GAS`
- `MORE_GAS`

### `compare_mazda_acc_rlogs.py`

Purpose:

- compare two or more Mazda logs
- print focused signal differences
- print raw byte-mode differences for:
  - `0x21b` / `CRZ_INFO`
  - `0x21c` / `CRZ_CTRL`
  - `0x21f` / `CRZ_EVENTS`
  - `0xfd` / `GAS`
  - `0x167` / `MORE_GAS`

This is the current main tool for preparing implementation-oriented evidence.

## Logs Analyzed

The following logs were analyzed:

- `rlog.zst`
- `rloglead.zst`
- `rlogmainonaccoff.zst`
- `rlognolead.zst`
- `rlogopenroadnoleadmostly.zst`
- `rlogstopngo.zst`
- `rlogstopngo2.zst`

### Scenario Classification

#### `rlogmainonaccoff.zst`

Best `MRCC standby` log.

Key characteristics:

- `CRZ_AVAILABLE=1`
- `CRZ_ACTIVE=0`
- `speed_min=0.00`, `speed_max=21.03 m/s`
- driver gas and brake both present
- lead-related bits occasionally present
- `CRZ_SPEED=0` throughout

Use:

- best log for understanding Mazda `main on, ACC not engaged`
- best log for determining whether standby injection is possible

#### `rlognolead.zst`

Best steady engaged cruise log.

Key characteristics:

- `CRZ_ACTIVE=1` for `5998/5999` samples
- `speed=31.03..31.58 m/s`
- `CRZ_SPEED=31.29 m/s` constant
- no driver gas/brake
- cleaner than the other highway logs

Important caveat:

- despite the filename, `RADAR_HAS_LEAD` is still high for much of the segment, so it is not a pure “no lead” log

Use:

- best engaged-cruise baseline
- best comparison target for standby vs engaged

#### `rloglead.zst`

Best lead-follow log.

Key characteristics:

- `CRZ_ACTIVE=1` for all samples
- `speed=26.41..31.43 m/s`
- larger dynamic accel/decel range
- no driver gas/brake
- `lead=4830/6000`

Use:

- best log for follow-state behavior
- best high-speed lead-follow reference

#### `rlog.zst`

Another good lead-follow log with clear accel/brake activity.

Key characteristics:

- `CRZ_ACTIVE=1` for `5999/6000`
- `speed=18.94..26.76 m/s`
- strong negative and positive longitudinal requests

Use:

- good confirmation log for the main longitudinal command findings

#### `rlogopenroadnoleadmostly.zst`

Engaged highway cruise with moderate dynamics.

Use:

- useful supporting log
- not the strongest primary reference

#### `rlogstopngo.zst`

Stop-and-go log, but contaminated by some driver gas input.

Use:

- secondary stop-go reference only

#### `rlogstopngo2.zst`

Best stop-go log.

Key characteristics:

- `CRZ_ACTIVE=1` for all samples
- `speed=0.00..8.15 m/s`
- `standstill=1915/6000`
- no driver gas/brake
- lead present throughout

Use:

- best stop/hold/resume reference
- best low-speed engaged-state reference

## Mazda Messages of Interest

The main candidate messages identified from the DBC and confirmed in the logs are:

- `0x21b` / `CRZ_INFO`
- `0x21c` / `CRZ_CTRL`
- `0x21f` / `CRZ_EVENTS`
- `0xfd` / `GAS`
- `0x167` / `MORE_GAS`

These appear consistently on `src=0` and as forwarded copies on another source in the logs.

The research used `src=0` as the primary Mazda bus to decode.

## Main Findings

### 1. Mazda longitudinal is already present on CAN as a signed stock request

The logs strongly support that Mazda stock MRCC publishes its longitudinal request on:

- `CRZ_INFO.ACCEL_CMD`
- `CRZ_EVENTS.ACCEL_CMD`
- `CRZ_EVENTS.ACCEL_CMD_LOW_RES`

These signals track measured longitudinal acceleration much better than `GAS.GAS_CMD`.

Observed behavior in engaged logs:

- `CRZ_INFO.ACCEL_CMD` spans positive and negative values
- `CRZ_EVENTS.ACCEL_CMD` spans positive and negative values
- `CRZ_EVENTS.ACCEL_CMD_LOW_RES` spans positive and negative values
- `GAS.GAS_CMD` remains positive-only and mostly looks like a supporting throttle-like request

This is the single biggest technical result: Mazda does not look like a platform where a gas-only interface is sufficient. There is already a signed stock accel/brake command on CAN.

### 2. `CRZ_INFO.ACCEL_CMD` is the strongest high-resolution candidate

Across engaged logs, `CRZ_INFO.ACCEL_CMD` behaves like the high-resolution version of the stock longitudinal request.

Observed numeric relationship:

- `CRZ_INFO.ACCEL_CMD - 8 * CRZ_EVENTS.ACCEL_CMD` has a median around `4`

This is not exact identity, but it is close enough to strongly suggest:

- `CRZ_EVENTS.ACCEL_CMD` is a lower-resolution companion signal
- `CRZ_INFO.ACCEL_CMD` is the more important high-resolution signal

### 3. `GAS.GAS_CMD` is real but secondary

`GAS.GAS_CMD` changes with vehicle behavior, but it is weaker than the signed accel fields.

Typical observed behavior:

- during acceleration it rises toward the upper end of its range
- during braking it falls toward a low baseline
- it never becomes a clearly signed negative brake request

Working interpretation:

- `GAS.GAS_CMD` is likely required as a companion or throttle-support signal
- it does not appear to be the primary full-range accel/brake command

### 4. MRCC standby keeps the message family alive, but not in a directly usable state

This was the main open question going into the batch analysis.

In `rlogmainonaccoff.zst`:

- `CRZ_AVAILABLE=1`
- `CRZ_ACTIVE=0`
- `0x21b`, `0x21c`, `0x21f`, `0xfd`, and `0x167` all remain present

That means the Mazda standby path does not simply stop transmitting.

But the important caveat is:

- `CRZ_INFO.ACCEL_CMD` is `4094` for `5999/6000` samples

This looks like an invalid or inactive sentinel, not a neutral accel request.

Conclusion:

- standby is not enough by itself
- a successful standby-injection path would need to synthesize engaged-style values, not merely tweak one field

### 5. `CRZ_CTRL` is the state-gating message

`0x21c` / `CRZ_CTRL` changes structurally between standby and engaged operation.

Most common standby mode bytes:

- `02 01 0b 00 00 00 00 00`

Most common engaged-cruise mode bytes:

- `0a 01 8b 20 00 00 10 00`

Most common engaged-follow mode bytes:

- `0a 01 8b 40 00 00 10 00`

Observed stop-go modes include:

- `0a 01 8b 20 00 00 10 00`
- `0a 01 8b 40 00 00 10 00`
- `0a 01 8b 60 00 00 10 00`

Important differences:

- standby vs engaged changes bytes `0`, `2`, `3`, and `6`
- engaged-vs-engaged comparisons show byte `3` changes the most across cruise, lead-follow, and stop-go

Working interpretation:

- `0x21c` is not just a passive status frame
- it appears to carry the controller’s active submode
- any emulation attempt must reproduce the engaged `0x21c` state, not only `0x21b`

### 6. Stop-go uses the same message family

`rlogstopngo2.zst` shows that:

- the same `0x21b`, `0x21c`, `0x21f`, and likely `0xfd` family is active in stop-go
- `CRZ_INFO.ACCEL_CMD` reaches around `-1024` at standstill/hold phases

This suggests:

- stop/hold behavior is not implemented via an entirely separate message family
- the longitudinal request likely stays within the same main command path, with submode/state changes on `0x21c`

### 7. The stock Mazda long message family is phase-split at 100 Hz

The current logs show that the longitudinal message family is not transmitted as one 50 Hz burst.

Instead, stock traffic alternates two 10 ms phases:

- phase A:
  - `0xfd` / `GAS`
  - `0x167` / `MORE_GAS`
  - `0x21f` / `CRZ_EVENTS`
- phase B about 10 ms later:
  - `0x21b` / `CRZ_INFO`
  - `0x21c` / `CRZ_CTRL`

Each individual message still runs at about 50 Hz, but the family as a whole is interleaved across a 100 Hz schedule.

This matters for implementation:

- a first replay experiment should preserve this phase split
- sending all five frames as one lump is less faithful than alternating the stock phase pattern
- `0x21f` replay can now be paired with the following `0x21b/0x21c` tick as a single logical 20 ms control step

## Detailed Comparative Findings

### Standby vs Engaged Cruise

Compared:

- `rlogmainonaccoff.zst`
- `rlognolead.zst`

Main differences:

- `CRZ_CTRL.CRZ_ACTIVE`: `0` vs `1`
- `CRZ_CTRL.ACC_ACTIVE_2`: `0` vs `1`
- `CRZ_INFO.ACCEL_CMD`: standby median `4094`, engaged median `-7`
- `CRZ_EVENTS.CRZ_SPEED`: standby `0`, engaged `112.63 kph`
- `CRZ_EVENTS.ACCEL_CMD_LOW_RES`: standby mostly `-8`, engaged mostly `-4`
- `GAS.GAS_CMD`: standby median `44`, engaged median `72`

Most important raw difference:

- `CRZ_CTRL 0x21c`
  - standby mode bytes: `02 01 0b 00 00 00 00 00`
  - engaged mode bytes: `0a 01 8b 20 00 00 10 00`

Interpretation:

- a future injector cannot treat `0x21c` as optional
- the stock controller is clearly in a different state machine branch between standby and engaged

### Engaged Cruise vs Engaged Lead-Follow

Compared:

- `rlognolead.zst`
- `rloglead.zst`

Main differences:

- `CRZ_CTRL.RADAR_LEAD_RELATIVE_DISTANCE` shifts upward in follow
- `CRZ_CTRL.ACC_GAS_MAYBE2` differs strongly between these logs
- `CRZ_INFO.ACCEL_CMD` and `CRZ_EVENTS.ACCEL_CMD` gain much wider range in follow

Most important raw difference:

- `CRZ_CTRL 0x21c`
  - cruise mode bytes: `0a 01 8b 20 00 00 10 00`
  - lead-follow mode bytes: `0a 01 8b 40 00 00 10 00`

Interpretation:

- byte `3` of `0x21c` is very likely part of the follow-state or control submode

### Engaged Lead-Follow vs Stop-Go

Compared:

- `rloglead.zst`
- `rlogstopngo2.zst`

Main differences:

- stop-go has persistent lead presence
- `CRZ_EVENTS.ACCEL_CMD` and `CRZ_INFO.ACCEL_CMD` shift much more negative
- stop-go introduces stronger low-speed submodes

Most important raw difference:

- `CRZ_CTRL 0x21c`
  - follow mode often centers on `...40...`
  - stop-go frequently uses `...20...`, `...40...`, and `...60...`

Interpretation:

- byte `3` likely encodes more than one engaged submode
- stop-go probably uses the same base message family with additional low-speed state encoding

## Current Working Model

The current best-fit model is:

1. The Mazda MRCC stack already publishes a full longitudinal command set on CAN.
2. `CRZ_INFO.ACCEL_CMD` is the best candidate for the main high-resolution accel/brake request.
3. `CRZ_EVENTS.ACCEL_CMD` and `CRZ_EVENTS.ACCEL_CMD_LOW_RES` are companion/echo/low-resolution versions.
4. `GAS.GAS_CMD` is likely a required secondary companion signal.
5. `CRZ_CTRL` is the control/state gate and appears required for valid engaged behavior.
6. Openpilot longitudinal for Mazda will most likely require replacing at least:
   - `0x21b`
   - `0x21c`
   - `0x21f`
   - probably `0xfd`

`0x167` may also be required, but it currently looks less central than the four above.

## What Has Not Been Proven Yet

The following are still open:

### 1. Whether standby injection can work at all

We now know standby traffic exists, but we do not know whether the car will accept engaged-style injected frames while stock radar remains active but ACC is not set.

### 2. Whether radar TX disable via UDS is sufficient

The thread points to the radar ECU as the likely longitudinal owner and proposes UDS TX-disable as the upstreamable route.

This still needs to be proven on actual Mazda hardware:

- whether the correct ECU/address has been identified
- whether TX-disable leaves the rest of the car in a workable state
- whether additional emulation is required to avoid faults

### 3. Which message set is minimally sufficient

We have a strong short list, but not a proven minimum viable transmit set.

Current likely minimum:

- `0x21b`
- `0x21c`
- `0x21f`
- `0xfd`

Possible additional requirement:

- `0x167`

Discord-based refinement:

- historical Mazda VCM behavior suggests that `0x21b` and `0x21c` may be the most essential pair to keep the car logically satisfied
- the newer radar still emits additional messages, so this is only a hypothesis, not proof
- this now justifies a staged minimum-set test order:
  - test `0x21b + 0x21c` first after radar disable
  - then add `0x21f`
  - then add `0xfd`
  - then add `0x167` if still needed

### 4. Message integrity details

The raw logs provide payload behavior, but implementation still requires:

- full bit-level field understanding where the DBC is incomplete
- any counter/checksum semantics if relevant
- required send rate and synchronization behavior

### 5. AEB implications

Disabling radar or replacing its outputs may affect stock AEB and other driver-assistance features.

This is an implementation and safety risk that still needs deliberate handling.

## The Actual Task Remaining

The remaining task is not “decode Mazda longitudinal from scratch.” The main command family is already visible.

The remaining task is:

1. Decide whether to:
   - inject on top of standby, or
   - disable stock radar/controller and replace its traffic
2. Build Mazda message generation for the main engaged-state frames.
3. Add Mazda `init/deinit` logic for any UDS disable path.
4. Add panda safety support for the Mazda longitudinal TX messages.
5. Add Mazda openpilot longitudinal interface/controller logic.
6. Tune and validate on road.

## Recommended Immediate Next Steps

These are the next steps that should happen now.

### Step 1: Build a Mazda message-emulation helper

Create a Mazda-specific experimental helper that can prepare and emit:

- `0x21b` / `CRZ_INFO`
- `0x21c` / `CRZ_CTRL`
- `0x21f` / `CRZ_EVENTS`
- `0xfd` / `GAS`
- optionally `0x167` / `MORE_GAS`

This helper should begin with replay-oriented scaffolding, not final controller logic.

Immediate objective:

- be able to construct “standby-like”
- “engaged cruise-like”
- “engaged follow-like”
- “stop-go-like”

variants of the Mazda longitudinal message set

### Step 2: Treat `CRZ_CTRL` byte-state transitions as a first-class problem

Do not start by mutating only `ACCEL_CMD`.

The current evidence says:

- `CRZ_CTRL` must be part of the sender
- byte `3` appears especially important for submode

The first implementation pass should explicitly support at least:

- standby-like mode
- engaged cruise mode
- engaged follow mode
- stop-go mode

### Step 3: Test the implementation path decision

Two possible paths remain:

#### Path A: Preferred

Disable the stock longitudinal owner, likely radar/controller TX, and replace the Mazda longitudinal traffic cleanly.

Why preferred:

- closer to known upstreamable architecture
- avoids fighting a live stock controller
- cleaner long-term design if it works

#### Path B: Fallback experiment

Keep MRCC in standby and inject engaged-style messages over the standby state.

Why only fallback:

- standby traffic is alive, but `CRZ_INFO.ACCEL_CMD` is invalid in standby
- likely harder to make robust
- higher chance of stock-controller interference

### Step 4: Prepare Mazda safety changes

Before any actuator testing on the car, panda safety will need:

- Mazda longitudinal TX whitelist expansion
- accel/brake command bounds
- any gating tied to controls allowed, gas pressed, brake pressed, standstill, or cruise state

### Step 5: Add implementation notes for low-speed behavior

Stop-go behavior should not be left as a later afterthought. The logs already show:

- distinct low-speed engaged states
- `CRZ_INFO.ACCEL_CMD` behavior near hold
- additional `CRZ_CTRL` submode variation

Low-speed mode must be part of the initial design, not bolted on afterward.

## Immediate Execution Plan

The next concrete engineering tasks should be:

1. Add a Mazda experimental packer/sender module for:
   - `CRZ_INFO`
   - `CRZ_CTRL`
   - `CRZ_EVENTS`
   - `GAS`
   - optional `MORE_GAS`
2. Seed `CRZ_CTRL` mode templates from the observed dominant payloads:
   - standby: `02 01 0b 00 00 00 00 00`
   - engaged cruise: `0a 01 8b 20 00 00 10 00`
   - engaged follow: `0a 01 8b 40 00 00 10 00`
   - stop-go variants: include `...20...`, `...40...`, `...60...`
3. Build a replay or mutation harness that can adjust:
   - `CRZ_INFO.ACCEL_CMD`
   - `CRZ_EVENTS.ACCEL_CMD`
   - `CRZ_EVENTS.ACCEL_CMD_LOW_RES`
   - `GAS.GAS_CMD`
4. Test whether the stock car accepts these messages in standby.
5. If not, move to radar/controller TX-disable experiments and full message replacement.

Based on later Discord discussion, the immediate test order should now be:

1. Prove radar disable behavior on the actual car at `0x764` and observe which Mazda longitudinal frames disappear.
2. If radar disable works, try the smallest plausible replacement set first:
   - `0x21b`
   - `0x21c`
3. Only add:
   - `0x21f`
   - `0xfd`
   - `0x167`
   if the smaller set is not sufficient.
4. Treat regular non-MRCC cruise as a lower-priority fallback idea until there is direct evidence that the PCM accepts equivalent speed/accel control over CAN in that mode.

## Artifacts Created In This Repo

The following local tools now exist to support continued work:

- [examples/analyze_mazda_acc_rlog.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/analyze_mazda_acc_rlog.py)
- [examples/compare_mazda_acc_rlogs.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/compare_mazda_acc_rlogs.py)
- [examples/prepare_mazda_longitudinal_templates.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/prepare_mazda_longitudinal_templates.py)
- [examples/extract_mazda_longitudinal_replay.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/extract_mazda_longitudinal_replay.py)
- [examples/run_mazda_longitudinal_replay_with_panda.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/run_mazda_longitudinal_replay_with_panda.py)
- [examples/test_mazda_radar_disable_with_panda.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/test_mazda_radar_disable_with_panda.py)
- [opendbc/car/mazda/longitudinal_experimental.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/opendbc/car/mazda/longitudinal_experimental.py)

Suggested usage:

```bash
cd /path/to/op-mazda-testkit
PYTHONPATH=/path/to/op-mazda-testkit \
python examples/analyze_mazda_acc_rlog.py /path/to/rloglead.zst
```

```bash
cd /path/to/op-mazda-testkit
PYTHONPATH=/path/to/op-mazda-testkit \
python examples/compare_mazda_acc_rlogs.py /path/to/rlogmainonaccoff.zst /path/to/rlognolead.zst
```

## Bottom Line

What we know now:

- Mazda MRCC longitudinal commands are already visible on CAN.
- The main command family is `0x21b`, `0x21c`, `0x21f`, and likely `0xfd`.
- `CRZ_INFO.ACCEL_CMD` is the strongest main signed longitudinal request candidate.
- `CRZ_CTRL` appears to be the engaged-state gate and submode carrier.
- Standby keeps the message family alive, but not in a directly usable neutral-control state.

What remains:

- implement the Mazda message-prep and sender path
- test standby injection versus stock-controller disable
- add panda safety and full integration if the message set can be controlled reliably

The work is now past “signal hunting” and into “message emulation and controller replacement.”
