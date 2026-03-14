# Mazda Longitudinal Update - 2026-03-13

This document summarizes what was explored and learned after the previous Mazda longitudinal write-up.

The previous docs established the offline CAN findings and the basic implementation hypothesis. This update covers the newer on-car Panda testing, the current replacement runner behavior, the fault-management work, and the remaining blockers.

## Summary

The most important new result is that Mazda radar suppression is partially achievable on the `2022 CX-5`, but only through `programming` session behavior, not through `COMMUNICATION_CONTROL`.

What is now established:

- `0x764` on bus `0` is a real front-radar-related diagnostic endpoint.
- `COMMUNICATION_CONTROL (0x28)` at `0x764` is still not supported on this car.
- Entering and holding `programming` session on `0x764` suppresses:
  - `0x21b` / `CRZ_INFO`
  - `0x21c` / `CRZ_CTRL`
  - radar tracks `0x361` to `0x366`
- During that same active session, stock traffic continues for:
  - `0x21f` / `CRZ_EVENTS`
  - `0xfd` / `GAS`
  - `0x167` / `MORE_GAS`
- Replacing `0x21b + 0x21c` is enough to keep the car temporarily calm during the active suppression window.
- Automating Mazda `SET-` button presses from the runner works. The car latches a `30.00 kph` set speed in the low-speed tests.
- Actual longitudinal actuation is still not proven. Even with `0x21f` replay and approximate `0x21f` patching, the observed low-speed runs still behaved like ordinary creep.

So the current state is:

- stock owner partial suppression: yes
- replacement of the most important suppressed frames: yes
- engagement/set-speed automation: yes
- clean actuation proof: no

## New On-Car UDS Findings

### `0x764` programming session suppresses the expected frames

The most useful active-session probe was the `programming` session-only test on `0x764`.

Observed before suppression:

- `0x21b` at `50 Hz`
- `0x21c` at `50 Hz`
- `0x21f` at `50 Hz`
- `0xfd` at `50 Hz`
- `0x167` at `50 Hz`
- `0x361` to `0x366` at `10 Hz`

Observed during active `programming` session:

- `0x21b`: effectively gone
- `0x21c`: effectively gone
- `0x361` to `0x366`: fully gone
- `0x21f`: still present
- `0xfd`: still present
- `0x167`: still present

This was the first strong evidence that the Mazda radar path can be partially displaced without an inline hardware interceptor.

### `COMMUNICATION_CONTROL` remains unsupported

`COMMUNICATION_CONTROL (0x28)` was retried across multiple sessions, including `programming`.

Result:

- rejected with NRC `0x11` (`service not supported`)

That means the working suppression path is currently:

- enter `programming`
- keep the session alive with tester-present

not:

- enter session
- disable TX with `0x28`

## New Replacement Runner Behavior

The current main test runner is:

- [run_mazda_crz_replacement_with_panda.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/run_mazda_crz_replacement_with_panda.py)

The Mazda helper that backs it is:

- [longitudinal_experimental.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/opendbc/car/mazda/longitudinal_experimental.py)

Since the last documentation update, the runner gained several practical features:

- local tee logging to a file so data survives SSH dropouts
- exit handoff timing instead of a hard stop
- optional replay of the current parked radar burst
- live decoded RX and TX status printing
- optional `0x21f` replacement at the correct phase
- optional approximate `0x21f` patching
- optional Mazda cruise-button pulses using `CRZ_BTNS`

The most useful additions in practice were:

- persistent local logs
- automatic `SET-` / `RESUME` pulses
- richer status output during the run

## What the Active Replacement Tests Show

### `0x21b + 0x21c` replacement is accepted enough to matter

Replacing only `0x21b + 0x21c` while holding `0x764` in `programming` session did all of the following:

- suppressed the loud front-radar warning during the active run
- allowed a parked test
- allowed a forward creep test in `Drive`

This is important because it means the replacement path is not being immediately rejected.

### Quiet camera/SBS warnings still appear

Even while `0x21b + 0x21c` replacement is active, the following warnings can appear:

- `Front Camera Sensor System Malfunction`
- `Smart Brake System Malfunction`
- `Forward Smart City Brake Support Malfunction`

This shows that the replacement is incomplete from the broader vehicle-systems point of view even when it is sufficient to keep the primary radar fault quiet during the active window.

### Replaying parked radar tracks did not help

The runner was extended to capture and replay the current parked radar burst from `0x361` to `0x366` while the radar was suppressed.

That change made no noticeable difference to the warnings.

The implication is that the missing visible radar-track burst is not the only remaining dependency upsetting the other modules.

## DTC Management Findings

The fault behavior after these tests was real and repeatable, not just cluster cosmetics.

The most relevant modules from the earlier readouts were:

- `0x764` / front radar
- `0x706` / camera-side ADAS
- `0x760` / DSC/ABS

The persistent low-speed ACC degradation was most consistent with the downstream `0x760` and `0x706` faults, not only the radar module itself.

To make cleanup easier, a Mazda-specific DTC helper was added:

- [clear_mazda_warnings.py](https://github.com/yummydirtx/op-mazda-testkit/blob/main/examples/clear_mazda_warnings.py)

It does three things in one pass:

- reads DTCs from the common affected Mazda ECUs
- clears them in a fixed order
- re-reads them and writes the whole session to a log

This does not guarantee the warnings will stay gone. If the underlying condition still exists, the ECUs will relatch the faults. But it removes the need to manually chain multiple generic UDS tools after every test run.

## Engagement Automation Findings

The runner now supports Mazda cruise-button pulses after suppression starts.

The most important low-speed test result here was:

- `SET-` pulses were accepted
- `CRZ_SPEED` latched from `0.00` to `30.00 kph`

That is the Mazda minimum set speed. So the script is now able to do a real part of the engagement workflow itself.

This matters because earlier testing depended on trying to pre-engage or manually juggle stock MRCC state. The script can now at least request the set-speed latch from inside the suppression window.

## What the Low-Speed Tests Did and Did Not Prove

The low-speed runs were still useful, but they need to be interpreted correctly.

What they did prove:

- the script-side button automation works
- the set-speed latch can be driven from the script
- `0x21b + 0x21c` replacement can keep the car stable enough to run during suppression
- `0x21f` replay and approximate `0x21f` patching do not immediately cause obvious actuation at low speed

What they did not prove:

- clean longitudinal actuation
- clean ownership of the full MRCC long path
- proper above-threshold cruise behavior
- proper stop-and-go behavior

In other words, the low-speed lot tests are now good for state-machine and warning work, but still weak for proving actual longitudinal authority.

## Current Best Interpretation

The current evidence points to this picture:

1. `0x21b + 0x21c` are necessary and accepted enough to hold part of the path together.
2. `0x21f` is still a major candidate gate for real actuation.
3. The fact that stock `0x21f` remains alive while the radar is in `programming` session is probably important.
4. Replaying or approximately patching `0x21f` from the runner is not yet enough to prove actuation at low speed.
5. Some additional owner-state, watchdog, or cross-module expectation still exists beyond the currently replaced frames.

This means the project has moved from:

- "can we identify the messages?"

to:

- "what exact remaining state or ownership condition still prevents real actuation?"

That is meaningful progress.

## New Practical Tooling Outcome

The repo now has a much more usable on-device workflow than it had before:

- persistent local run logs
- scripted Mazda `SET-` / `RESUME` pulses
- a Mazda-specific DTC cleanup tool
- enough active-session radar suppression to perform repeatable controlled tests

This is a real operational improvement even though actuation is still unresolved.

## Current Blockers

The main remaining blockers are:

- actual longitudinal actuation still unproven
- stock `0x21f` still present during the suppression method that currently works
- incomplete understanding of which remaining state causes the camera/SBS/DSC faults
- lack of a clean safe venue for repeated above-threshold testing

The test-environment constraint is not trivial. Many of the remaining questions are best answered above the normal minimum Mazda cruise speed, and those tests should not be improvised on normal roads with this setup.

## Recommended Next Steps

The next useful work items are:

1. Keep using low-speed lots for state and warning experiments only.
2. Use the new local logging and Mazda warning reset script after each run.
3. When a safe venue exists, repeat the engage-button flow above the minimum Mazda cruise speed.
4. Compare:
   - `0x21b + 0x21c` only
   - `0x21b + 0x21c + 0x21f` replay
   - `0x21b + 0x21c + 0x21f` approximate patching
5. If actuation still does not appear, focus next on proving whether stock `0x21f` is the real ownership gate or whether another state message is still missing.

## Bottom Line

Since the previous documentation update, the project gained three major things:

- a real partial suppression path on the car
- a real partial replacement path on the car
- much better on-device tooling for repeatable tests and cleanup

That is substantial progress.

The missing piece is no longer message discovery. The missing piece is proving and then stabilizing real actuation under the partially suppressed Mazda radar/control state.
