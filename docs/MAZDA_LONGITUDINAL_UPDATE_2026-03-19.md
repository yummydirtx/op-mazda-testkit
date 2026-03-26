# Mazda Longitudinal Update - 2026-03-19

This document summarizes what was learned after the previous Mazda longitudinal update on March 13, 2026.

The most important change is that the project is no longer blocked on two foundational questions:

- can radar suppression be held on-car for extended drive testing?
- can the suppressed/replaced path actually command longitudinal actuation?

Both now have direct on-car evidence behind them.

## Summary

What is now established:

- the `0x764` suppression path can be held long enough to support extended multi-minute drive testing
- replacing `0x21b + 0x21c` while the radar is suppressed can keep the active window alive over those longer runs
- `CRZ_INFO.ACCEL_CMD` has real longitudinal authority on this car
- positive overrides accelerate the car
- negative overrides decelerate / brake the car
- zero behaves like a hold command rather than chasing set speed

That means the project is no longer blocked at:

- "can we hold suppression long enough to matter?"
- "does the replaced path actually move the car?"

The main blocker is now different:

- preventing the camera / SBS / DSC / radar fault chain that starts during the active suppression window

## New Proven Results

### 1. Extended active suppression is practical

Longer on-car runs showed that the working suppression method is not limited to a tiny parked proof.

During active `programming` session on `0x764`, with replacement of `0x21b + 0x21c`:

- `0x21b` stays suppressed from the stock radar owner
- `0x21c` stays suppressed from the stock radar owner
- radar tracks stay suppressed
- stock `0x21f`, `0xfd`, and `0x167` remain alive

This was sustained over real multi-minute on-road testing, not just short bench-style windows.

That is a meaningful milestone because it shows the suppression path is operationally usable for real experiments, even though it is still not clean from a fault-management perspective.

### 2. Real longitudinal authority is now proven

On-car testing now shows that `CRZ_INFO.ACCEL_CMD` is not just correlated metadata. It is a real actuator input on this platform.

Observed behavior:

- positive `ACCEL_CMD` overrides produce acceleration
- negative `ACCEL_CMD` overrides produce deceleration / braking
- zero `ACCEL_CMD` behaves like a speed-hold request

This matches the offline CAN analysis and upgrades it from a strong hypothesis to an on-car proven result.

The practical implication is:

- the Mazda long path is not blocked on "finding an actuation message"
- `0x21b` already gives usable longitudinal authority

### 3. Mixed ownership is enough for basic actuation

The currently working state is still mixed ownership:

- injected `0x21b`
- injected `0x21c`
- stock `0x21f`
- stock `0xfd`
- stock `0x167`

That mixed-owner state is already sufficient to produce real positive and negative longitudinal response.

This matters because it means the project does not need full immediate ownership of every stock longitudinal-related message just to prove the path.

### 4. The warnings are still an active-window problem, not only an exit problem

Further parked and short-run testing showed that the key warnings can begin during the active suppression window itself, before the radar is handed back.

Examples include:

- front camera / ADAS malfunction
- smart brake / SCBS-related malfunction warnings

The loud MRCC-disabling escalation can still be sensitive to handoff and recovery timing, but the deeper issue is earlier:

- other modules are already unhappy while the radar is being suppressed

So the main remaining question is no longer:

- "can we hand back cleanly?"

It is now:

- "what exact cross-module validity condition is being violated during the active window?"

## Additional New Findings

### Whole-bus diff did not reveal a large hidden set of missing bus-0 frames

Whole-bus before/during comparisons on bus `0` confirmed the previously known suppressed set and exposed one additional radar-owned message:

- `0x499`

That is useful, but it did not change the overall outcome by itself.

The important interpretation is:

- the remaining fault path probably is not explained by a large number of unknown disappearing bus-0 messages
- the remaining issue is more likely message validity, coherence, rolling state, or another cross-module expectation

### Radar replay became more realistic but did not solve the fault chain by itself

The radar replay path was improved from a frozen single burst to a captured multi-snapshot pre-session sequence.

That was the right direction technically, because the radar-track family clearly contains rolling state across samples.

But even with the more realistic replay sequence, the broader warning behavior still did not become clean.

This further supports the idea that the remaining blocker is not "one missing obvious frame" but "the total active-window state is still not valid enough for the rest of the vehicle."

## Updated Current Interpretation

The best current model is now:

1. `0x764` `programming` session remains the working suppression path.
2. `0x21b` is the real high-resolution longitudinal command.
3. `0x21c` is required state / mode gating.
4. Full immediate ownership of `0x21f`, `0xfd`, and `0x167` is not required to prove basic actuation.
5. The remaining blocker to upstream-quality behavior is not raw actuation.
6. The remaining blocker is keeping the camera / SBS / DSC / radar stack logically happy while suppression is active.

## What Is Now Proven Versus Still Open

Proven now:

- sustained on-car suppression over extended multi-minute testing
- real positive acceleration authority
- real negative deceleration / brake authority
- zero-command speed hold behavior
- mixed-owner operation is enough for real longitudinal response

Still open:

- how to prevent camera / SBS / DSC faults during the active window
- how to restore a truly clean post-test state without re-arming faults
- whether a cleaner full-standby replacement state is needed for `0x21b + 0x21c`
- whether any remaining perception-side state must be emulated more faithfully

## Bottom Line

Since March 13, 2026, the project advanced in a major way:

- radar suppression is no longer just a short-window curiosity
- longitudinal actuation is no longer hypothetical

The path now has enough evidence to say:

- Mazda longitudinal on this platform is possible through the current software-only suppression path

The unresolved work is narrower and clearer than before:

- make the active suppression window valid enough that the rest of the car does not fault while it is happening
