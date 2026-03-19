#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path
from subprocess import CalledProcessError, check_output

from opendbc.car import uds
from opendbc.car.structs import CarParams
from panda import Panda


TARGET_ADDRS: tuple[int, ...] = (0x21B, 0x21C, 0x21F, 0x0FD, 0x167, 0x361, 0x362, 0x363, 0x364, 0x365, 0x366)
DEFAULT_SAMPLE_ADDRS: tuple[int, ...] = TARGET_ADDRS + (0x499,)
SESSION_BY_NAME: dict[str, uds.SESSION_TYPE] = {
  "default": uds.SESSION_TYPE.DEFAULT,
  "programming": uds.SESSION_TYPE.PROGRAMMING,
  "extended": uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC,
  "safety": uds.SESSION_TYPE.SAFETY_SYSTEM_DIAGNOSTIC,
}


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Probe Mazda radar disable at 0x764 and observe which CAN messages disappear.")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus for UDS/radar")
  parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x764, help="Radar UDS address")
  parser.add_argument("--can-speed-kbps", type=int, default=500, help="CAN speed in kbps")
  parser.add_argument("--observe-seconds", type=float,
                      help="Legacy alias that sets both --before-seconds and --during-seconds to the same value")
  parser.add_argument("--before-seconds", type=float, default=3.0,
                      help="Seconds to count target frames before entering the diagnostic session")
  parser.add_argument("--settle-seconds", type=float, default=0.0,
                      help="Optional quiet period after the baseline count and before entering the diagnostic session")
  parser.add_argument("--during-seconds", type=float, default=3.0,
                      help="Seconds to count target frames while holding tester-present active")
  parser.add_argument("--after-seconds", type=float, default=0.0,
                      help="Optional post-exit observation window after releasing the radar session")
  parser.add_argument("--progress-interval", type=float, default=0.0,
                      help="Print a progress line every N seconds during the active hold; 0 disables")
  parser.add_argument("--disable-rx-too", action="store_true", help="Use DISABLE_RX_DISABLE_TX (0x28 03 01) instead of ENABLE_RX_DISABLE_TX (0x28 01 01)")
  parser.add_argument("--session", choices=tuple(SESSION_BY_NAME), default="extended", help="Diagnostic session to enter before communication-control")
  parser.add_argument("--try-all-sessions", action="store_true", help="Try default, extended, and safety-system sessions until communication-control is accepted")
  parser.add_argument("--session-only", action="store_true", help="Do not send communication-control. Hold the chosen diagnostic session and observe traffic while tester-present is active.")
  parser.add_argument("--exit-default-session", action="store_true",
                      help="After the hold, explicitly request UDS default session on the radar before returning Panda to silent mode")
  parser.add_argument("--all-addrs", action="store_true",
                      help="Count every CAN address seen on the selected bus and report dropout candidates, not just the current Mazda shortlist")
  parser.add_argument("--drop-ratio-threshold", type=float, default=0.5,
                      help="In --all-addrs mode, report addresses whose active-session rate is at or below this fraction of the baseline rate")
  parser.add_argument("--min-before-count", type=int, default=3,
                      help="In --all-addrs mode, require at least this many baseline frames before reporting an address as dropped")
  parser.add_argument("--max-drop-lines", type=int, default=80,
                      help="In --all-addrs mode, cap the number of reported dropout addresses")
  parser.add_argument("--sample-output", type=Path,
                      help="Optional JSON artifact path for timestamped raw-frame samples from each phase")
  parser.add_argument("--sample-addrs", type=str,
                      help="Comma-separated CAN addresses to sample into --sample-output. Defaults to 0x21b,0x21c,0x21f,0xfd,0x167,0x361-0x366,0x499")
  parser.add_argument("--sample-limit", type=int, default=40,
                      help="Maximum sampled frames to retain per address per phase in --sample-output")
  parser.add_argument("--allow-aeb-risk", action="store_true", help="Required. Radar disable may remove high-speed AEB.")
  parser.add_argument("--output", type=Path, help="Optional text output file")
  return parser.parse_args()


def ensure_pandad_stopped() -> None:
  try:
    check_output(["pidof", "pandad"])
    print("pandad is running, please stop openpilot before probing radar disable.")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1:
      raise


def parse_addr_csv(value: str | None) -> tuple[int, ...]:
  if value is None or value.strip() == "":
    return DEFAULT_SAMPLE_ADDRS
  return tuple(int(part.strip(), 0) for part in value.split(",") if part.strip())


def finalize_phase_samples(samples: dict[int, list[dict[str, object]]]) -> dict[str, list[dict[str, object]]]:
  return {hex(addr): entries for addr, entries in sorted(samples.items())}


def count_addrs(panda: Panda,
                bus: int,
                seconds: float,
                *,
                all_addrs: bool = False,
                sample_addrs: set[int] | None = None,
                sample_limit: int = 0) -> tuple[Counter[int], dict[int, list[dict[str, object]]]]:
  deadline = time.monotonic() + seconds
  start = time.monotonic()
  counts: Counter[int] = Counter()
  samples: dict[int, list[dict[str, object]]] = defaultdict(list)
  panda.can_clear(0xFFFF)
  while time.monotonic() < deadline:
    now = time.monotonic()
    for addr, dat, src in panda.can_recv():
      if src == bus and (all_addrs or addr in TARGET_ADDRS):
        counts[addr] += 1
      if src == bus and sample_addrs and addr in sample_addrs and len(samples[addr]) < sample_limit:
        samples[addr].append({"t": round(now - start, 6), "data": dat.hex()})
    time.sleep(0.001)
  return counts, samples


def count_addrs_with_tester_present(panda: Panda, bus: int, addr: int, seconds: float,
                                    tester_present_interval: float = 0.5,
                                    progress_interval: float = 0.0,
                                    *,
                                    all_addrs: bool = False,
                                    sample_addrs: set[int] | None = None,
                                    sample_limit: int = 0) -> tuple[Counter[int], dict[int, list[dict[str, object]]]]:
  deadline = time.monotonic() + seconds
  start = time.monotonic()
  next_tester_present = time.monotonic()
  next_progress = time.monotonic() + progress_interval if progress_interval > 0 else float("inf")
  counts: Counter[int] = Counter()
  samples: dict[int, list[dict[str, object]]] = defaultdict(list)
  panda.can_clear(0xFFFF)
  while time.monotonic() < deadline:
    now = time.monotonic()
    if now >= next_tester_present:
      send_tester_present_suppress_response(panda, bus, addr)
      next_tester_present = now + tester_present_interval
    if now >= next_progress:
      elapsed = seconds - max(deadline - now, 0.0)
      track_total = sum(counts[target_addr] for target_addr in TARGET_ADDRS if 0x361 <= target_addr <= 0x366)
      print(
        f"  hold progress: t={elapsed:.1f}s/{seconds:.1f}s "
        f"0x21b={counts[0x21B]} 0x21c={counts[0x21C]} 0x21f={counts[0x21F]} "
        f"0xfd={counts[0x0FD]} 0x167={counts[0x167]} radar_tracks={track_total}"
      )
      next_progress = now + progress_interval

    for rx_addr, dat, src in panda.can_recv():
      if src == bus and (all_addrs or rx_addr in TARGET_ADDRS):
        counts[rx_addr] += 1
      if src == bus and sample_addrs and rx_addr in sample_addrs and len(samples[rx_addr]) < sample_limit:
        samples[rx_addr].append({"t": round(now - start, 6), "data": dat.hex()})
    time.sleep(0.001)
  return counts, samples


def format_counts(title: str, counts: Counter[int], seconds: float, *, all_addrs: bool = False) -> str:
  lines = [title]
  addrs = sorted(counts) if all_addrs else list(TARGET_ADDRS)
  if all_addrs:
    lines.append(f"  unique_addrs={len(addrs)}")
  for addr in addrs:
    hz = counts[addr] / seconds
    lines.append(f"  {hex(addr)} count={counts[addr]} hz={hz:.2f}")
  return "\n".join(lines)


def format_dropout_candidates(title: str,
                              before_counts: Counter[int],
                              during_counts: Counter[int],
                              before_seconds: float,
                              during_seconds: float,
                              *,
                              min_before_count: int,
                              drop_ratio_threshold: float,
                              max_lines: int) -> str:
  rows: list[tuple[float, float, int, int, int]] = []
  for addr in set(before_counts) | set(during_counts):
    before = before_counts[addr]
    during = during_counts[addr]
    if before < min_before_count:
      continue
    before_hz = before / before_seconds
    during_hz = during / during_seconds
    ratio = (during_hz / before_hz) if before_hz > 0 else float("inf")
    if ratio <= drop_ratio_threshold:
      rows.append((ratio, -before_hz, addr, before, during))

  rows.sort()
  lines = [
    title,
    f"  filters: min_before_count={min_before_count} drop_ratio_threshold={drop_ratio_threshold:.2f} max_lines={max_lines}",
  ]
  if not rows:
    lines.append("  no dropout candidates matched the current filters")
    return "\n".join(lines)

  for ratio, neg_before_hz, addr, before, during in rows[:max_lines]:
    before_hz = -neg_before_hz
    during_hz = during / during_seconds
    lines.append(
      f"  {hex(addr)} before={before} ({before_hz:.2f}Hz) "
      f"during={during} ({during_hz:.2f}Hz) ratio={ratio:.2f}"
    )
  if len(rows) > max_lines:
    lines.append(f"  ... truncated {len(rows) - max_lines} additional addresses")
  return "\n".join(lines)


def append_target_deltas(report: list[str],
                         title: str,
                         before_counts: Counter[int],
                         current_counts: Counter[int],
                         before_seconds: float,
                         current_seconds: float) -> None:
  report.append(title)
  for addr in TARGET_ADDRS:
    before = before_counts[addr]
    current = current_counts[addr]
    before_hz = before / before_seconds if before_seconds > 0 else 0.0
    current_hz = current / current_seconds if current_seconds > 0 else 0.0
    ratio = (current_hz / before_hz) if before_hz > 0 else 0.0
    report.append(
      f"  {hex(addr)} before={before} ({before_hz:.2f}Hz) "
      f"current={current} ({current_hz:.2f}Hz) ratio={ratio:.2f}"
    )


def send_tester_present_suppress_response(panda: Panda, bus: int, addr: int) -> None:
  panda.can_send(addr, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus)


def session_attempt_order(selected_session_name: str, try_all_sessions: bool) -> list[tuple[str, uds.SESSION_TYPE]]:
  if try_all_sessions:
    ordered = [selected_session_name, *[name for name in SESSION_BY_NAME if name != selected_session_name]]
  else:
    ordered = [selected_session_name]
  return [(name, SESSION_BY_NAME[name]) for name in ordered]


def main() -> None:
  args = parse_args()
  if not args.allow_aeb_risk:
    print("--allow-aeb-risk is required. Disabling the radar may disable high-speed AEB.")
    sys.exit(1)
  if args.observe_seconds is not None:
    args.before_seconds = args.observe_seconds
    args.during_seconds = args.observe_seconds

  ensure_pandad_stopped()
  sample_addrs = set(parse_addr_csv(args.sample_addrs)) if args.sample_output is not None else set()

  panda = Panda()
  panda.set_can_speed_kbps(args.bus, args.can_speed_kbps)
  panda.set_safety_mode(CarParams.SafetyModel.elm327)

  control_type = uds.CONTROL_TYPE.DISABLE_RX_DISABLE_TX if args.disable_rx_too else uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX
  report: list[str] = []
  disable_succeeded = False
  session_entered = False
  phase_samples: dict[str, dict[str, list[dict[str, object]]]] = {}

  try:
    before_counts, before_samples = count_addrs(
      panda,
      args.bus,
      args.before_seconds,
      all_addrs=args.all_addrs,
      sample_addrs=sample_addrs,
      sample_limit=args.sample_limit,
    )
    if sample_addrs:
      phase_samples["before"] = finalize_phase_samples(before_samples)
    report.append(format_counts("Before disable:", before_counts, args.before_seconds, all_addrs=args.all_addrs))
    if args.settle_seconds > 0:
      report.append(f"\nSettling for {args.settle_seconds:.1f}s before entering the diagnostic session.")
      time.sleep(args.settle_seconds)

    client = uds.UdsClient(panda, args.addr, bus=args.bus)
    report.append(f"\nUDS session start: addr={hex(args.addr)} bus={args.bus} control_type={control_type.name}")

    observed_counts: Counter[int] | None = None
    for session_name, session_type in session_attempt_order(args.session, args.try_all_sessions):
      report.append(f"Trying session={session_name} ({session_type.name})")
      try:
        client.diagnostic_session_control(session_type)
        report.append(f"  diagnostic_session_control ok: {session_name}")
        send_tester_present_suppress_response(panda, args.bus, args.addr)
        report.append(f"  tester_present(0x3e80) sent: {session_name}")
        session_entered = True
      except Exception as e:
        report.append(f"  diagnostic_session_control failed: {e!r}")
        continue

      if not args.session_only:
        try:
          client.communication_control(control_type, uds.MESSAGE_TYPE.NORMAL)
          report.append(f"  communication_control ok: {session_name}")
          disable_succeeded = True
        except uds.NegativeResponseError as e:
          report.append(f"  communication_control negative response: {e} service_id={hex(e.service_id)} error_code={hex(e.error_code)}")
        except Exception as e:
          report.append(f"  communication_control failed: {e!r}")

      if disable_succeeded or args.session_only:
        observed_counts, observed_samples = count_addrs_with_tester_present(
          panda, args.bus, args.addr, args.during_seconds,
          progress_interval=args.progress_interval,
          all_addrs=args.all_addrs,
          sample_addrs=sample_addrs,
          sample_limit=args.sample_limit,
        )
        if sample_addrs:
          phase_samples["active"] = finalize_phase_samples(observed_samples)
        report.append("")
        title = "During active session:" if args.session_only and not disable_succeeded else "During disable hold:"
        report.append(format_counts(title, observed_counts, args.during_seconds, all_addrs=args.all_addrs))
        report.append("")
        append_target_deltas(
          report,
          "Target message deltas (baseline -> active):",
          before_counts,
          observed_counts,
          args.before_seconds,
          args.during_seconds,
        )
        if args.all_addrs:
          report.append("")
          report.append(format_dropout_candidates(
            "All-address dropout candidates (baseline -> active):",
            before_counts,
            observed_counts,
            args.before_seconds,
            args.during_seconds,
            min_before_count=args.min_before_count,
            drop_ratio_threshold=args.drop_ratio_threshold,
            max_lines=args.max_drop_lines,
        ))
        break

    if not (disable_succeeded or args.session_only):
      after_counts, after_samples = count_addrs(
        panda,
        args.bus,
        args.before_seconds,
        all_addrs=args.all_addrs,
        sample_addrs=sample_addrs,
        sample_limit=args.sample_limit,
      )
      if sample_addrs:
        phase_samples["after"] = finalize_phase_samples(after_samples)
      report.append("")
      report.append(format_counts("After disable:", after_counts, args.before_seconds, all_addrs=args.all_addrs))

      report.append("")
      append_target_deltas(
        report,
        "Target message deltas (baseline -> after):",
        before_counts,
        after_counts,
        args.before_seconds,
        args.before_seconds,
      )
      if args.all_addrs:
        report.append("")
        report.append(format_dropout_candidates(
          "All-address dropout candidates (baseline -> after):",
          before_counts,
          after_counts,
          args.before_seconds,
          args.before_seconds,
          min_before_count=args.min_before_count,
          drop_ratio_threshold=args.drop_ratio_threshold,
          max_lines=args.max_drop_lines,
        ))

    if not disable_succeeded and not args.session_only:
      report.append("\nNo tested diagnostic session accepted COMMUNICATION_CONTROL.")
    elif args.session_only and session_entered:
      report.append("\nSession-only probe completed.")

    if args.exit_default_session and session_entered:
      try:
        client.diagnostic_session_control(uds.SESSION_TYPE.DEFAULT)
        report.append("Radar default-session request accepted.")
      except Exception as e:
        report.append(f"Radar default-session request failed: {e!r}")

    if args.after_seconds > 0:
      after_exit_counts, after_exit_samples = count_addrs(
        panda,
        args.bus,
        args.after_seconds,
        all_addrs=args.all_addrs,
        sample_addrs=sample_addrs,
        sample_limit=args.sample_limit,
      )
      if sample_addrs:
        phase_samples["post_exit"] = finalize_phase_samples(after_exit_samples)
      report.append("")
      report.append(format_counts("After exit:", after_exit_counts, args.after_seconds, all_addrs=args.all_addrs))
      report.append("")
      append_target_deltas(
        report,
        "Target message deltas (baseline -> post-exit):",
        before_counts,
        after_exit_counts,
        args.before_seconds,
        args.after_seconds,
      )
      if args.all_addrs:
        report.append("")
        report.append(format_dropout_candidates(
          "All-address dropout candidates (baseline -> post-exit):",
          before_counts,
          after_exit_counts,
          args.before_seconds,
          args.after_seconds,
          min_before_count=args.min_before_count,
          drop_ratio_threshold=args.drop_ratio_threshold,
          max_lines=args.max_drop_lines,
        ))

  finally:
    panda.set_safety_mode(CarParams.SafetyModel.silent)
    report.append("\nPanda returned to silent safety mode.")

  output = "\n".join(report)
  print(output)
  if args.output is not None:
    args.output.write_text(output + "\n")
    print(f"\nWrote report to {args.output}")
  if args.sample_output is not None:
    sample_payload = {
      "bus": args.bus,
      "radar_addr": hex(args.addr),
      "session": args.session,
      "session_only": args.session_only,
      "before_seconds": args.before_seconds,
      "settle_seconds": args.settle_seconds,
      "during_seconds": args.during_seconds,
      "after_seconds": args.after_seconds,
      "sample_limit": args.sample_limit,
      "sample_addrs": [hex(addr) for addr in sorted(sample_addrs)],
      "phases": phase_samples,
    }
    args.sample_output.write_text(json.dumps(sample_payload, indent=2) + "\n")
    print(f"Wrote phase samples JSON to {args.sample_output}")


if __name__ == "__main__":
  main()
