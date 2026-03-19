#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path
from subprocess import CalledProcessError, check_output

from opendbc.car import uds
from opendbc.car.structs import CarParams
from panda import Panda


TARGET_ADDRS: tuple[int, ...] = (0x21B, 0x21C, 0x21F, 0x0FD, 0x167, 0x361, 0x362, 0x363, 0x364, 0x365, 0x366)
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
  parser.add_argument("--during-seconds", type=float, default=3.0,
                      help="Seconds to count target frames while holding tester-present active")
  parser.add_argument("--progress-interval", type=float, default=0.0,
                      help="Print a progress line every N seconds during the active hold; 0 disables")
  parser.add_argument("--disable-rx-too", action="store_true", help="Use DISABLE_RX_DISABLE_TX (0x28 03 01) instead of ENABLE_RX_DISABLE_TX (0x28 01 01)")
  parser.add_argument("--session", choices=tuple(SESSION_BY_NAME), default="extended", help="Diagnostic session to enter before communication-control")
  parser.add_argument("--try-all-sessions", action="store_true", help="Try default, extended, and safety-system sessions until communication-control is accepted")
  parser.add_argument("--session-only", action="store_true", help="Do not send communication-control. Hold the chosen diagnostic session and observe traffic while tester-present is active.")
  parser.add_argument("--exit-default-session", action="store_true",
                      help="After the hold, explicitly request UDS default session on the radar before returning Panda to silent mode")
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


def count_addrs(panda: Panda, bus: int, seconds: float) -> Counter[int]:
  deadline = time.monotonic() + seconds
  counts: Counter[int] = Counter()
  panda.can_clear(0xFFFF)
  while time.monotonic() < deadline:
    for addr, _dat, src in panda.can_recv():
      if src == bus and addr in TARGET_ADDRS:
        counts[addr] += 1
    time.sleep(0.001)
  return counts


def count_addrs_with_tester_present(panda: Panda, bus: int, addr: int, seconds: float,
                                    tester_present_interval: float = 0.5,
                                    progress_interval: float = 0.0) -> Counter[int]:
  deadline = time.monotonic() + seconds
  next_tester_present = time.monotonic()
  next_progress = time.monotonic() + progress_interval if progress_interval > 0 else float("inf")
  counts: Counter[int] = Counter()
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

    for rx_addr, _dat, src in panda.can_recv():
      if src == bus and rx_addr in TARGET_ADDRS:
        counts[rx_addr] += 1
    time.sleep(0.001)
  return counts


def format_counts(title: str, counts: Counter[int], seconds: float) -> str:
  lines = [title]
  for addr in TARGET_ADDRS:
    hz = counts[addr] / seconds
    lines.append(f"  {hex(addr)} count={counts[addr]} hz={hz:.2f}")
  return "\n".join(lines)


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

  panda = Panda()
  panda.set_can_speed_kbps(args.bus, args.can_speed_kbps)
  panda.set_safety_mode(CarParams.SafetyModel.elm327)

  control_type = uds.CONTROL_TYPE.DISABLE_RX_DISABLE_TX if args.disable_rx_too else uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX
  report: list[str] = []
  disable_succeeded = False
  session_entered = False

  try:
    before_counts = count_addrs(panda, args.bus, args.before_seconds)
    report.append(format_counts("Before disable:", before_counts, args.before_seconds))

    client = uds.UdsClient(panda, args.addr, bus=args.bus)
    report.append(f"\nUDS session start: addr={hex(args.addr)} bus={args.bus} control_type={control_type.name}")

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
        observed_counts = count_addrs_with_tester_present(
          panda, args.bus, args.addr, args.during_seconds,
          progress_interval=args.progress_interval,
        )
        report.append("")
        title = "During active session:" if args.session_only and not disable_succeeded else "During disable hold:"
        report.append(format_counts(title, observed_counts, args.during_seconds))
        report.append("\nTarget message deltas:")
        for addr in TARGET_ADDRS:
          before = before_counts[addr]
          during = observed_counts[addr]
          ratio = (during / before) if before else 0.0
          report.append(f"  {hex(addr)} before={before} during={during} ratio={ratio:.2f}")
        break

    if not (disable_succeeded or args.session_only):
      after_counts = count_addrs(panda, args.bus, args.before_seconds)
      report.append("")
      report.append(format_counts("After disable:", after_counts, args.before_seconds))

      report.append("\nTarget message deltas:")
      for addr in TARGET_ADDRS:
        before = before_counts[addr]
        after = after_counts[addr]
        ratio = (after / before) if before else 0.0
        report.append(f"  {hex(addr)} before={before} after={after} ratio={ratio:.2f}")

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

  finally:
    panda.set_safety_mode(CarParams.SafetyModel.silent)
    report.append("\nPanda returned to silent safety mode.")

  output = "\n".join(report)
  print(output)
  if args.output is not None:
    args.output.write_text(output + "\n")
    print(f"\nWrote report to {args.output}")


if __name__ == "__main__":
  main()
