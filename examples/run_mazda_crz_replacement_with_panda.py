#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path
from subprocess import CalledProcessError, check_output

for candidate in (Path("/data/opendbc_test"),):
  if candidate.exists():
    candidate_str = str(candidate)
    if candidate_str not in sys.path:
      sys.path.insert(0, candidate_str)

try:
  from opendbc.car.mazda.longitudinal_experimental import (
    CRZ_CTRL_ADDR,
    CRZ_INFO_ADDR,
    MazdaLongitudinalProfile,
    MazdaLongitudinalReplayMutator,
    command_stream_from_hex_sequence,
  )
except ModuleNotFoundError:
  helper_path = Path("/data/opendbc_test/opendbc/car/mazda/longitudinal_experimental.py")
  if not helper_path.exists():
    raise

  spec = importlib.util.spec_from_file_location("mazda_longitudinal_experimental", helper_path)
  if spec is None or spec.loader is None:
    raise RuntimeError(f"Unable to load Mazda helper module from {helper_path}")
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  CRZ_CTRL_ADDR = module.CRZ_CTRL_ADDR
  CRZ_INFO_ADDR = module.CRZ_INFO_ADDR
  MazdaLongitudinalProfile = module.MazdaLongitudinalProfile
  MazdaLongitudinalReplayMutator = module.MazdaLongitudinalReplayMutator
  command_stream_from_hex_sequence = module.command_stream_from_hex_sequence

from opendbc.car import uds
from opendbc.car.structs import CarParams
from panda import Panda


SESSION_BY_NAME: dict[str, uds.SESSION_TYPE] = {
  "default": uds.SESSION_TYPE.DEFAULT,
  "programming": uds.SESSION_TYPE.PROGRAMMING,
  "extended": uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC,
  "safety": uds.SESSION_TYPE.SAFETY_SYSTEM_DIAGNOSTIC,
}


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Hold Mazda radar in a diagnostic session and replace suppressed CRZ_INFO/CRZ_CTRL frames through Panda."
  )
  parser.add_argument("stream_json", type=Path, help="JSON file produced by extract_mazda_longitudinal_replay.py")
  parser.add_argument("--profile", choices=[profile.value for profile in MazdaLongitudinalProfile], help="Override the profile stored in the JSON")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus to send on")
  parser.add_argument("--can-speed-kbps", type=int, default=500, help="CAN bus speed in kbps")
  parser.add_argument("--duration", type=float, default=5.0, help="Replay duration in seconds, 0 for infinite")
  parser.add_argument("--info-accel-cmd", type=float, default=0.0, help="CRZ_INFO.ACCEL_CMD override")
  parser.add_argument("--session", choices=tuple(SESSION_BY_NAME), default="programming", help="Diagnostic session to hold on the radar ECU")
  parser.add_argument("--radar-addr", type=lambda x: int(x, 0), default=0x764, help="Radar UDS address")
  parser.add_argument("--tester-present-interval", type=float, default=0.5, help="Seconds between raw 0x3E80 tester-present frames")
  parser.add_argument("--all-output", action="store_true", help="Required. Sends with Panda allOutput safety mode and disables normal TX safety.")
  return parser.parse_args()


def ensure_pandad_stopped() -> None:
  try:
    check_output(["pidof", "pandad"])
    print("pandad is running, please stop openpilot before using direct Panda replay.")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1:
      raise


def load_stream(path: Path, profile_override: str | None):
  payload = json.loads(path.read_text())
  profile = MazdaLongitudinalProfile(profile_override or payload["profile"])
  return command_stream_from_hex_sequence(profile, payload["frames"])


def send_tester_present_suppress_response(panda: Panda, bus: int, addr: int) -> None:
  panda.can_send(addr, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus)


def main() -> None:
  args = parse_args()
  if not args.all_output:
    print("--all-output is required for this script. This mode disables Panda TX safety checks.")
    sys.exit(1)

  ensure_pandad_stopped()
  stream = load_stream(args.stream_json, args.profile)
  mutator = MazdaLongitudinalReplayMutator(stream)

  panda = Panda()
  panda.set_can_speed_kbps(args.bus, args.can_speed_kbps)
  panda.set_safety_mode(CarParams.SafetyModel.allOutput)

  print(f"Entering radar session: addr={hex(args.radar_addr)} session={args.session}")
  client = uds.UdsClient(panda, args.radar_addr, bus=args.bus)
  client.diagnostic_session_control(SESSION_BY_NAME[args.session])
  send_tester_present_suppress_response(panda, args.bus, args.radar_addr)

  start_time = time.monotonic()
  next_send = start_time
  next_tester_present = start_time + args.tester_present_interval

  print(f"Running Mazda CRZ replacement: profile={stream.profile.value} bus={args.bus} speed={args.can_speed_kbps}kbps duration={args.duration}s")
  print("Sending 0x21b and 0x21c only at 50 Hz while holding the radar diagnostic session.")
  print("Press Ctrl-C to stop.")

  try:
    while args.duration == 0 or (time.monotonic() - start_time) < args.duration:
      now = time.monotonic()
      if now >= next_tester_present:
        send_tester_present_suppress_response(panda, args.bus, args.radar_addr)
        next_tester_present = now + args.tester_present_interval

      command_set = mutator.next_command_set(
        profile=stream.profile,
        info_accel_cmd=args.info_accel_cmd,
        unsafe_patch_events=False,
      )
      panda.can_send_many(
        [
          (CRZ_INFO_ADDR, command_set.raw_21b, args.bus),
          (CRZ_CTRL_ADDR, command_set.raw_21c, args.bus),
        ],
        timeout=0,
      )

      next_send += 0.02
      sleep_time = next_send - time.monotonic()
      if sleep_time > 0:
        time.sleep(sleep_time)
      else:
        next_send = time.monotonic()
  finally:
    panda.set_safety_mode(CarParams.SafetyModel.silent)
    print("Panda returned to silent safety mode.")


if __name__ == "__main__":
  main()
