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
  MazdaLongitudinalProfile = module.MazdaLongitudinalProfile
  MazdaLongitudinalReplayMutator = module.MazdaLongitudinalReplayMutator
  command_stream_from_hex_sequence = module.command_stream_from_hex_sequence

from opendbc.car.structs import CarParams
from panda import Panda


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Run a Mazda longitudinal replay stream through Panda at stock phase cadence.")
  parser.add_argument("stream_json", type=Path, help="JSON file produced by extract_mazda_longitudinal_replay.py")
  parser.add_argument("--profile", choices=[profile.value for profile in MazdaLongitudinalProfile], help="Override the profile stored in the JSON")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus to send on")
  parser.add_argument("--can-speed-kbps", type=int, default=500, help="CAN bus speed in kbps")
  parser.add_argument("--duration", type=float, default=10.0, help="Replay duration in seconds, 0 for infinite")
  parser.add_argument("--info-accel-cmd", type=float, default=0.0, help="CRZ_INFO.ACCEL_CMD override")
  parser.add_argument("--gas-cmd", type=float, help="Optional GAS.GAS_CMD override")
  parser.add_argument("--unsafe-patch-events", action="store_true", help="Also patch CRZ_EVENTS using approximate scaling")
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

  start_time = time.monotonic()
  next_send = start_time

  print(f"Running Mazda replay: profile={stream.profile.value} bus={args.bus} speed={args.can_speed_kbps}kbps duration={args.duration}s")
  print("Phase cadence: A=[0xfd,0x167,0x21f], B=[0x21b,0x21c], every 10 ms.")
  print("Press Ctrl-C to stop.")

  try:
    while args.duration == 0 or (time.monotonic() - start_time) < args.duration:
      can_msgs = mutator.next_phase_can_data(
        profile=stream.profile,
        info_accel_cmd=args.info_accel_cmd,
        gas_cmd=args.gas_cmd,
        unsafe_patch_events=args.unsafe_patch_events,
        bus=args.bus,
      )
      panda.can_send_many([(msg.address, msg.dat, msg.src) for msg in can_msgs], timeout=0)

      next_send += 0.01
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
