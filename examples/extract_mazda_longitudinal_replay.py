#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path

from opendbc.can import CANParser
from opendbc.car.logreader import LogReader

try:
  from opendbc.car.mazda.longitudinal_experimental import (
    MazdaLongitudinalCommandSet,
    MazdaLongitudinalCommandStream,
    MazdaLongitudinalProfile,
    MazdaLongitudinalReplayMutator,
    decode_signal,
    matches_crz_info_checksum_guess,
  )
except ModuleNotFoundError:
  helper_path = Path(__file__).resolve().parents[1] / "opendbc" / "car" / "mazda" / "longitudinal_experimental.py"
  if not helper_path.exists():
    raise

  spec = importlib.util.spec_from_file_location("mazda_longitudinal_experimental", helper_path)
  if spec is None or spec.loader is None:
    raise RuntimeError(f"Unable to load Mazda helper module from {helper_path}")
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  MazdaLongitudinalCommandSet = module.MazdaLongitudinalCommandSet
  MazdaLongitudinalCommandStream = module.MazdaLongitudinalCommandStream
  MazdaLongitudinalProfile = module.MazdaLongitudinalProfile
  MazdaLongitudinalReplayMutator = module.MazdaLongitudinalReplayMutator
  decode_signal = module.decode_signal
  matches_crz_info_checksum_guess = module.matches_crz_info_checksum_guess


CANDIDATE_MESSAGES: tuple[tuple[str, int], ...] = (
  ("CRZ_CTRL", 50),
  ("CRZ_INFO", 50),
  ("CRZ_EVENTS", 50),
  ("GAS", 50),
  ("MORE_GAS", 50),
  ("BRAKE", 50),
  ("ENGINE_DATA", 100),
  ("PEDALS", 50),
)

TARGET_MODE_BYTES: dict[MazdaLongitudinalProfile, set[int]] = {
  MazdaLongitudinalProfile.STANDBY: {0x00},
  MazdaLongitudinalProfile.ENGAGED_CRUISE: {0x20},
  MazdaLongitudinalProfile.ENGAGED_FOLLOW: {0x40},
  MazdaLongitudinalProfile.STOP_GO_HOLD: {0x20, 0x40, 0x60},
}


@dataclass(frozen=True)
class ReplaySample:
  path: Path
  t_s: float
  score: float
  command_set: MazdaLongitudinalCommandSet


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Extract a contiguous Mazda longitudinal replay window from one or more rlogs.")
  parser.add_argument("rlogs", nargs="+", type=Path, help="Mazda rlog.zst files to scan")
  parser.add_argument("--profile", required=True, choices=[profile.value for profile in MazdaLongitudinalProfile], help="Target Mazda longitudinal profile")
  parser.add_argument("--frames", type=int, default=16, help="Number of 50 Hz frames to extract")
  parser.add_argument("--bus", type=int, default=0, help="CAN src/bus to decode (default: 0)")
  parser.add_argument("--output", type=Path, help="Optional JSON output path for the extracted replay window")
  return parser.parse_args()


def choose_profile(cp: CANParser, raw_21c: bytes) -> MazdaLongitudinalProfile | None:
  available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1.0
  active = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1.0
  standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1.0
  has_lead = cp.vl["CRZ_CTRL"]["RADAR_HAS_LEAD"] == 1.0
  v_ego = cp.vl["ENGINE_DATA"]["SPEED"] * (1000.0 / 3600.0)
  mode_byte = raw_21c[3]

  if available and not active and mode_byte == 0x00:
    return MazdaLongitudinalProfile.STANDBY
  if active and standstill and v_ego < 0.3:
    return MazdaLongitudinalProfile.STOP_GO_HOLD
  if active and has_lead and mode_byte == 0x40:
    return MazdaLongitudinalProfile.ENGAGED_FOLLOW
  if active and mode_byte == 0x20 and v_ego > 5.0:
    return MazdaLongitudinalProfile.ENGAGED_CRUISE
  return None


def score_sample(profile: MazdaLongitudinalProfile, cp: CANParser, raw_21b: bytes, raw_21c: bytes) -> float:
  v_ego = cp.vl["ENGINE_DATA"]["SPEED"] * (1000.0 / 3600.0)
  info_accel_cmd = cp.vl["CRZ_INFO"]["ACCEL_CMD"]
  gas_pressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0.0
  brake_pressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1.0
  lead = cp.vl["CRZ_CTRL"]["RADAR_HAS_LEAD"] == 1.0
  mode_penalty = 1000.0 * float(raw_21c[3] not in TARGET_MODE_BYTES[profile])
  checksum_penalty = 500.0 * float(not matches_crz_info_checksum_guess(raw_21b))
  pedal_penalty = 10_000.0 * float(gas_pressed) + 10_000.0 * float(brake_pressed)

  if profile == MazdaLongitudinalProfile.STANDBY:
    return mode_penalty + checksum_penalty + pedal_penalty + abs(v_ego - 15.0)
  if profile == MazdaLongitudinalProfile.ENGAGED_CRUISE:
    return mode_penalty + checksum_penalty + pedal_penalty + abs(info_accel_cmd) + abs(v_ego - 30.0)
  if profile == MazdaLongitudinalProfile.ENGAGED_FOLLOW:
    return mode_penalty + checksum_penalty + pedal_penalty + abs(info_accel_cmd) + abs(v_ego - 28.0) + 500.0 * float(not lead)
  return mode_penalty + checksum_penalty + pedal_penalty + abs(info_accel_cmd + 1024.0) + 200.0 * abs(v_ego)


def collect_matching_samples(paths: list[Path], profile: MazdaLongitudinalProfile, bus: int) -> list[ReplaySample]:
  matches: list[ReplaySample] = []

  for path in paths:
    if not path.is_file():
      raise FileNotFoundError(path)

    cp = CANParser("mazda_2017", list(CANDIDATE_MESSAGES), bus)
    last_raw: dict[int, bytes] = {}

    for event in LogReader(str(path), only_union_types=True):
      if event.which() != "can":
        continue

      frames = []
      updated_addrs: set[int] = set()
      for can_msg in event.can:
        dat = bytes(can_msg.dat)
        frames.append((can_msg.address, dat, can_msg.src))
        if can_msg.src == bus and can_msg.address in (0x21B, 0x21C, 0x21F, 0x0FD, 0x167):
          last_raw[can_msg.address] = dat
          updated_addrs.add(can_msg.address)

      cp.update((event.logMonoTime, frames))
      if not all(address in last_raw for address in (0x21B, 0x21C, 0x21F, 0x0FD, 0x167)):
        continue
      if 0x21B not in updated_addrs or 0x21C not in updated_addrs:
        continue

      chosen = choose_profile(cp, last_raw[0x21C])
      if chosen != profile:
        continue

      score = score_sample(profile, cp, last_raw[0x21B], last_raw[0x21C])
      matches.append(ReplaySample(
        path=path,
        t_s=event.logMonoTime * 1e-9,
        score=score,
        command_set=MazdaLongitudinalCommandSet(
          profile=profile,
          raw_21b=last_raw[0x21B],
          raw_21c=last_raw[0x21C],
          raw_21f=last_raw[0x21F],
          raw_fd=last_raw[0x0FD],
          raw_167=last_raw[0x167],
        ),
      ))

  return matches


def choose_best_window(samples: list[ReplaySample], frames: int) -> tuple[MazdaLongitudinalCommandStream, list[ReplaySample]]:
  if len(samples) < frames:
    raise RuntimeError(f"Not enough matching samples: found {len(samples)}, need {frames}")

  best_window: list[ReplaySample] | None = None
  best_score = float("inf")

  for start in range(0, len(samples) - frames + 1):
    window = samples[start:start + frames]
    same_log = all(sample.path == window[0].path for sample in window)
    contiguous = all((window[i].t_s - window[i - 1].t_s) < 0.03 for i in range(1, len(window)))
    if not same_log or not contiguous:
      continue

    avg_score = sum(sample.score for sample in window) / len(window)
    if avg_score < best_score:
      best_score = avg_score
      best_window = window

  if best_window is None:
    raise RuntimeError("No contiguous replay window found")

  return MazdaLongitudinalCommandStream(
    profile=best_window[0].command_set.profile or MazdaLongitudinalProfile.ENGAGED_CRUISE,
    commands=tuple(sample.command_set for sample in best_window),
  ), best_window


def print_window(stream: MazdaLongitudinalCommandStream, samples: list[ReplaySample], frames: int) -> None:
  print(f"Selected replay window: profile={stream.profile.value} frames={len(stream)}")
  for index, command_set in enumerate(stream.commands):
    sample = samples[index]
    print(
      f"  frame={index:02d} log={sample.path.name} t={sample.t_s:.3f}s "
      f"ctr1={decode_signal('CRZ_INFO', command_set.raw_21b, 'CTR1'):.0f} "
      f"events_ctr={decode_signal('CRZ_EVENTS', command_set.raw_21f, 'CTR'):.0f} "
      f"gas_ctr={decode_signal('GAS', command_set.raw_fd, 'CTR'):.0f} "
      f"more_gas_ctr={decode_signal('MORE_GAS', command_set.raw_167 or b'\\x00' * 8, 'CTR'):.0f} "
      f"mode=0x{command_set.raw_21c[3]:02x}"
    )

  print("\nPhase cadence:")
  print("  phase A at 100 Hz half-slot: 0x0fd GAS, 0x167 MORE_GAS, 0x21f CRZ_EVENTS")
  print("  phase B 10 ms later:        0x21b CRZ_INFO, 0x21c CRZ_CTRL")

  print("\nPython literal:")
  print("MAZDA_LONG_REPLAY_STREAM_HEX = [")
  for command_set in stream.commands:
    print("  {")
    print(f"    'CRZ_INFO': '{command_set.raw_21b.hex()}',")
    print(f"    'CRZ_CTRL': '{command_set.raw_21c.hex()}',")
    print(f"    'CRZ_EVENTS': '{command_set.raw_21f.hex()}',")
    print(f"    'GAS': '{command_set.raw_fd.hex()}',")
    print(f"    'MORE_GAS': '{(command_set.raw_167 or b'').hex()}',")
    print("  },")
  print("]")


def write_json(stream: MazdaLongitudinalCommandStream, output: Path) -> None:
  payload = {
    "profile": stream.profile.value,
    "frames": [
      {
        "CRZ_INFO": command_set.raw_21b.hex(),
        "CRZ_CTRL": command_set.raw_21c.hex(),
        "CRZ_EVENTS": command_set.raw_21f.hex(),
        "GAS": command_set.raw_fd.hex(),
        "MORE_GAS": (command_set.raw_167 or b"").hex(),
      }
      for command_set in stream.commands
    ],
  }
  output.write_text(json.dumps(payload, indent=2) + "\n")


def print_mutation_preview(stream: MazdaLongitudinalCommandStream) -> None:
  mutator = MazdaLongitudinalReplayMutator(stream)
  neutral = mutator.next_command_set(info_accel_cmd=0.0)
  accel = mutator.next_command_set(info_accel_cmd=160.0)
  brake = mutator.next_command_set(info_accel_cmd=-512.0)

  print("\nMutation preview:")
  print(f"  neutral raw_21b={neutral.raw_21b.hex()} accel_cmd={decode_signal('CRZ_INFO', neutral.raw_21b, 'ACCEL_CMD'):.0f}")
  print(f"  accel+  raw_21b={accel.raw_21b.hex()} accel_cmd={decode_signal('CRZ_INFO', accel.raw_21b, 'ACCEL_CMD'):.0f}")
  print(f"  brake-  raw_21b={brake.raw_21b.hex()} accel_cmd={decode_signal('CRZ_INFO', brake.raw_21b, 'ACCEL_CMD'):.0f}")
  print("  CRZ_EVENTS is preserved from the captured stream unless you explicitly enable unsafe patching in code.")
  mutator.reset()
  phase_a = mutator.next_phase_can_data(info_accel_cmd=0.0)
  phase_b = mutator.next_phase_can_data(info_accel_cmd=0.0)
  print(f"  phase A can ids={[hex(msg.address) for msg in phase_a]}")
  print(f"  phase B can ids={[hex(msg.address) for msg in phase_b]}")


def main() -> None:
  args = parse_args()
  profile = MazdaLongitudinalProfile(args.profile)
  samples = collect_matching_samples(args.rlogs, profile, args.bus)
  stream, window_samples = choose_best_window(samples, args.frames)

  print_window(stream, window_samples, args.frames)
  print_mutation_preview(stream)
  if args.output is not None:
    write_json(stream, args.output)
    print(f"\nWrote replay window JSON to {args.output}")


if __name__ == "__main__":
  main()
