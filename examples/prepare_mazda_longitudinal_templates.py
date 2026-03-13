#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from opendbc.can import CANParser
from opendbc.car.logreader import LogReader
from opendbc.car.mazda.longitudinal_experimental import (
  CRZ_CTRL_MODE_TEMPLATES,
  MazdaLongitudinalCommandSet,
  MazdaLongitudinalProfile,
  build_command_set,
  decode_signal,
  matches_crz_info_checksum_guess,
)


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


@dataclass(frozen=True)
class TemplateSample:
  profile: MazdaLongitudinalProfile
  path: Path
  t_s: float
  score: float
  v_ego: float
  standstill: bool
  lead: bool
  raw_21b: bytes
  raw_21c: bytes
  raw_21f: bytes
  raw_fd: bytes
  raw_167: bytes

  @property
  def command_set(self) -> MazdaLongitudinalCommandSet:
    return MazdaLongitudinalCommandSet(
      profile=self.profile,
      raw_21b=self.raw_21b,
      raw_21c=self.raw_21c,
      raw_21f=self.raw_21f,
      raw_fd=self.raw_fd,
      raw_167=self.raw_167,
    )


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Extract canonical Mazda longitudinal message templates from one or more rlogs.")
  parser.add_argument("rlogs", nargs="+", type=Path, help="Mazda rlog.zst files to scan")
  parser.add_argument("--bus", type=int, default=0, help="CAN src/bus to decode (default: 0)")
  return parser.parse_args()


def choose_profile(cp: CANParser, raw_21c: bytes) -> MazdaLongitudinalProfile | None:
  available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1.0
  active = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1.0
  standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1.0
  has_lead = cp.vl["CRZ_CTRL"]["RADAR_HAS_LEAD"] == 1.0
  v_ego = cp.vl["ENGINE_DATA"]["SPEED"] * (1000.0 / 3600.0)
  mode_byte = raw_21c[3]

  if available and not active:
    return MazdaLongitudinalProfile.STANDBY
  if active and standstill and v_ego < 0.3:
    return MazdaLongitudinalProfile.STOP_GO_HOLD
  if active and mode_byte == CRZ_CTRL_MODE_TEMPLATES[MazdaLongitudinalProfile.ENGAGED_FOLLOW][3] and has_lead:
    return MazdaLongitudinalProfile.ENGAGED_FOLLOW
  if active and mode_byte == CRZ_CTRL_MODE_TEMPLATES[MazdaLongitudinalProfile.ENGAGED_CRUISE][3] and v_ego > 5.0:
    return MazdaLongitudinalProfile.ENGAGED_CRUISE
  return None


def score_sample(profile: MazdaLongitudinalProfile, cp: CANParser, raw_21b: bytes, raw_21c: bytes) -> float:
  v_ego = cp.vl["ENGINE_DATA"]["SPEED"] * (1000.0 / 3600.0)
  gas_pressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0.0
  brake_pressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1.0
  info_accel_cmd = cp.vl["CRZ_INFO"]["ACCEL_CMD"]
  has_lead = cp.vl["CRZ_CTRL"]["RADAR_HAS_LEAD"] == 1.0
  mode_template = CRZ_CTRL_MODE_TEMPLATES[profile]
  mode_penalty = 0.0
  for index in (0, 2, 3, 6):
    mode_penalty += 250.0 * float(raw_21c[index] != mode_template[index])

  penalty = 10_000.0 * float(gas_pressed) + 10_000.0 * float(brake_pressed)

  if profile == MazdaLongitudinalProfile.STANDBY:
    return penalty + mode_penalty + abs(v_ego - 15.0)
  if profile == MazdaLongitudinalProfile.ENGAGED_CRUISE:
    return penalty + mode_penalty + abs(info_accel_cmd) + abs(v_ego - 30.0)
  if profile == MazdaLongitudinalProfile.ENGAGED_FOLLOW:
    return penalty + mode_penalty + abs(info_accel_cmd) + abs(v_ego - 28.0) + 500.0 * float(not has_lead)
  checksum_penalty = 2_000.0 * float(not matches_crz_info_checksum_guess(raw_21b))
  return penalty + mode_penalty + checksum_penalty + abs(info_accel_cmd + 1024.0) + 200.0 * abs(v_ego)


def extract_templates(paths: list[Path], bus: int) -> dict[MazdaLongitudinalProfile, TemplateSample]:
  templates: dict[MazdaLongitudinalProfile, TemplateSample] = {}

  for path in paths:
    if not path.is_file():
      raise FileNotFoundError(path)

    cp = CANParser("mazda_2017", list(CANDIDATE_MESSAGES), bus)
    last_raw: dict[int, bytes] = {}

    for event in LogReader(str(path), only_union_types=True):
      if event.which() != "can":
        continue

      frames = []
      for can_msg in event.can:
        dat = bytes(can_msg.dat)
        frames.append((can_msg.address, dat, can_msg.src))
        if can_msg.src == bus and can_msg.address in (0x21B, 0x21C, 0x21F, 0x0FD, 0x167):
          last_raw[can_msg.address] = dat

      cp.update((event.logMonoTime, frames))

      if not all(address in last_raw for address in (0x21B, 0x21C, 0x21F, 0x0FD, 0x167)):
        continue

      raw_21c = last_raw[0x21C]
      profile = choose_profile(cp, raw_21c)
      if profile is None:
        continue

      score = score_sample(profile, cp, last_raw[0x21B], raw_21c)
      v_ego = cp.vl["ENGINE_DATA"]["SPEED"] * (1000.0 / 3600.0)
      candidate = TemplateSample(
        profile=profile,
        path=path,
        t_s=event.logMonoTime * 1e-9,
        score=score,
        v_ego=v_ego,
        standstill=cp.vl["PEDALS"]["STANDSTILL"] == 1.0,
        lead=cp.vl["CRZ_CTRL"]["RADAR_HAS_LEAD"] == 1.0,
        raw_21b=last_raw[0x21B],
        raw_21c=raw_21c,
        raw_21f=last_raw[0x21F],
        raw_fd=last_raw[0x0FD],
        raw_167=last_raw[0x167],
      )

      current = templates.get(profile)
      if current is None or candidate.score < current.score:
        templates[profile] = candidate

  return templates


def print_command_set(sample: TemplateSample) -> None:
  print(f"{sample.profile.value}: log={sample.path.name} t={sample.t_s:.3f}s score={sample.score:.1f} vEgo={sample.v_ego:.2f} standstill={int(sample.standstill)} lead={int(sample.lead)}")
  print(f"  CRZ_INFO  0x21b {sample.raw_21b.hex()} checksum_guess_ok={matches_crz_info_checksum_guess(sample.raw_21b)} ctr1={decode_signal('CRZ_INFO', sample.raw_21b, 'CTR1'):.0f} accel_cmd={decode_signal('CRZ_INFO', sample.raw_21b, 'ACCEL_CMD'):.0f}")
  print(f"  CRZ_CTRL  0x21c {sample.raw_21c.hex()} mode_byte3=0x{sample.raw_21c[3]:02x}")
  print(f"  CRZ_EVENTS 0x21f {sample.raw_21f.hex()} ctr={decode_signal('CRZ_EVENTS', sample.raw_21f, 'CTR'):.0f} accel_cmd={decode_signal('CRZ_EVENTS', sample.raw_21f, 'ACCEL_CMD'):.0f} accel_low_res={decode_signal('CRZ_EVENTS', sample.raw_21f, 'ACCEL_CMD_LOW_RES'):.0f} crz_speed={decode_signal('CRZ_EVENTS', sample.raw_21f, 'CRZ_SPEED'):.3f} kph")
  print(f"  GAS       0x0fd {sample.raw_fd.hex()} ctr={decode_signal('GAS', sample.raw_fd, 'CTR'):.0f} gas_cmd={decode_signal('GAS', sample.raw_fd, 'GAS_CMD'):.0f}")
  print(f"  MORE_GAS  0x167 {sample.raw_167.hex()} ctr={decode_signal('MORE_GAS', sample.raw_167, 'CTR'):.0f}")


def print_python_literal(templates: dict[MazdaLongitudinalProfile, TemplateSample]) -> None:
  print("\nPython literal:")
  print("MAZDA_LONG_TEMPLATE_HEX = {")
  for profile in MazdaLongitudinalProfile:
    sample = templates.get(profile)
    if sample is None:
      continue
    print(f"  '{profile.value}': {{")
    print(f"    'CRZ_INFO': '{sample.raw_21b.hex()}',")
    print(f"    'CRZ_CTRL': '{sample.raw_21c.hex()}',")
    print(f"    'CRZ_EVENTS': '{sample.raw_21f.hex()}',")
    print(f"    'GAS': '{sample.raw_fd.hex()}',")
    print(f"    'MORE_GAS': '{sample.raw_167.hex()}',")
    print("  },")
  print("}")


def print_patch_examples(templates: dict[MazdaLongitudinalProfile, TemplateSample]) -> None:
  cruise = templates.get(MazdaLongitudinalProfile.ENGAGED_CRUISE)
  hold = templates.get(MazdaLongitudinalProfile.STOP_GO_HOLD)
  if cruise is None:
    return

  neutral = build_command_set(cruise.command_set, profile=MazdaLongitudinalProfile.ENGAGED_CRUISE, crz_info_accel_cmd=0.0)
  accel = build_command_set(cruise.command_set, profile=MazdaLongitudinalProfile.ENGAGED_CRUISE, crz_info_accel_cmd=160.0)
  brake = build_command_set((hold or cruise).command_set, profile=MazdaLongitudinalProfile.STOP_GO_HOLD if hold is not None else MazdaLongitudinalProfile.ENGAGED_CRUISE, crz_info_accel_cmd=-512.0)

  print("\nCRZ_INFO patch examples (0x21f intentionally left untouched because its checksum is not solved yet):")
  print(f"  engaged_cruise neutral  raw_21b={neutral.raw_21b.hex()} accel_cmd={decode_signal('CRZ_INFO', neutral.raw_21b, 'ACCEL_CMD'):.0f} checksum_guess_ok={matches_crz_info_checksum_guess(neutral.raw_21b)}")
  print(f"  engaged_cruise accel+   raw_21b={accel.raw_21b.hex()} accel_cmd={decode_signal('CRZ_INFO', accel.raw_21b, 'ACCEL_CMD'):.0f} checksum_guess_ok={matches_crz_info_checksum_guess(accel.raw_21b)}")
  print(f"  stop_go brake-         raw_21b={brake.raw_21b.hex()} accel_cmd={decode_signal('CRZ_INFO', brake.raw_21b, 'ACCEL_CMD'):.0f} checksum_guess_ok={matches_crz_info_checksum_guess(brake.raw_21b)}")


def print_next_steps(templates: dict[MazdaLongitudinalProfile, TemplateSample]) -> None:
  print("\nImmediate next test steps:")
  print("  1. Use the engaged_cruise template as the first sender baseline for 0x21b/0x21c/0xfd/0x167.")
  print("  2. Keep CRZ_CTRL mode selection explicit: cruise centers on 0x20, follow on 0x40, and stop-go cycles through 0x20/0x40/0x60 in byte 3.")
  print("  3. Use the current CRZ_INFO checksum logic only as a high-speed guess; at least one low-speed hold submode does not fit it.")
  print("  4. Keep CRZ_EVENTS as logged replay traffic until its checksum is solved, or solve that checksum before synthetic 0x21f mutation.")
  print("  5. For the first on-car experiment, prefer full message replay/mutation over final controller logic.")


def main() -> None:
  args = parse_args()
  templates = extract_templates(args.rlogs, args.bus)

  print("Selected Mazda longitudinal templates:")
  for profile in MazdaLongitudinalProfile:
    sample = templates.get(profile)
    if sample is None:
      print(f"{profile.value}: not found")
      continue
    print_command_set(sample)

  print_python_literal(templates)
  print_patch_examples(templates)
  print_next_steps(templates)


if __name__ == "__main__":
  main()
