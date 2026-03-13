#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import median

from opendbc.can import CANParser
from opendbc.car.logreader import LogReader


CANDIDATE_MESSAGES: dict[str, int] = {
  "CRZ_CTRL": 0x21C,
  "CRZ_INFO": 0x21B,
  "CRZ_EVENTS": 0x21F,
  "GAS": 0x0FD,
  "MORE_GAS": 0x167,
}

SUPPORT_MESSAGES: list[tuple[str, int]] = [
  ("BRAKE", 50),
  ("ENGINE_DATA", 100),
  ("PEDALS", 50),
]

FOCUSED_FIELDS: tuple[tuple[str, str], ...] = (
  ("CRZ_CTRL", "CRZ_ACTIVE"),
  ("CRZ_CTRL", "CRZ_AVAILABLE"),
  ("CRZ_CTRL", "ACC_ACTIVE_2"),
  ("CRZ_CTRL", "RADAR_HAS_LEAD"),
  ("CRZ_CTRL", "RADAR_LEAD_RELATIVE_DISTANCE"),
  ("CRZ_CTRL", "DISTANCE_SETTING"),
  ("CRZ_CTRL", "ACC_GAS_MAYBE"),
  ("CRZ_CTRL", "ACC_GAS_MAYBE2"),
  ("CRZ_CTRL", "5_SEC_DISABLE_TIMER"),
  ("CRZ_EVENTS", "ACCEL_CMD"),
  ("CRZ_EVENTS", "ACCEL_CMD_LOW_RES"),
  ("CRZ_EVENTS", "CRZ_SPEED"),
  ("CRZ_INFO", "ACCEL_CMD"),
  ("GAS", "GAS_CMD"),
)


@dataclass(frozen=True)
class LogData:
  path: Path
  duration_s: float
  can_samples: int
  speed_min: float
  speed_max: float
  cruise_speed_min: float
  cruise_speed_max: float
  available_samples: int
  engaged_samples: int
  lead_samples: int
  standstill_samples: int
  gas_pressed_samples: int
  brake_pressed_samples: int
  field_values: dict[str, list[float]]
  raw_counts: dict[int, Counter[str]]


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Compare Mazda ACC CAN behavior between two or more rlogs.")
  parser.add_argument("rlogs", nargs="+", type=Path, help="Two or more Mazda rlog.zst files")
  parser.add_argument("--bus", type=int, default=0, help="CAN src/bus to decode (default: 0)")
  parser.add_argument("--top-raw", type=int, default=3, help="Number of top raw payloads to print per message")
  return parser.parse_args()


def extract_log(path: Path, bus: int) -> LogData:
  if not path.is_file():
    raise FileNotFoundError(path)

  messages = [(msg_name, 50) for msg_name in CANDIDATE_MESSAGES] + SUPPORT_MESSAGES
  cp = CANParser("mazda_2017", messages, bus)

  field_values: dict[str, list[float]] = {f"{msg}.{sig}": [] for msg, sig in FOCUSED_FIELDS}
  raw_counts: dict[int, Counter[str]] = defaultdict(Counter)

  first_can_time = None
  last_can_time = None
  can_samples = 0
  available_samples = 0
  engaged_samples = 0
  lead_samples = 0
  standstill_samples = 0
  gas_pressed_samples = 0
  brake_pressed_samples = 0
  speeds: list[float] = []
  cruise_speeds: list[float] = []

  for event in LogReader(str(path), only_union_types=True):
    if event.which() != "can":
      continue

    if first_can_time is None:
      first_can_time = event.logMonoTime
    last_can_time = event.logMonoTime

    frames = []
    for can_msg in event.can:
      dat = bytes(can_msg.dat)
      frames.append((can_msg.address, dat, can_msg.src))
      if can_msg.src == bus and can_msg.address in CANDIDATE_MESSAGES.values():
        raw_counts[can_msg.address][dat.hex()] += 1

    cp.update((event.logMonoTime, frames))
    can_samples += 1

    speed_mps = cp.vl["ENGINE_DATA"]["SPEED"] * (1000.0 / 3600.0)
    cruise_speed_mps = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * (1000.0 / 3600.0)

    speeds.append(speed_mps)
    cruise_speeds.append(cruise_speed_mps)

    available_samples += int(cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1.0)
    engaged_samples += int(cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1.0)
    lead_samples += int(cp.vl["CRZ_CTRL"]["RADAR_HAS_LEAD"] == 1.0)
    standstill_samples += int(cp.vl["PEDALS"]["STANDSTILL"] == 1.0)
    gas_pressed_samples += int(cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0.0)
    brake_pressed_samples += int(cp.vl["PEDALS"]["BRAKE_ON"] == 1.0)

    for msg_name, sig_name in FOCUSED_FIELDS:
      field_values[f"{msg_name}.{sig_name}"].append(cp.vl[msg_name][sig_name])

  if can_samples == 0:
    raise RuntimeError(f"No CAN samples found in {path}")

  duration_s = (last_can_time - first_can_time) * 1e-9 if first_can_time is not None and last_can_time is not None else 0.0
  return LogData(
    path=path,
    duration_s=duration_s,
    can_samples=can_samples,
    speed_min=min(speeds),
    speed_max=max(speeds),
    cruise_speed_min=min(cruise_speeds),
    cruise_speed_max=max(cruise_speeds),
    available_samples=available_samples,
    engaged_samples=engaged_samples,
    lead_samples=lead_samples,
    standstill_samples=standstill_samples,
    gas_pressed_samples=gas_pressed_samples,
    brake_pressed_samples=brake_pressed_samples,
    field_values=field_values,
    raw_counts=dict(raw_counts),
  )


def top_values(values: list[float], count: int = 5) -> list[tuple[float, int]]:
  return Counter(values).most_common(count)


def format_common(values: list[tuple[float, int]]) -> str:
  return ", ".join(f"{value}:{count}" for value, count in values)


def mode_bytes(raw_counter: Counter[str]) -> list[int] | None:
  if not raw_counter:
    return None

  byte_counters = [Counter() for _ in range(8)]
  for raw_hex, count in raw_counter.items():
    raw_bytes = bytes.fromhex(raw_hex)
    if len(raw_bytes) != 8:
      continue
    for i, raw_byte in enumerate(raw_bytes):
      byte_counters[i][raw_byte] += count

  return [byte_counter.most_common(1)[0][0] for byte_counter in byte_counters]


def format_mode_bytes(mode: list[int] | None) -> str:
  if mode is None:
    return "none"
  return " ".join(f"{value:02x}" for value in mode)


def differing_byte_positions(a: list[int] | None, b: list[int] | None) -> list[int]:
  if a is None or b is None:
    return []
  return [i for i, (lhs, rhs) in enumerate(zip(a, b, strict=True)) if lhs != rhs]


def print_log_summary(log: LogData) -> None:
  print(f"{log.path.name}: duration={log.duration_s:.2f}s samples={log.can_samples} speed={log.speed_min:.2f}..{log.speed_max:.2f} m/s cruise={log.cruise_speed_min:.2f}..{log.cruise_speed_max:.2f} m/s")
  print(
    f"  available={log.available_samples}/{log.can_samples} engaged={log.engaged_samples}/{log.can_samples} "
    f"lead={log.lead_samples}/{log.can_samples} standstill={log.standstill_samples}/{log.can_samples} "
    f"gas_pressed={log.gas_pressed_samples}/{log.can_samples} brake_pressed={log.brake_pressed_samples}/{log.can_samples}"
  )


def print_signal_compare(lhs: LogData, rhs: LogData) -> None:
  print(f"\nSignal comparison: {lhs.path.name} vs {rhs.path.name}")
  print("  field                            lhs median   rhs median   lhs min..max         rhs min..max         lhs top values             rhs top values")
  for msg_name, sig_name in FOCUSED_FIELDS:
    field = f"{msg_name}.{sig_name}"
    lhs_values = lhs.field_values[field]
    rhs_values = rhs.field_values[field]
    print(
      f"  {field:30} {median(lhs_values):10.3f} {median(rhs_values):10.3f} "
      f"{min(lhs_values):8.3f}..{max(lhs_values):8.3f}  {min(rhs_values):8.3f}..{max(rhs_values):8.3f}  "
      f"{format_common(top_values(lhs_values, 3)):24} {format_common(top_values(rhs_values, 3))}"
    )


def print_raw_compare(lhs: LogData, rhs: LogData, top_raw: int) -> None:
  print(f"\nRaw payload comparison: {lhs.path.name} vs {rhs.path.name}")
  for msg_name, address in CANDIDATE_MESSAGES.items():
    lhs_mode = mode_bytes(lhs.raw_counts.get(address, Counter()))
    rhs_mode = mode_bytes(rhs.raw_counts.get(address, Counter()))
    print(f"  {msg_name} {hex(address)}")
    print(f"    lhs mode bytes: {format_mode_bytes(lhs_mode)}")
    print(f"    rhs mode bytes: {format_mode_bytes(rhs_mode)}")
    print(f"    differing byte positions: {differing_byte_positions(lhs_mode, rhs_mode)}")
    print(f"    lhs top payloads: {lhs.raw_counts.get(address, Counter()).most_common(top_raw)}")
    print(f"    rhs top payloads: {rhs.raw_counts.get(address, Counter()).most_common(top_raw)}")


def main() -> None:
  args = parse_args()
  logs = [extract_log(path, args.bus) for path in args.rlogs]

  print("Per-log summaries:")
  for log in logs:
    print_log_summary(log)

  for lhs, rhs in zip(logs, logs[1:], strict=False):
    print_signal_compare(lhs, rhs)
    print_raw_compare(lhs, rhs, args.top_raw)


if __name__ == "__main__":
  main()
