#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from heapq import nlargest, nsmallest
from math import sqrt
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

SKIP_SIGNAL_PARTS: tuple[str, ...] = (
  "CHK",
  "CTR",
  "INV",
  "NEW_SIGNAL",
  "STATIC",
  "MSG_",
)


@dataclass(frozen=True)
class RankedSignal:
  field: str
  unique_values: int
  min_value: float
  max_value: float
  speed_rate_lag: int
  speed_rate_corr: float
  veh_acc_lag: int
  veh_acc_corr: float

  @property
  def score(self) -> float:
    return max(abs(self.speed_rate_corr), abs(self.veh_acc_corr))


@dataclass(frozen=True)
class Snapshot:
  t_s: float
  v_ego: float
  speed_rate: float
  veh_acc_x: float
  brake_pressure: float
  info_accel_cmd: float
  events_accel_cmd: float
  events_accel_low_res: float
  gas_cmd: float
  raw_21b: str
  raw_21c: str
  raw_21f: str
  raw_fd: str
  raw_167: str


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Analyze Mazda ACC candidate signals from an rlog.zst file.")
  parser.add_argument("rlog", type=Path, help="Path to the Mazda rlog (.zst)")
  parser.add_argument("--bus", type=int, default=0, help="CAN src/bus to decode (default: 0)")
  parser.add_argument("--top", type=int, default=12, help="Number of ranked candidate signals to print")
  parser.add_argument("--snapshots", type=int, default=5, help="Number of accel/decel snapshots to print")
  parser.add_argument("--max-lag", type=int, default=15, help="Max sample lag when scoring signal correlation")
  parser.add_argument("--min-unique", type=int, default=8, help="Minimum unique values for a signal to be ranked")
  return parser.parse_args()


def should_rank_signal(signal_name: str) -> bool:
  return not any(part in signal_name for part in SKIP_SIGNAL_PARTS)


def corr(xs: list[float], ys: list[float]) -> float:
  x_mean = sum(xs) / len(xs)
  y_mean = sum(ys) / len(ys)

  numerator = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys, strict=True))
  x_den = sqrt(sum((x - x_mean) ** 2 for x in xs))
  y_den = sqrt(sum((y - y_mean) ** 2 for y in ys))

  return numerator / (x_den * y_den) if x_den and y_den else 0.0


def best_lag_corr(xs: list[float], ys: list[float], max_lag: int) -> tuple[int, float]:
  best_lag = 0
  best_corr = corr(xs, ys)

  for lag in range(-max_lag, max_lag + 1):
    if lag < 0:
      lagged_xs = xs[-lag:]
      lagged_ys = ys[:len(lagged_xs)]
    elif lag > 0:
      lagged_xs = xs[:-lag]
      lagged_ys = ys[lag:]
    else:
      lagged_xs = xs
      lagged_ys = ys

    if len(lagged_xs) < 3:
      continue

    lag_corr = corr(lagged_xs, lagged_ys)
    if abs(lag_corr) > abs(best_corr):
      best_lag = lag
      best_corr = lag_corr

  return best_lag, best_corr


def format_corr(lag: int, value: float) -> str:
  return f"{value:+.3f}@{lag:+d}"


def build_signal_list(cp: CANParser) -> list[tuple[str, str, str]]:
  signals = []
  for msg_name in CANDIDATE_MESSAGES:
    for sig_name in cp.vl[msg_name]:
      if should_rank_signal(sig_name):
        signals.append((msg_name, sig_name, f"{msg_name}.{sig_name}"))
  return signals


def analyze_rlog(args: argparse.Namespace) -> tuple[dict[str, object], list[RankedSignal], list[Snapshot], Counter[tuple[int, int]]]:
  if not args.rlog.is_file():
    raise FileNotFoundError(args.rlog)

  messages = [(msg_name, 50) for msg_name in CANDIDATE_MESSAGES] + SUPPORT_MESSAGES
  cp = CANParser("mazda_2017", messages, args.bus)
  candidate_signals = build_signal_list(cp)

  signal_samples: dict[str, list[float]] = {field: [] for _, _, field in candidate_signals}
  candidate_counts: Counter[tuple[int, int]] = Counter()
  last_raw: dict[int, bytes] = {}
  snapshots: list[Snapshot] = []

  first_log_time = None
  last_log_time = None
  first_can_time = None
  last_can_time = None
  car_params = None
  can_samples = 0
  engaged_samples = 0
  moving_samples = 0
  gas_pressed_samples = 0
  brake_pressed_samples = 0

  v_ego_samples: list[float] = []
  speed_rate_samples: list[float] = []
  veh_acc_x_samples: list[float] = []
  cruise_speed_samples: list[float] = []
  engaged_mask: list[bool] = []
  prev_t_s = None
  prev_v_ego = None

  for event in LogReader(str(args.rlog), only_union_types=True):
    if first_log_time is None:
      first_log_time = event.logMonoTime
    last_log_time = event.logMonoTime

    which = event.which()
    if which == "carParams" and car_params is None:
      car_params = event.carParams
      continue

    if which != "can":
      continue

    if first_can_time is None:
      first_can_time = event.logMonoTime
    last_can_time = event.logMonoTime

    frames = []
    for can_msg in event.can:
      dat = bytes(can_msg.dat)
      frames.append((can_msg.address, dat, can_msg.src))
      if can_msg.address in CANDIDATE_MESSAGES.values():
        candidate_counts[(can_msg.address, can_msg.src)] += 1
      if can_msg.src == args.bus and can_msg.address in CANDIDATE_MESSAGES.values():
        last_raw[can_msg.address] = dat

    cp.update((event.logMonoTime, frames))
    can_samples += 1

    t_s = event.logMonoTime * 1e-9
    v_ego = cp.vl["ENGINE_DATA"]["SPEED"] * (1000.0 / 3600.0)
    if prev_t_s is None or prev_v_ego is None or t_s <= prev_t_s:
      speed_rate = 0.0
    else:
      speed_rate = (v_ego - prev_v_ego) / (t_s - prev_t_s)
    prev_t_s = t_s
    prev_v_ego = v_ego

    cruise_enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1.0
    veh_acc_x = cp.vl["BRAKE"]["VEHICLE_ACC_X"]
    gas_pressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0.0
    brake_pressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1.0

    engaged_samples += int(cruise_enabled)
    moving_samples += int(v_ego > 1.0)
    gas_pressed_samples += int(gas_pressed)
    brake_pressed_samples += int(brake_pressed)

    v_ego_samples.append(v_ego)
    speed_rate_samples.append(speed_rate)
    veh_acc_x_samples.append(veh_acc_x)
    cruise_speed_samples.append(cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * (1000.0 / 3600.0))
    engaged_mask.append(cruise_enabled)

    for msg_name, sig_name, field in candidate_signals:
      signal_samples[field].append(cp.vl[msg_name][sig_name])

    snapshots.append(Snapshot(
      t_s=t_s,
      v_ego=v_ego,
      speed_rate=speed_rate,
      veh_acc_x=veh_acc_x,
      brake_pressure=cp.vl["BRAKE"]["BRAKE_PRESSURE"],
      info_accel_cmd=cp.vl["CRZ_INFO"]["ACCEL_CMD"],
      events_accel_cmd=cp.vl["CRZ_EVENTS"]["ACCEL_CMD"],
      events_accel_low_res=cp.vl["CRZ_EVENTS"]["ACCEL_CMD_LOW_RES"],
      gas_cmd=cp.vl["GAS"]["GAS_CMD"],
      raw_21b=last_raw.get(0x21B, b"").hex(),
      raw_21c=last_raw.get(0x21C, b"").hex(),
      raw_21f=last_raw.get(0x21F, b"").hex(),
      raw_fd=last_raw.get(0x0FD, b"").hex(),
      raw_167=last_raw.get(0x167, b"").hex(),
    ))

  if can_samples == 0:
    raise RuntimeError("No CAN messages found in the log.")

  active_indices = [i for i, engaged in enumerate(engaged_mask) if engaged and v_ego_samples[i] > 1.0]
  if not active_indices:
    active_indices = [i for i, v_ego in enumerate(v_ego_samples) if v_ego > 1.0]

  if not active_indices:
    raise RuntimeError("No moving samples found in the log.")

  active_speed_rate = [speed_rate_samples[i] for i in active_indices]
  active_veh_acc_x = [veh_acc_x_samples[i] for i in active_indices]

  ranked: list[RankedSignal] = []
  for field, values in signal_samples.items():
    active_values = [values[i] for i in active_indices]
    unique_values = len(set(active_values))
    if unique_values < args.min_unique:
      continue

    speed_rate_lag, speed_rate_corr = best_lag_corr(active_values, active_speed_rate, args.max_lag)
    veh_acc_lag, veh_acc_corr = best_lag_corr(active_values, active_veh_acc_x, args.max_lag)

    ranked.append(RankedSignal(
      field=field,
      unique_values=unique_values,
      min_value=min(active_values),
      max_value=max(active_values),
      speed_rate_lag=speed_rate_lag,
      speed_rate_corr=speed_rate_corr,
      veh_acc_lag=veh_acc_lag,
      veh_acc_corr=veh_acc_corr,
    ))

  ranked.sort(key=lambda row: (-row.score, row.field))

  summary = {
    "car_params": car_params,
    "can_samples": can_samples,
    "engaged_samples": engaged_samples,
    "moving_samples": moving_samples,
    "gas_pressed_samples": gas_pressed_samples,
    "brake_pressed_samples": brake_pressed_samples,
    "duration_s": (last_can_time - first_can_time) * 1e-9 if first_can_time is not None and last_can_time is not None else 0.0,
    "v_ego_min": min(v_ego_samples[i] for i in active_indices),
    "v_ego_max": max(v_ego_samples[i] for i in active_indices),
    "cruise_speed_min": min(cruise_speed_samples[i] for i in active_indices),
    "cruise_speed_max": max(cruise_speed_samples[i] for i in active_indices),
  }

  return summary, ranked, snapshots, candidate_counts


def print_summary(args: argparse.Namespace, summary: dict[str, object], ranked: list[RankedSignal], snapshots: list[Snapshot],
                  candidate_counts: Counter[tuple[int, int]]) -> None:
  car_params = summary["car_params"]

  print(f"Route: {args.rlog}")
  if car_params is not None:
    print(f"Fingerprint: {car_params.carFingerprint}  brand={car_params.brand}  pcmCruise={car_params.pcmCruise}  openpilotLong={car_params.openpilotLongitudinalControl}")
  print(
    f"Duration: {summary['duration_s']:.2f}s  can_samples={summary['can_samples']}  "
    f"moving_samples={summary['moving_samples']}  engaged_samples={summary['engaged_samples']}  "
    f"gas_pressed_samples={summary['gas_pressed_samples']}  brake_pressed_samples={summary['brake_pressed_samples']}"
  )
  print(
    f"Active vEgo range: {summary['v_ego_min']:.3f}..{summary['v_ego_max']:.3f} m/s  "
    f"cruise_speed range: {summary['cruise_speed_min']:.3f}..{summary['cruise_speed_max']:.3f} m/s"
  )

  print("\nCandidate address counts by src:")
  for (address, src), count in sorted(candidate_counts.items()):
    print(f"  {hex(address)} src={src:<3} count={count}")

  print("\nTop candidate signals:")
  print("  signal                         uniq      min      max   speedRate corr   vehAccX corr")
  for row in ranked[:args.top]:
    print(
      f"  {row.field:28} {row.unique_values:5d} {row.min_value:8.3f} {row.max_value:8.3f} "
      f"{format_corr(row.speed_rate_lag, row.speed_rate_corr):>16} {format_corr(row.veh_acc_lag, row.veh_acc_corr):>14}"
    )

  core_lookup = {row.field: row for row in ranked}
  if all(field in core_lookup for field in ("CRZ_INFO.ACCEL_CMD", "CRZ_EVENTS.ACCEL_CMD")):
    diffs = [snapshot.info_accel_cmd - 8 * snapshot.events_accel_cmd for snapshot in snapshots]
    print(f"\nCRZ_INFO.ACCEL_CMD - 8*CRZ_EVENTS.ACCEL_CMD median: {median(diffs):.3f}")

  print("\nTop accel snapshots:")
  for snapshot in nlargest(args.snapshots, snapshots, key=lambda row: row.veh_acc_x):
    print(
      f"  t={snapshot.t_s:.3f}s vEgo={snapshot.v_ego:.3f} speedRate={snapshot.speed_rate:.3f} vehAccX={snapshot.veh_acc_x:.3f} "
      f"info={snapshot.info_accel_cmd:.1f} events={snapshot.events_accel_cmd:.1f} "
      f"eventsLow={snapshot.events_accel_low_res:.1f} gas={snapshot.gas_cmd:.1f}"
    )
    print(
      f"    0x21b={snapshot.raw_21b} 0x21c={snapshot.raw_21c} 0x21f={snapshot.raw_21f} "
      f"0xfd={snapshot.raw_fd} 0x167={snapshot.raw_167}"
    )

  print("\nTop decel snapshots:")
  for snapshot in nsmallest(args.snapshots, snapshots, key=lambda row: row.veh_acc_x):
    print(
      f"  t={snapshot.t_s:.3f}s vEgo={snapshot.v_ego:.3f} speedRate={snapshot.speed_rate:.3f} vehAccX={snapshot.veh_acc_x:.3f} "
      f"info={snapshot.info_accel_cmd:.1f} events={snapshot.events_accel_cmd:.1f} "
      f"eventsLow={snapshot.events_accel_low_res:.1f} gas={snapshot.gas_cmd:.1f}"
    )
    print(
      f"    0x21b={snapshot.raw_21b} 0x21c={snapshot.raw_21c} 0x21f={snapshot.raw_21f} "
      f"0xfd={snapshot.raw_fd} 0x167={snapshot.raw_167}"
    )


def main() -> None:
  args = parse_args()
  summary, ranked, snapshots, candidate_counts = analyze_rlog(args)
  print_summary(args, summary, ranked, snapshots, candidate_counts)


if __name__ == "__main__":
  main()
