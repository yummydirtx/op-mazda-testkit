#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from opendbc.can.parser import CANParser
from opendbc.car.logreader import LogReader


@dataclass(frozen=True)
class Sample:
  t_s: float
  speed_kph: float
  veh_acc_x: float
  brake_pressure: float
  info_accel_cmd: float
  events_accel_cmd: float
  events_accel_low_res: float
  traction_brake: int
  standstill: int


@dataclass(frozen=True)
class Window:
  start: Sample
  end: Sample
  kind: str


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Find stock Mazda ACC windows where braking persists after 0x21b/0x21f have relaxed.",
  )
  parser.add_argument("rlogs", nargs="+", type=Path, help="Path(s) to rlog.zst files")
  parser.add_argument("--bus", type=int, default=0, help="CAN src/bus to analyze")
  parser.add_argument("--decel-threshold", type=float, default=-0.15,
                      help="Treat VEHICLE_ACC_X at or below this value as real braking")
  parser.add_argument("--info-threshold", type=float, default=100.0,
                      help="Treat |CRZ_INFO.ACCEL_CMD| at or below this as relaxed")
  parser.add_argument("--events-threshold", type=float, default=20.0,
                      help="Treat |CRZ_EVENTS.ACCEL_CMD| at or below this as relaxed")
  parser.add_argument("--hold-info-threshold", type=float, default=50.0,
                      help="Treat |CRZ_INFO.ACCEL_CMD| at or below this as hold-latched / relaxed at standstill")
  parser.add_argument("--min-window-seconds", type=float, default=0.05,
                      help="Only print windows at least this long")
  return parser.parse_args()


def build_samples(path: Path, bus: int) -> list[Sample]:
  cp = CANParser(
    "mazda_2017",
    [
      ("ENGINE_DATA", 100),
      ("BRAKE", 50),
      ("PEDALS", 50),
      ("CRZ_INFO", 50),
      ("CRZ_CTRL", 50),
      ("CRZ_EVENTS", 50),
      ("TRACTION", 20),
    ],
    bus,
  )
  first_t_s: float | None = None
  out: list[Sample] = []
  for event in LogReader(str(path), only_union_types=True):
    if event.which() != "can":
      continue
    frames = [(m.address, bytes(m.dat), m.src) for m in event.can if m.src == bus]
    if not frames:
      continue
    t_s = event.logMonoTime * 1e-9
    if first_t_s is None:
      first_t_s = t_s
    rel_t_s = t_s - first_t_s
    cp.update([(event.logMonoTime, frames)])
    if int(cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"]) != 1:
      continue
    out.append(Sample(
      t_s=rel_t_s,
      speed_kph=cp.vl["ENGINE_DATA"]["SPEED"],
      veh_acc_x=cp.vl["BRAKE"]["VEHICLE_ACC_X"],
      brake_pressure=cp.vl["BRAKE"]["BRAKE_PRESSURE"],
      info_accel_cmd=cp.vl["CRZ_INFO"]["ACCEL_CMD"],
      events_accel_cmd=cp.vl["CRZ_EVENTS"]["ACCEL_CMD"],
      events_accel_low_res=cp.vl["CRZ_EVENTS"]["ACCEL_CMD_LOW_RES"],
      traction_brake=int(cp.vl["TRACTION"]["BRAKE"]),
      standstill=int(cp.vl["PEDALS"]["STANDSTILL"]),
    ))
  return out


def collect_windows(samples: list[Sample], kind: str, predicate, min_window_seconds: float) -> list[Window]:
  windows: list[Window] = []
  start_idx: int | None = None
  for idx, sample in enumerate(samples):
    if predicate(sample):
      if start_idx is None:
        start_idx = idx
    elif start_idx is not None:
      start = samples[start_idx]
      end = samples[idx - 1]
      if end.t_s - start.t_s >= min_window_seconds:
        windows.append(Window(start=start, end=end, kind=kind))
      start_idx = None
  if start_idx is not None:
    start = samples[start_idx]
    end = samples[-1]
    if end.t_s - start.t_s >= min_window_seconds:
      windows.append(Window(start=start, end=end, kind=kind))
  return windows


def print_window(window: Window) -> None:
  start = window.start
  end = window.end
  print(
    f"  {window.kind}: {start.t_s:.3f}s -> {end.t_s:.3f}s "
    f"({end.t_s - start.t_s:.3f}s)"
  )
  print(
    f"    start speed={start.speed_kph:.2f}kph acc_x={start.veh_acc_x:.2f} pressure={start.brake_pressure:.0f} "
    f"21b={start.info_accel_cmd:.0f} 21f={start.events_accel_cmd:.0f} low={start.events_accel_low_res:.0f} "
    f"tr_brake={start.traction_brake} stand={start.standstill}"
  )
  print(
    f"    end   speed={end.speed_kph:.2f}kph acc_x={end.veh_acc_x:.2f} pressure={end.brake_pressure:.0f} "
    f"21b={end.info_accel_cmd:.0f} 21f={end.events_accel_cmd:.0f} low={end.events_accel_low_res:.0f} "
    f"tr_brake={end.traction_brake} stand={end.standstill}"
  )


def main() -> None:
  args = parse_args()
  for path in args.rlogs:
    samples = build_samples(path, args.bus)
    decel_windows = collect_windows(
      samples,
      "decel_with_relaxed_cmd",
      lambda s: (
        s.traction_brake == 1
        and s.veh_acc_x <= args.decel_threshold
        and abs(s.info_accel_cmd) <= args.info_threshold
        and abs(s.events_accel_cmd) <= args.events_threshold
      ),
      args.min_window_seconds,
    )
    hold_windows = collect_windows(
      samples,
      "standstill_with_relaxed_21b",
      lambda s: (
        s.traction_brake == 1
        and s.standstill == 1
        and abs(s.info_accel_cmd) <= args.hold_info_threshold
      ),
      args.min_window_seconds,
    )

    print(f"{path.name}: active_samples={len(samples)}")
    print(f"  decel_with_relaxed_cmd windows={len(decel_windows)}")
    for window in decel_windows[:8]:
      print_window(window)
    if len(decel_windows) > 8:
      print(f"  ... {len(decel_windows) - 8} more decel_with_relaxed_cmd windows")

    print(f"  standstill_with_relaxed_21b windows={len(hold_windows)}")
    for window in hold_windows[:8]:
      print_window(window)
    if len(hold_windows) > 8:
      print(f"  ... {len(hold_windows) - 8} more standstill_with_relaxed_21b windows")


if __name__ == "__main__":
  main()
