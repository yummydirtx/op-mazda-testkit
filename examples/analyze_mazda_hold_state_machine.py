#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path

from opendbc.can.parser import CANParser
from opendbc.car.logreader import LogReader


ZERO_THRESHOLD_KPH = 0.05


@dataclass(frozen=True)
class StatePoint:
  t_s: float
  speed_kph: float
  standstill: int
  brake_on: int
  traction_brake: int
  check_temp_standstill: int
  veh_acc_x: float
  crz_info_accel_cmd: float
  crz_events_accel_cmd: float
  crz_events_accel_low_res: float
  crz_speed_kph: float
  pedal_gas: float
  res: int
  set_plus: int
  set_minus: int
  raw_21b: str | None
  raw_21c: str | None
  raw_21f: str | None


@dataclass(frozen=True)
class HoldEvent:
  index: int
  rise: StatePoint
  first_zero: StatePoint | None
  drop: StatePoint | None
  traction_brake_drop: StatePoint | None
  crz_ctrl_states: tuple[tuple[float, str], ...]


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Analyze Mazda stop/HOLD state-machine events from one or more rlogs.",
  )
  parser.add_argument("rlogs", nargs="+", type=Path, help="Path(s) to rlog.zst files")
  parser.add_argument("--bus", type=int, default=0, help="CAN src/bus to analyze")
  parser.add_argument("--min-moving-kph", type=float, default=1.0,
                      help="Require prior speed above this threshold before a STANDSTILL rise counts as a hold event")
  parser.add_argument("--zero-threshold-kph", type=float, default=ZERO_THRESHOLD_KPH,
                      help="Treat speed at or below this threshold as zero")
  parser.add_argument("--json-output", type=Path,
                      help="Optional JSON artifact path for machine-readable event summaries")
  return parser.parse_args()


def point_from_parser(t_s: float, parser: CANParser, raw_21b: str | None, raw_21c: str | None, raw_21f: str | None) -> StatePoint:
  return StatePoint(
    t_s=t_s,
    speed_kph=parser.vl["ENGINE_DATA"]["SPEED"],
    standstill=int(parser.vl["PEDALS"]["STANDSTILL"]),
    brake_on=int(parser.vl["PEDALS"]["BRAKE_ON"]),
    traction_brake=int(parser.vl["TRACTION"]["BRAKE"]),
    check_temp_standstill=int(parser.vl["CHECK_AND_TEMP"]["STANDSTILL"]),
    veh_acc_x=parser.vl["BRAKE"]["VEHICLE_ACC_X"],
    crz_info_accel_cmd=parser.vl["CRZ_INFO"]["ACCEL_CMD"],
    crz_events_accel_cmd=parser.vl["CRZ_EVENTS"]["ACCEL_CMD"],
    crz_events_accel_low_res=parser.vl["CRZ_EVENTS"]["ACCEL_CMD_LOW_RES"],
    crz_speed_kph=parser.vl["CRZ_EVENTS"]["CRZ_SPEED"],
    pedal_gas=parser.vl["ENGINE_DATA"]["PEDAL_GAS"],
    res=int(parser.vl["CRZ_BTNS"]["RES"]),
    set_plus=int(parser.vl["CRZ_BTNS"]["SET_P"]),
    set_minus=int(parser.vl["CRZ_BTNS"]["SET_M"]),
    raw_21b=raw_21b,
    raw_21c=raw_21c,
    raw_21f=raw_21f,
  )


def load_points(path: Path, bus: int, zero_threshold_kph: float) -> list[StatePoint]:
  parser = CANParser(
    "mazda_2017",
    [
      ("ENGINE_DATA", 100),
      ("PEDALS", 50),
      ("BRAKE", 50),
      ("CRZ_INFO", 50),
      ("CRZ_CTRL", 50),
      ("CRZ_EVENTS", 50),
      ("CRZ_BTNS", 50),
      ("TRACTION", 20),
      ("CHECK_AND_TEMP", 10),
    ],
    bus,
  )
  first_t_s: float | None = None
  raw_21b: str | None = None
  raw_21c: str | None = None
  raw_21f: str | None = None
  prev_watch: tuple[object, ...] | None = None
  prev_is_zero: bool | None = None
  points: list[StatePoint] = []

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

    parser.update([(event.logMonoTime, frames)])
    for addr, dat, _src in frames:
      if addr == 0x21B:
        raw_21b = dat.hex()
      elif addr == 0x21C:
        raw_21c = dat.hex()
      elif addr == 0x21F:
        raw_21f = dat.hex()

    point = point_from_parser(rel_t_s, parser, raw_21b, raw_21c, raw_21f)
    watch = (
      point.standstill,
      point.brake_on,
      point.traction_brake,
      point.check_temp_standstill,
      point.raw_21b,
      point.raw_21c,
      point.raw_21f,
      point.res,
      point.set_plus,
      point.set_minus,
    )
    is_zero = point.speed_kph <= zero_threshold_kph
    crossed_zero = prev_is_zero is None or is_zero != prev_is_zero
    if prev_watch is None or watch != prev_watch or crossed_zero:
      points.append(point)
      prev_watch = watch
      prev_is_zero = is_zero

  return points


def build_hold_events(points: list[StatePoint], min_moving_kph: float, zero_threshold_kph: float) -> list[HoldEvent]:
  events: list[HoldEvent] = []
  recent_speed = deque(maxlen=200)
  event_rise_index: int | None = None
  prev_point: StatePoint | None = None
  seen_21c: list[tuple[float, str]] = []
  first_zero_index: int | None = None
  traction_brake_drop_index: int | None = None
  moving_before_rise = False

  for idx, point in enumerate(points):
    recent_speed.append(point.speed_kph)
    if prev_point is not None:
      if event_rise_index is None:
        moving_before_rise = moving_before_rise or any(v > min_moving_kph for v in recent_speed)
        if prev_point.standstill == 0 and point.standstill == 1 and moving_before_rise:
          event_rise_index = idx
          seen_21c = []
          first_zero_index = idx if point.speed_kph <= zero_threshold_kph else None
          traction_brake_drop_index = None
          if point.raw_21c is not None:
            seen_21c.append((point.t_s, point.raw_21c))
      else:
        if point.raw_21c is not None and (not seen_21c or seen_21c[-1][1] != point.raw_21c):
          seen_21c.append((point.t_s, point.raw_21c))
        if first_zero_index is None and point.speed_kph <= zero_threshold_kph:
          first_zero_index = idx
        if traction_brake_drop_index is None and prev_point.traction_brake == 1 and point.traction_brake == 0:
          traction_brake_drop_index = idx
        if prev_point.standstill == 1 and point.standstill == 0:
          rise = points[event_rise_index]
          events.append(
            HoldEvent(
              index=len(events),
              rise=rise,
              first_zero=points[first_zero_index] if first_zero_index is not None else None,
              drop=point,
              traction_brake_drop=points[traction_brake_drop_index] if traction_brake_drop_index is not None else None,
              crz_ctrl_states=tuple((t - rise.t_s, raw) for t, raw in seen_21c),
            )
          )
          event_rise_index = None
          seen_21c = []
          first_zero_index = None
          traction_brake_drop_index = None
          moving_before_rise = False
          recent_speed.clear()
    prev_point = point

  if event_rise_index is not None:
    rise = points[event_rise_index]
    events.append(
      HoldEvent(
        index=len(events),
        rise=rise,
        first_zero=points[first_zero_index] if first_zero_index is not None else None,
        drop=None,
        traction_brake_drop=points[traction_brake_drop_index] if traction_brake_drop_index is not None else None,
        crz_ctrl_states=tuple((t - rise.t_s, raw) for t, raw in seen_21c),
      )
    )

  return events


def summarize_point(label: str, point: StatePoint | None) -> str:
  if point is None:
    return f"  {label}: none"
  return (
    f"  {label}: t={point.t_s:.3f}s speed={point.speed_kph:.2f}kph "
    f"stand={point.standstill} brake_on={point.brake_on} tr_brake={point.traction_brake} "
    f"chk_temp_stand={point.check_temp_standstill} acc_x={point.veh_acc_x:.2f} "
    f"21b_acc={point.crz_info_accel_cmd:.0f} 21f_acc={point.crz_events_accel_cmd:.0f} "
    f"21f_low={point.crz_events_accel_low_res:.0f} crz_speed={point.crz_speed_kph:.2f}"
  )


def print_event(event: HoldEvent) -> None:
  print(f"  hold_event {event.index}:")
  print(summarize_point("stand_rise", event.rise))
  print(summarize_point("first_zero", event.first_zero))
  print(summarize_point("stand_drop", event.drop))
  print(summarize_point("traction_brake_drop", event.traction_brake_drop))
  if event.first_zero is not None:
    print(f"    delta rise->zero: {event.first_zero.t_s - event.rise.t_s:+.3f}s")
  if event.drop is not None:
    print(f"    delta rise->drop: {event.drop.t_s - event.rise.t_s:+.3f}s")
  if event.drop is not None and event.traction_brake_drop is not None:
    print(f"    delta stand_drop->tr_brake_drop: {event.traction_brake_drop.t_s - event.drop.t_s:+.3f}s")
  if event.crz_ctrl_states:
    print("    0x21c states:")
    for rel_t_s, raw in event.crz_ctrl_states:
      print(f"      t={rel_t_s:+.3f}s raw21c={raw}")


def main() -> None:
  args = parse_args()
  json_payload: dict[str, object] = {}

  for path in args.rlogs:
    points = load_points(path, args.bus, args.zero_threshold_kph)
    events = build_hold_events(points, args.min_moving_kph, args.zero_threshold_kph)

    print(f"{path.name}: points={len(points)} hold_events={len(events)}")
    for event in events:
      print_event(event)

    json_payload[str(path)] = {
      "points": len(points),
      "hold_events": [asdict(event) for event in events],
    }

  if args.json_output is not None:
    args.json_output.write_text(json.dumps(json_payload, indent=2))
    print(f"\nWrote JSON summary to {args.json_output}")


if __name__ == "__main__":
  main()
