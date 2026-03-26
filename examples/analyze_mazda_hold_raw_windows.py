#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path

from opendbc.can.parser import CANParser
from opendbc.car.logreader import LogReader


DEFAULT_ADDRS = (0x165, 0x415, 0x436, 0x21C, 0x121, 0x078, 0x079, 0x45B, 0x43D, 0x420)


@dataclass(frozen=True)
class Anchor:
  kind: str
  t_s: float
  speed_kph: float
  standstill: int
  traction_brake: int
  brake_on: int
  active: int


@dataclass(frozen=True)
class Transition:
  dt_s: float
  t_s: float
  address: int
  data_hex: str
  speed_kph: float
  standstill: int
  traction_brake: int
  brake_on: int
  active: int


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Print raw Mazda CAN transitions in a window around a HOLD/standstill anchor.",
  )
  parser.add_argument("rlogs", nargs="+", type=Path, help="Path(s) to rlog.zst files")
  parser.add_argument("--bus", type=int, default=0, help="CAN src/bus to analyze")
  parser.add_argument("--addrs", default=",".join(f"0x{addr:x}" for addr in DEFAULT_ADDRS),
                      help="Comma-separated addresses to track")
  parser.add_argument("--anchor", choices=("stand_rise", "first_zero"), default="stand_rise",
                      help="Anchor the window on the first standstill rise after movement, or the first zero speed after that movement")
  parser.add_argument("--min-moving-kph", type=float, default=0.5,
                      help="Require prior speed above this threshold before a standstill event counts")
  parser.add_argument("--zero-threshold-kph", type=float, default=0.05,
                      help="Treat speed at or below this threshold as zero")
  parser.add_argument("--pre-seconds", type=float, default=0.25,
                      help="Seconds before the anchor to include")
  parser.add_argument("--post-seconds", type=float, default=2.0,
                      help="Seconds after the anchor to include")
  parser.add_argument("--event-index", type=int, default=0,
                      help="Which detected hold event to analyze (0-based)")
  parser.add_argument("--json-output", type=Path,
                      help="Optional JSON artifact path")
  return parser.parse_args()


def parse_addrs(raw: str) -> tuple[int, ...]:
  out: list[int] = []
  for token in raw.split(","):
    token = token.strip()
    if not token:
      continue
    out.append(int(token, 0))
  return tuple(out)


def build_parser(bus: int) -> CANParser:
  return CANParser(
    "mazda_2017",
    [
      ("ENGINE_DATA", 100),
      ("PEDALS", 50),
      ("TRACTION", 20),
      ("CRZ_CTRL", 50),
      ("CRZ_EVENTS", 50),
      ("BRAKE", 50),
    ],
    bus,
  )


def speed_point(cp: CANParser, t_s: float) -> Anchor:
  return Anchor(
    kind="point",
    t_s=t_s,
    speed_kph=cp.vl["ENGINE_DATA"]["SPEED"],
    standstill=int(cp.vl["PEDALS"]["STANDSTILL"]),
    traction_brake=int(cp.vl["TRACTION"]["BRAKE"]),
    brake_on=int(cp.vl["PEDALS"]["BRAKE_ON"]),
    active=int(cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"]),
  )


def find_anchor(path: Path, bus: int, anchor_kind: str, min_moving_kph: float, zero_threshold_kph: float, event_index: int) -> Anchor:
  cp = build_parser(bus)
  first_t_s: float | None = None
  recent_speed = deque(maxlen=400)
  prev_standstill: int | None = None
  pending_rise: Anchor | None = None
  seen_events = -1

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
    point = speed_point(cp, rel_t_s)
    recent_speed.append(point.speed_kph)

    if prev_standstill is None:
      prev_standstill = point.standstill
      continue

    moved_recently = min_moving_kph <= 0.0 or any(v > min_moving_kph for v in recent_speed)
    if prev_standstill == 0 and point.standstill == 1 and moved_recently:
      pending_rise = Anchor(
        kind="stand_rise",
        t_s=point.t_s,
        speed_kph=point.speed_kph,
        standstill=point.standstill,
        traction_brake=point.traction_brake,
        brake_on=point.brake_on,
        active=point.active,
      )
      if anchor_kind == "stand_rise":
        seen_events += 1
        if seen_events == event_index:
          return pending_rise

    if pending_rise is not None and point.speed_kph <= zero_threshold_kph:
      zero_anchor = Anchor(
        kind="first_zero",
        t_s=point.t_s,
        speed_kph=point.speed_kph,
        standstill=point.standstill,
        traction_brake=point.traction_brake,
        brake_on=point.brake_on,
        active=point.active,
      )
      if anchor_kind == "first_zero":
        seen_events += 1
        if seen_events == event_index:
          return zero_anchor
      pending_rise = None
      recent_speed.clear()

    prev_standstill = point.standstill

  raise RuntimeError(f"No matching {anchor_kind} event index={event_index} found in {path}")


def collect_transitions(path: Path, bus: int, addrs: tuple[int, ...], anchor: Anchor, pre_seconds: float, post_seconds: float) -> list[Transition]:
  cp = build_parser(bus)
  first_t_s: float | None = None
  last_data: dict[int, str] = {}
  out: list[Transition] = []

  window_start = anchor.t_s - pre_seconds
  window_end = anchor.t_s + post_seconds

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
    if rel_t_s < window_start or rel_t_s > window_end:
      continue

    for addr, dat, _src in frames:
      if addr not in addrs:
        continue
      hex_data = dat.hex()
      if last_data.get(addr) == hex_data:
        continue
      out.append(Transition(
        dt_s=rel_t_s - anchor.t_s,
        t_s=rel_t_s,
        address=addr,
        data_hex=hex_data,
        speed_kph=cp.vl["ENGINE_DATA"]["SPEED"],
        standstill=int(cp.vl["PEDALS"]["STANDSTILL"]),
        traction_brake=int(cp.vl["TRACTION"]["BRAKE"]),
        brake_on=int(cp.vl["PEDALS"]["BRAKE_ON"]),
        active=int(cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"]),
      ))
      last_data[addr] = hex_data

  return out


def print_report(path: Path, anchor: Anchor, transitions: list[Transition]) -> None:
  print(
    f"{path.name}: anchor={anchor.kind} t={anchor.t_s:.3f}s speed={anchor.speed_kph:.2f}kph "
    f"stand={anchor.standstill} trb={anchor.traction_brake} brake_on={anchor.brake_on} active={anchor.active}"
  )
  for transition in transitions:
    print(
      f"  dt={transition.dt_s:+.3f}s t={transition.t_s:.3f}s "
      f"0x{transition.address:03x} {transition.data_hex} "
      f"speed={transition.speed_kph:.2f} stand={transition.standstill} "
      f"trb={transition.traction_brake} brake_on={transition.brake_on} active={transition.active}"
    )


def main() -> None:
  args = parse_args()
  addrs = parse_addrs(args.addrs)
  json_payload: dict[str, object] = {}

  for path in args.rlogs:
    anchor = find_anchor(path, args.bus, args.anchor, args.min_moving_kph, args.zero_threshold_kph, args.event_index)
    transitions = collect_transitions(path, args.bus, addrs, anchor, args.pre_seconds, args.post_seconds)
    print_report(path, anchor, transitions)
    json_payload[str(path)] = {
      "anchor": asdict(anchor),
      "transitions": [asdict(transition) for transition in transitions],
    }

  if args.json_output is not None:
    args.json_output.write_text(json.dumps(json_payload, indent=2) + "\n")
    print(f"\nWrote JSON summary to {args.json_output}")


if __name__ == "__main__":
  main()
