#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path

from opendbc.can.dbc import DBC
from opendbc.can.parser import CANParser, get_raw_value
from opendbc.car.logreader import LogReader


MAZDA_DBC = DBC("mazda_2017")


@dataclass(frozen=True)
class Transition:
  rel_t_s: float
  address: int
  src: int
  before: str | None
  after: str


@dataclass(frozen=True)
class StopWindow:
  path: Path
  anchor_kind: str
  anchor_t_s: float
  first_zero_t_s: float | None
  first_standstill_t_s: float | None
  transitions: tuple[Transition, ...]


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Compare Mazda stock-vs-alpha CAN transitions around the first zero-speed stop window.",
  )
  parser.add_argument("stock_rlog", type=Path, help="Stock stop route")
  parser.add_argument("alpha_rlog", type=Path, help="Alpha route with creep / failed hold")
  parser.add_argument("--state-src", type=int, default=0,
                      help="CAN src used for stop-window state tracking (ENGINE_DATA/PEDALS)")
  parser.add_argument("--stock-default-src", type=int, default=0,
                      help="Default CAN src to compare in the stock log")
  parser.add_argument("--alpha-default-src", type=int, default=0,
                      help="Default CAN src to compare in the alpha log")
  parser.add_argument("--stock-src-overrides", type=str, default="",
                      help="Comma-separated per-address src overrides for the stock log, e.g. 0x21b=0,0x21c=0")
  parser.add_argument("--alpha-src-overrides", type=str, default="",
                      help="Comma-separated per-address src overrides for the alpha log, e.g. 0x21b=128,0x21c=128,0x21f=0,0x165=0")
  parser.add_argument("--pre-seconds", type=float, default=1.5, help="Seconds before first zero speed to keep")
  parser.add_argument("--post-seconds", type=float, default=1.5, help="Seconds after first zero speed to keep")
  parser.add_argument("--anchor", choices=("first_zero", "stand_rise"), default="first_zero",
                      help="Anchor the comparison window on the first zero-speed sample after movement, or on the first PEDALS.STANDSTILL rise after movement")
  parser.add_argument("--event-index", type=int, default=0,
                      help="Which detected stop/standstill event to analyze in both logs (0-based)")
  parser.add_argument("--stock-event-index", type=int,
                      help="Override the stock-log stop/standstill event index (0-based)")
  parser.add_argument("--alpha-event-index", type=int,
                      help="Override the alpha-log stop/standstill event index (0-based)")
  parser.add_argument("--moving-threshold", type=float, default=0.25, help="Require prior speed above this many m/s before selecting a stop")
  parser.add_argument("--zero-threshold", type=float, default=0.02, help="Treat speed at or below this many m/s as zero")
  parser.add_argument("--max-addrs", type=int, default=25, help="Max addresses to print in each summary")
  parser.add_argument("--max-transitions", type=int, default=8, help="Max stock-only transitions to print per address")
  return parser.parse_args()


def addr_label(address: int) -> str:
  msg = MAZDA_DBC.addr_to_msg.get(address)
  return f"{hex(address)} ({msg.name})" if msg is not None else hex(address)


def addr_src_label(address: int, src: int) -> str:
  return f"{addr_label(address)} src={src}"


def changed_bytes(before: bytes, after: bytes) -> list[int]:
  return [i for i, (lhs, rhs) in enumerate(zip(before, after, strict=True)) if lhs != rhs]


def parse_src_overrides(raw: str) -> dict[int, int]:
  overrides: dict[int, int] = {}
  if not raw.strip():
    return overrides

  for token in raw.split(","):
    token = token.strip()
    if not token:
      continue
    address_raw, src_raw = token.split("=", 1)
    overrides[int(address_raw, 0)] = int(src_raw, 0)
  return overrides


def selected_src(address: int, default_src: int, src_overrides: dict[int, int]) -> int:
  return src_overrides.get(address, default_src)


FOCUS_FIELDS: dict[int, tuple[str, ...]] = {
  0x165: ("ACC_ACTIVE", "BRAKE_ON", "STANDSTILL", "GEAR"),
  0x415: ("BRAKE", "IS_MOVING", "BRAKE_WARNING", "ABS_MALFUNCTION",
          "DSC_OFF", "TCS_DCS_MALFUNCTION", "LOUD_BEEP", "CTR1", "CTR2", "CTR3"),
  0x436: ("NEW_SIGNAL_1", "NEW_SIGNAL_2", "NEW_SIGNAL_3", "NEW_SIGNAL_4"),
  0x21B: ("ACCEL_CMD", "ACC_ACTIVE", "ACC_SET_ALLOWED", "CTR1"),
  0x21C: ("CRZ_ACTIVE", "CRZ_AVAILABLE", "ACC_ACTIVE_2", "DISTANCE_SETTING",
          "RADAR_HAS_LEAD", "RADAR_LEAD_RELATIVE_DISTANCE", "ACC_GAS_MAYBE", "ACC_GAS_MAYBE2",
          "NEW_SIGNAL_9", "NEW_SIGNAL_10"),
  0x21F: ("ACCEL_CMD", "ACCEL_CMD_LOW_RES", "CRUISE_ACTIVE_CAR_MOVING",
          "CRZ_STARTED", "GAS_MAYBE", "NEW_SIGNAL_21", "CRZ_SPEED"),
}


def decode_focus_fields(address: int, raw_hex: str) -> str | None:
  if address not in FOCUS_FIELDS:
    return None

  msg = MAZDA_DBC.addr_to_msg.get(address)
  if msg is None:
    return None

  raw = bytes.fromhex(raw_hex)
  parts: list[str] = []
  for field in FOCUS_FIELDS[address]:
    sig = msg.sigs[field]
    raw_value = get_raw_value(raw, sig)
    if sig.is_signed:
      raw_value -= ((raw_value >> (sig.size - 1)) & 0x1) * (1 << sig.size)
    value = raw_value * sig.factor + sig.offset
    parts.append(f"{field}={value}")
  return ", ".join(parts)


def find_stop_window(path: Path, state_src: int, default_src: int, src_overrides: dict[int, int],
                     pre_s: float, post_s: float, anchor_kind: str, event_index: int,
                     moving_threshold: float, zero_threshold: float) -> StopWindow:
  cp = CANParser("mazda_2017", [("ENGINE_DATA", 100), ("PEDALS", 50)], state_src)
  recent_events: deque[tuple[float, list[tuple[int, bytes, int]]]] = deque()
  last_payload: dict[tuple[int, int], str] = {}
  pending_after: list[tuple[float, list[tuple[int, bytes, int]]]] = []

  first_t_s: float | None = None
  anchor_t_s: float | None = None
  first_zero_t_s: float | None = None
  first_standstill_t_s: float | None = None
  prev_standstill: bool | None = None
  pending_rise_t_s: float | None = None
  seen_events = -1
  movement_armed = False

  for event in LogReader(str(path), only_union_types=True):
    if event.which() != "can":
      continue

    state_frames = [(m.address, bytes(m.dat), m.src) for m in event.can if m.src == state_src]
    if not state_frames:
      continue

    t_s = event.logMonoTime * 1e-9
    if first_t_s is None:
      first_t_s = t_s
    rel_t_s = t_s - first_t_s
    cp.update([(event.logMonoTime, state_frames)])
    v_ego = cp.vl["ENGINE_DATA"]["SPEED"] / 3.6
    standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    if v_ego > moving_threshold:
      movement_armed = True

    selected_frames = [
      (m.address, bytes(m.dat), m.src)
      for m in event.can
      if m.src == selected_src(m.address, default_src, src_overrides)
    ]

    recent_events.append((rel_t_s, selected_frames))
    while recent_events and (rel_t_s - recent_events[0][0]) > pre_s:
      recent_events.popleft()

    if anchor_t_s is not None:
      if first_standstill_t_s is None and standstill:
        first_standstill_t_s = rel_t_s
      if first_zero_t_s is None and v_ego <= zero_threshold:
        first_zero_t_s = rel_t_s
      pending_after.append((rel_t_s, selected_frames))
      if (rel_t_s - anchor_t_s) >= post_s:
        break
      prev_standstill = standstill
      continue

    if prev_standstill is None:
      prev_standstill = standstill
      if anchor_kind == "stand_rise" and movement_armed and standstill:
        seen_events += 1
        movement_armed = False
        if seen_events == event_index:
          anchor_t_s = rel_t_s
          first_standstill_t_s = rel_t_s
          if v_ego <= zero_threshold:
            first_zero_t_s = rel_t_s
          pending_after = list(recent_events)
      continue

    if not prev_standstill and standstill and movement_armed:
      seen_events += 1
      movement_armed = False
      if anchor_kind == "first_zero" and seen_events == event_index:
        pending_rise_t_s = rel_t_s
      if anchor_kind == "stand_rise":
        if seen_events == event_index:
          anchor_t_s = rel_t_s
          first_standstill_t_s = rel_t_s
          if v_ego <= zero_threshold:
            first_zero_t_s = rel_t_s
          pending_after = list(recent_events)
          prev_standstill = standstill
          continue

    if pending_rise_t_s is not None and v_ego <= zero_threshold:
      if anchor_kind == "first_zero":
        anchor_t_s = rel_t_s
        first_zero_t_s = rel_t_s
        first_standstill_t_s = pending_rise_t_s
        pending_after = list(recent_events)
        pending_rise_t_s = None
        prev_standstill = standstill
        continue
      pending_rise_t_s = None

    prev_standstill = standstill

  if anchor_t_s is None:
    raise RuntimeError(f"Could not find anchor={anchor_kind} event_index={event_index} in {path}")

  transitions: list[Transition] = []
  for event_rel_t_s, frames in pending_after:
    rel_t_s = event_rel_t_s - anchor_t_s
    for address, dat, src in frames:
      raw = dat.hex()
      key = (address, src)
      previous = last_payload.get(key)
      if previous != raw:
        transitions.append(Transition(rel_t_s=rel_t_s, address=address, src=src, before=previous, after=raw))
        last_payload[key] = raw

  if first_zero_t_s is not None:
    first_zero_t_s -= anchor_t_s
  if first_standstill_t_s is not None:
    first_standstill_t_s -= anchor_t_s

  return StopWindow(
    path=path,
    anchor_kind=anchor_kind,
    anchor_t_s=anchor_t_s,
    first_zero_t_s=first_zero_t_s,
    first_standstill_t_s=first_standstill_t_s,
    transitions=tuple(transitions),
  )


def transitions_by_addr(window: StopWindow) -> dict[tuple[int, int], list[Transition]]:
  out: dict[tuple[int, int], list[Transition]] = defaultdict(list)
  for transition in window.transitions:
    out[(transition.address, transition.src)].append(transition)
  return out


def transition_key(transition: Transition) -> tuple[int, str | None, str]:
  return transition.src, transition.before, transition.after


def print_window_summary(window: StopWindow) -> None:
  print(f"{window.path.name}:")
  print(f"  anchor={window.anchor_kind} t={window.anchor_t_s:.3f}s")
  if window.first_zero_t_s is None:
    print("  first_zero=never")
  else:
    print(f"  first_zero={window.first_zero_t_s:+.3f}s relative to anchor")
  if window.first_standstill_t_s is None:
    print("  first_PEDALS.STANDSTILL=never")
  else:
    print(f"  first_PEDALS.STANDSTILL={window.first_standstill_t_s:+.3f}s relative to anchor")
  print(f"  captured_transitions={len(window.transitions)}")


def print_unique_transition_summary(stock_window: StopWindow, alpha_window: StopWindow,
                                    max_addrs: int, max_transitions: int) -> None:
  stock_by_addr = transitions_by_addr(stock_window)
  alpha_keys_by_addr: dict[tuple[int, int], set[tuple[int, str | None, str]]] = {
    address: {transition_key(t) for t in transitions}
    for address, transitions in transitions_by_addr(alpha_window).items()
  }

  unique_stock: list[tuple[tuple[int, int], list[Transition]]] = []
  for address_src, transitions in stock_by_addr.items():
    alpha_keys = alpha_keys_by_addr.get(address_src, set())
    unique = [t for t in transitions if transition_key(t) not in alpha_keys]
    if unique:
      unique_stock.append((address_src, unique))

  unique_stock.sort(key=lambda item: (-len(item[1]), item[0][0], item[0][1]))

  print("\nStock-only stop-window transitions:")
  for (address, src), unique in unique_stock[:max_addrs]:
    msg = MAZDA_DBC.addr_to_msg.get(address)
    common_changed: dict[int, int] = defaultdict(int)
    for transition in unique:
      if transition.before is None:
        continue
      before_b = bytes.fromhex(transition.before)
      after_b = bytes.fromhex(transition.after)
      for byte_index in changed_bytes(before_b, after_b):
        common_changed[byte_index] += 1

    print(f"  {addr_src_label(address, src):28} unique_transitions={len(unique)} changed_bytes={dict(sorted(common_changed.items()))}")
    if msg is not None:
      signal_names = ", ".join(msg.sigs.keys())
      print(f"    signals: {signal_names}")
    for transition in unique[:max_transitions]:
      if transition.before is None:
        print(f"    t={transition.rel_t_s:+.3f}s init -> {transition.after}")
        decoded = decode_focus_fields(transition.address, transition.after)
        if decoded is not None:
          print(f"      {decoded}")
      else:
        before_b = bytes.fromhex(transition.before)
        after_b = bytes.fromhex(transition.after)
        print(
          f"    t={transition.rel_t_s:+.3f}s {transition.before} -> {transition.after} "
          f"bytes={changed_bytes(before_b, after_b)}"
        )
        decoded = decode_focus_fields(transition.address, transition.after)
        if decoded is not None:
          print(f"      {decoded}")
    if len(unique) > max_transitions:
      print(f"    ... {len(unique) - max_transitions} more")


def print_stock_and_alpha_for_focus_addrs(stock_window: StopWindow, alpha_window: StopWindow) -> None:
  stock_by_addr = transitions_by_addr(stock_window)
  alpha_by_addr = transitions_by_addr(alpha_window)

  print("\nFocused addresses:")
  focus_keys = sorted({*stock_by_addr.keys(), *alpha_by_addr.keys()})
  for address, src in focus_keys:
    stock = stock_by_addr.get((address, src), [])
    alpha = alpha_by_addr.get((address, src), [])
    if not stock and not alpha:
      continue
    if address not in (0x165, 0x202, 0x21B, 0x21C, 0x21F, 0x0FD, 0x167, 0x415, 0x420, 0x436):
      continue
    print(f"  {addr_src_label(address, src)}")
    print(f"    stock transitions={len(stock)} alpha transitions={len(alpha)}")
    for label, transitions in (("stock", stock), ("alpha", alpha)):
      for transition in transitions[:5]:
        if transition.before is None:
          print(f"    {label:5} t={transition.rel_t_s:+.3f}s init -> {transition.after}")
        else:
          print(f"    {label:5} t={transition.rel_t_s:+.3f}s {transition.before} -> {transition.after}")
        decoded = decode_focus_fields(transition.address, transition.after)
        if decoded is not None:
          print(f"          {decoded}")
      if len(transitions) > 5:
        print(f"    {label:5} ... {len(transitions) - 5} more")


def main() -> None:
  args = parse_args()
  stock_event_index = args.stock_event_index if args.stock_event_index is not None else args.event_index
  alpha_event_index = args.alpha_event_index if args.alpha_event_index is not None else args.event_index
  stock_window = find_stop_window(args.stock_rlog, args.state_src, args.stock_default_src,
                                  parse_src_overrides(args.stock_src_overrides),
                                  args.pre_seconds, args.post_seconds, args.anchor, stock_event_index,
                                  args.moving_threshold, args.zero_threshold)
  alpha_window = find_stop_window(args.alpha_rlog, args.state_src, args.alpha_default_src,
                                  parse_src_overrides(args.alpha_src_overrides),
                                  args.pre_seconds, args.post_seconds, args.anchor, alpha_event_index,
                                  args.moving_threshold, args.zero_threshold)

  print_window_summary(stock_window)
  print_window_summary(alpha_window)
  print_stock_and_alpha_for_focus_addrs(stock_window, alpha_window)
  print_unique_transition_summary(stock_window, alpha_window, args.max_addrs, args.max_transitions)


if __name__ == "__main__":
  main()
