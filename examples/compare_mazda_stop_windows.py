#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path

from opendbc.can.dbc import DBC
from opendbc.can.parser import CANParser
from opendbc.car.logreader import LogReader


MAZDA_DBC = DBC("mazda_2017")


@dataclass(frozen=True)
class Transition:
  rel_t_s: float
  address: int
  before: str | None
  after: str


@dataclass(frozen=True)
class StopWindow:
  path: Path
  zero_t_s: float
  first_standstill_t_s: float | None
  transitions: tuple[Transition, ...]


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Compare Mazda stock-vs-alpha CAN transitions around the first zero-speed stop window.",
  )
  parser.add_argument("stock_rlog", type=Path, help="Stock stop route")
  parser.add_argument("alpha_rlog", type=Path, help="Alpha route with creep / failed hold")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus/src to analyze")
  parser.add_argument("--pre-seconds", type=float, default=1.5, help="Seconds before first zero speed to keep")
  parser.add_argument("--post-seconds", type=float, default=1.5, help="Seconds after first zero speed to keep")
  parser.add_argument("--moving-threshold", type=float, default=1.0, help="Require prior speed above this many m/s before selecting a stop")
  parser.add_argument("--zero-threshold", type=float, default=0.02, help="Treat speed at or below this many m/s as zero")
  parser.add_argument("--max-addrs", type=int, default=25, help="Max addresses to print in each summary")
  parser.add_argument("--max-transitions", type=int, default=8, help="Max stock-only transitions to print per address")
  return parser.parse_args()


def addr_label(address: int) -> str:
  msg = MAZDA_DBC.addr_to_msg.get(address)
  return f"{hex(address)} ({msg.name})" if msg is not None else hex(address)


def changed_bytes(before: bytes, after: bytes) -> list[int]:
  return [i for i, (lhs, rhs) in enumerate(zip(before, after, strict=True)) if lhs != rhs]


def find_stop_window(path: Path, bus: int, pre_s: float, post_s: float,
                     moving_threshold: float, zero_threshold: float) -> StopWindow:
  cp = CANParser("mazda_2017", [("ENGINE_DATA", 100), ("PEDALS", 50)], bus)
  recent_speed = deque(maxlen=500)
  recent_events: deque[tuple[float, list[tuple[int, bytes, int]]]] = deque()
  last_payload: dict[int, str] = {}
  pending_after: list[tuple[float, list[tuple[int, bytes, int]]]] = []

  zero_t_s: float | None = None
  first_standstill_t_s: float | None = None

  for event in LogReader(str(path), only_union_types=True):
    if event.which() != "can":
      continue

    can_frames = [(m.address, bytes(m.dat), m.src) for m in event.can if m.src == bus]
    if not can_frames:
      continue

    t_s = event.logMonoTime * 1e-9
    cp.update([(event.logMonoTime, can_frames)])
    v_ego = cp.vl["ENGINE_DATA"]["SPEED"] / 3.6
    standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1

    if first_standstill_t_s is None and standstill:
      first_standstill_t_s = t_s

    recent_speed.append((t_s, v_ego))
    recent_events.append((t_s, can_frames))
    while recent_events and (t_s - recent_events[0][0]) > pre_s:
      recent_events.popleft()

    if zero_t_s is None:
      seen_moving = any(speed > moving_threshold for _, speed in recent_speed)
      if seen_moving and v_ego <= zero_threshold:
        zero_t_s = t_s
        pending_after = list(recent_events)
        continue

    if zero_t_s is not None:
      pending_after.append((t_s, can_frames))
      if (t_s - zero_t_s) >= post_s:
        break

  if zero_t_s is None:
    raise RuntimeError(f"Could not find a stop window in {path}")

  transitions: list[Transition] = []
  for t_s, frames in pending_after:
    rel_t_s = t_s - zero_t_s
    for address, dat, _src in frames:
      raw = dat.hex()
      previous = last_payload.get(address)
      if previous != raw:
        transitions.append(Transition(rel_t_s=rel_t_s, address=address, before=previous, after=raw))
        last_payload[address] = raw

  if first_standstill_t_s is not None:
    first_standstill_t_s -= zero_t_s

  return StopWindow(
    path=path,
    zero_t_s=zero_t_s,
    first_standstill_t_s=first_standstill_t_s,
    transitions=tuple(transitions),
  )


def transitions_by_addr(window: StopWindow) -> dict[int, list[Transition]]:
  out: dict[int, list[Transition]] = defaultdict(list)
  for transition in window.transitions:
    out[transition.address].append(transition)
  return out


def transition_key(transition: Transition) -> tuple[str | None, str]:
  return transition.before, transition.after


def print_window_summary(window: StopWindow) -> None:
  print(f"{window.path.name}:")
  print(f"  first_zero_t={window.zero_t_s:.3f}s")
  if window.first_standstill_t_s is None:
    print("  first_PEDALS.STANDSTILL=never")
  else:
    print(f"  first_PEDALS.STANDSTILL={window.first_standstill_t_s:+.3f}s relative to first zero")
  print(f"  captured_transitions={len(window.transitions)}")


def print_unique_transition_summary(stock_window: StopWindow, alpha_window: StopWindow,
                                    max_addrs: int, max_transitions: int) -> None:
  stock_by_addr = transitions_by_addr(stock_window)
  alpha_keys_by_addr: dict[int, set[tuple[str | None, str]]] = {
    address: {transition_key(t) for t in transitions}
    for address, transitions in transitions_by_addr(alpha_window).items()
  }

  unique_stock: list[tuple[int, list[Transition]]] = []
  for address, transitions in stock_by_addr.items():
    alpha_keys = alpha_keys_by_addr.get(address, set())
    unique = [t for t in transitions if transition_key(t) not in alpha_keys]
    if unique:
      unique_stock.append((address, unique))

  unique_stock.sort(key=lambda item: (-len(item[1]), item[0]))

  print("\nStock-only stop-window transitions:")
  for address, unique in unique_stock[:max_addrs]:
    msg = MAZDA_DBC.addr_to_msg.get(address)
    common_changed: dict[int, int] = defaultdict(int)
    for transition in unique:
      if transition.before is None:
        continue
      before_b = bytes.fromhex(transition.before)
      after_b = bytes.fromhex(transition.after)
      for byte_index in changed_bytes(before_b, after_b):
        common_changed[byte_index] += 1

    print(f"  {addr_label(address):28} unique_transitions={len(unique)} changed_bytes={dict(sorted(common_changed.items()))}")
    if msg is not None:
      signal_names = ", ".join(msg.sigs.keys())
      print(f"    signals: {signal_names}")
    for transition in unique[:max_transitions]:
      if transition.before is None:
        print(f"    t={transition.rel_t_s:+.3f}s init -> {transition.after}")
      else:
        before_b = bytes.fromhex(transition.before)
        after_b = bytes.fromhex(transition.after)
        print(
          f"    t={transition.rel_t_s:+.3f}s {transition.before} -> {transition.after} "
          f"bytes={changed_bytes(before_b, after_b)}"
        )
    if len(unique) > max_transitions:
      print(f"    ... {len(unique) - max_transitions} more")


def print_stock_and_alpha_for_focus_addrs(stock_window: StopWindow, alpha_window: StopWindow) -> None:
  focus_addrs = (
    0x165,  # PEDALS
    0x202,  # ENGINE_DATA
    0x21B,  # CRZ_INFO
    0x21C,  # CRZ_CTRL
    0x21F,  # CRZ_EVENTS
    0x0FD,  # GAS
    0x167,  # MORE_GAS
    0x420,
    0x436,
  )
  stock_by_addr = transitions_by_addr(stock_window)
  alpha_by_addr = transitions_by_addr(alpha_window)

  print("\nFocused addresses:")
  for address in focus_addrs:
    stock = stock_by_addr.get(address, [])
    alpha = alpha_by_addr.get(address, [])
    if not stock and not alpha:
      continue
    print(f"  {addr_label(address)}")
    print(f"    stock transitions={len(stock)} alpha transitions={len(alpha)}")
    for label, transitions in (("stock", stock), ("alpha", alpha)):
      for transition in transitions[:5]:
        if transition.before is None:
          print(f"    {label:5} t={transition.rel_t_s:+.3f}s init -> {transition.after}")
        else:
          print(f"    {label:5} t={transition.rel_t_s:+.3f}s {transition.before} -> {transition.after}")
      if len(transitions) > 5:
        print(f"    {label:5} ... {len(transitions) - 5} more")


def main() -> None:
  args = parse_args()
  stock_window = find_stop_window(args.stock_rlog, args.bus, args.pre_seconds, args.post_seconds,
                                  args.moving_threshold, args.zero_threshold)
  alpha_window = find_stop_window(args.alpha_rlog, args.bus, args.pre_seconds, args.post_seconds,
                                  args.moving_threshold, args.zero_threshold)

  print_window_summary(stock_window)
  print_window_summary(alpha_window)
  print_stock_and_alpha_for_focus_addrs(stock_window, alpha_window)
  print_unique_transition_summary(stock_window, alpha_window, args.max_addrs, args.max_transitions)


if __name__ == "__main__":
  main()
