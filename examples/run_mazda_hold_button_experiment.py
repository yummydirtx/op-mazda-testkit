#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from subprocess import CalledProcessError, check_output

from opendbc.car.structs import CarParams
from panda import Panda


@dataclass(frozen=True)
class TimedFrame:
  delay_s: float
  address: int
  data: bytes


def _frame(delay_s: float, address: int, data_hex: str) -> TimedFrame:
  return TimedFrame(delay_s=delay_s, address=address, data=bytes.fromhex(data_hex))


# Candidate manual HOLD button/state transitions observed in:
#   HOLDHOLDHOLD.zst
#   holddrivehold.zst
#
# These are intentionally split into small presets so they can be tested
# independently on-car.
PRESETS: dict[str, list[TimedFrame]] = {
  "hold-on-9a": [
    _frame(0.000, 0x9A, "0008000088000300"),
  ],
  "hold-on-50": [
    _frame(0.000, 0x50, "03000f02aa55aaf9"),
    _frame(0.020, 0x50, "02000f02aa55aafb"),
  ],
  "hold-on-436": [
    _frame(0.000, 0x436, "0000190000000001"),
  ],
  "hold-on-274": [
    _frame(0.000, 0x274, "8001004002040200"),
  ],
  "hold-on-9b": [
    _frame(0.000, 0x9B, "2000000000000000"),
  ],
  "hold-on-state-all": [
    _frame(0.000, 0x9A, "0008000088000300"),
    _frame(0.000, 0x9B, "2000000000000000"),
    _frame(0.000, 0x436, "0000190000000001"),
  ],
  "hold-on-50-274": [
    _frame(0.000, 0x50, "03000f02aa55aaf9"),
    _frame(0.000, 0x274, "8001004002040200"),
    _frame(0.020, 0x50, "02000f02aa55aafb"),
  ],
  "hold-on-50-9b": [
    _frame(0.000, 0x50, "03000f02aa55aaf9"),
    _frame(0.020, 0x50, "02000f02aa55aafb"),
    _frame(0.020, 0x9B, "2200000000000000"),
  ],
  "hold-on-274-9b": [
    _frame(0.000, 0x274, "8001004002040200"),
    _frame(0.020, 0x9B, "2200000000000000"),
  ],
  "hold-on-all": [
    _frame(0.000, 0x50, "03000f02aa55aaf9"),
    _frame(0.000, 0x274, "8001004002040200"),
    _frame(0.020, 0x50, "02000f02aa55aafb"),
    _frame(0.020, 0x9B, "2200000000000000"),
  ],
  "hold-off-50": [
    _frame(0.000, 0x50, "01000f02aa55aafd"),
    _frame(0.020, 0x50, "00000f02aa55aaff"),
  ],
  "hold-off-274": [
    _frame(0.000, 0x274, "0000004000040200"),
    _frame(0.020, 0x274, "0010004000000200"),
  ],
  "hold-off-9a": [
    _frame(0.000, 0x9A, "000800c088000300"),
    _frame(0.020, 0x9A, "00c800c088000300"),
  ],
  "hold-off-9b": [
    _frame(0.000, 0x9B, "2000140000000000"),
    _frame(0.040, 0x9B, "2000000000000000"),
  ],
  "hold-off-436": [
    _frame(0.000, 0x436, "1400000000000001"),
    _frame(0.020, 0x436, "0000000000000001"),
  ],
  "hold-off-all": [
    _frame(0.000, 0x50, "01000f02aa55aafd"),
    _frame(0.020, 0x50, "00000f02aa55aaff"),
    _frame(0.020, 0x274, "0000004000040200"),
    _frame(0.040, 0x9A, "000800c088000300"),
    _frame(0.040, 0x9B, "2000140000000000"),
    _frame(0.060, 0x9A, "00c800c088000300"),
    _frame(0.060, 0x274, "0010004000000200"),
    _frame(0.080, 0x9B, "2000000000000000"),
    _frame(0.100, 0x436, "1400000000000001"),
    _frame(0.120, 0x436, "0000000000000001"),
  ],
}


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Send candidate Mazda manual HOLD button/state CAN pulses through Panda."
  )
  parser.add_argument("preset", choices=sorted(PRESETS), help="Named message preset to inject")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus to send on")
  parser.add_argument("--can-speed-kbps", type=int, default=500, help="CAN speed in kbps")
  parser.add_argument("--repeat", type=int, default=1, help="How many times to replay the preset")
  parser.add_argument("--repeat-gap", type=float, default=0.5, help="Seconds between repeats")
  parser.add_argument("--dry-run", action="store_true", help="Print the preset frames without sending them")
  parser.add_argument("--all-output", action="store_true",
                      help="Required. Sends with Panda allOutput safety mode and disables normal TX safety.")
  return parser.parse_args()


def ensure_pandad_stopped() -> None:
  try:
    check_output(["pidof", "pandad"])
    print("pandad is running, please stop openpilot before using direct Panda injection.")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1:
      raise


def describe_preset(name: str, frames: list[TimedFrame]) -> str:
  lines = [f"Preset {name}:"]
  for frame in frames:
    lines.append(f"  t+{frame.delay_s:0.3f}s addr={hex(frame.address)} data={frame.data.hex()}")
  return "\n".join(lines)


def run_preset(panda: Panda, frames: list[TimedFrame], bus: int, repeat: int, repeat_gap: float) -> None:
  for iteration in range(repeat):
    start = time.monotonic()
    index = 0
    while index < len(frames):
      now = time.monotonic()
      due: list[TimedFrame] = []
      while index < len(frames) and (now - start) >= frames[index].delay_s:
        due.append(frames[index])
        index += 1
      if due:
        panda.can_send_many([(frame.address, frame.data, bus) for frame in due], timeout=0)
      else:
        time.sleep(0.001)
    if iteration != repeat - 1 and repeat_gap > 0:
      time.sleep(repeat_gap)


def main() -> None:
  args = parse_args()
  frames = PRESETS[args.preset]
  print(describe_preset(args.preset, frames))

  if args.dry_run:
    return
  if not args.all_output:
    print("--all-output is required for this script. This mode disables Panda TX safety checks.")
    sys.exit(1)
  if args.repeat < 1:
    print("--repeat must be >= 1")
    sys.exit(1)

  ensure_pandad_stopped()

  panda = Panda()
  panda.set_can_speed_kbps(args.bus, args.can_speed_kbps)
  panda.set_safety_mode(CarParams.SafetyModel.allOutput)

  try:
    print(f"Sending preset {args.preset} on bus {args.bus} ({args.repeat}x).")
    print("Use this parked in D with your foot on the brake, no cruise, and watch the HOLD lamp.")
    run_preset(panda, frames, args.bus, args.repeat, args.repeat_gap)
  finally:
    panda.set_safety_mode(CarParams.SafetyModel.silent)
    print("Panda returned to silent safety mode.")


if __name__ == "__main__":
  main()
