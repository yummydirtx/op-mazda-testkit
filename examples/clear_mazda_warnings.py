#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import sys
import traceback
from pathlib import Path
from subprocess import CalledProcessError, check_output

from opendbc.car import uds
from opendbc.car.structs import CarParams
from panda import Panda


DEFAULT_ADDRS: tuple[int, ...] = (0x760, 0x706, 0x764)
ECU_NAMES: dict[int, str] = {
  0x760: "DSC/ABS",
  0x706: "camera/ADAS",
  0x764: "front radar",
  0x7DF: "functional broadcast",
}


class TeeStream:
  def __init__(self, primary, secondary):
    self.primary = primary
    self.secondary = secondary

  def write(self, data: str) -> int:
    self.primary.write(data)
    self.secondary.write(data)
    return len(data)

  def flush(self) -> None:
    self.primary.flush()
    self.secondary.flush()

  def isatty(self) -> bool:
    return bool(getattr(self.primary, "isatty", lambda: False)())


def default_output_path() -> Path:
  root = Path("/data/mazda") if Path("/data/mazda").exists() else (Path("/data/tmp") if Path("/data/tmp").exists() else Path("/tmp"))
  stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
  return root / f"mazda_warning_reset_{stamp}.log"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Read and clear Mazda warning DTCs for the common MRCC test ECUs.")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus to use")
  parser.add_argument("--addr", type=lambda x: int(x, 0), action="append",
                      help="Override target UDS address. May be repeated. Defaults to 0x760, 0x706, 0x764.")
  parser.add_argument("--include-functional", action="store_true",
                      help="Also send a functional 0x7DF clear after individual ECU clears.")
  parser.add_argument("--read-only", action="store_true",
                      help="Read DTCs only and do not send any clear requests.")
  parser.add_argument("--skip-after-read", action="store_true",
                      help="Do not re-read DTCs after clearing.")
  parser.add_argument("--output", type=Path, default=default_output_path(),
                      help="Local file to tee all output to.")
  return parser.parse_args()


def ensure_pandad_stopped() -> None:
  try:
    check_output(["pidof", "pandad"])
    print("pandad is running, please stop openpilot before clearing DTCs.")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1:
      raise


def describe_addr(addr: int) -> str:
  return ECU_NAMES.get(addr, "unknown ECU")


def start_extended_session(client: uds.UdsClient, addr: int) -> bool:
  try:
    client.diagnostic_session_control(uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC)
    print(f"  extended diagnostic session ok on {hex(addr)}")
    return True
  except uds.MessageTimeoutError as e:
    if addr == 0x7DF:
      print(f"  extended diagnostic session timeout on functional {hex(addr)} (expected quirk): {e}")
      return True
    print(f"  extended diagnostic session timeout on {hex(addr)}: {e}")
    return False
  except uds.NegativeResponseError as e:
    print(f"  extended diagnostic session rejected on {hex(addr)}: {e}")
    return False


def read_dtcs(client: uds.UdsClient, addr: int) -> list[str]:
  data = client.read_dtc_information(uds.DTC_REPORT_TYPE.DTC_BY_STATUS_MASK, uds.DTC_STATUS_MASK_TYPE.ALL)
  lines = [f"  status availability: {' '.join(uds.get_dtc_status_names(data[0]))}"]
  if len(data) == 1:
    lines.append("  no DTCs reported")
    return lines

  lines.append("  DTC status:")
  for i in range(1, len(data), 4):
    dtc_num = uds.get_dtc_num_as_str(data[i:i+3])
    dtc_status = " ".join(uds.get_dtc_status_names(data[i+3]))
    lines.append(f"    {dtc_num} {dtc_status}")
  return lines


def try_read_dtcs(panda: Panda, addr: int, bus: int, heading: str) -> None:
  print(f"{heading} {hex(addr)} ({describe_addr(addr)})")
  client = uds.UdsClient(panda, addr, bus=bus)
  if not start_extended_session(client, addr):
    return
  try:
    for line in read_dtcs(client, addr):
      print(line)
  except uds.MessageTimeoutError as e:
    print(f"  read DTC timeout on {hex(addr)}: {e}")
  except uds.NegativeResponseError as e:
    print(f"  read DTC rejected on {hex(addr)}: {e}")


def clear_dtcs(panda: Panda, addr: int, bus: int) -> None:
  print(f"Clearing {hex(addr)} ({describe_addr(addr)})")
  client = uds.UdsClient(panda, addr, bus=bus)
  if not start_extended_session(client, addr):
    return
  try:
    client.clear_diagnostic_information(uds.DTC_GROUP_TYPE.ALL)
    print("  clear diagnostic information sent")
  except uds.MessageTimeoutError as e:
    if addr == 0x7DF:
      print(f"  functional clear timed out on {hex(addr)} (expected quirk): {e}")
      return
    print(f"  clear diagnostic information timeout on {hex(addr)}: {e}")
  except uds.NegativeResponseError as e:
    print(f"  clear diagnostic information rejected on {hex(addr)}: {e}")


def run(args: argparse.Namespace) -> None:
  ensure_pandad_stopped()
  addrs = tuple(args.addr) if args.addr else DEFAULT_ADDRS

  panda = Panda()
  panda.set_safety_mode(CarParams.SafetyModel.elm327)

  print(f"Started at {dt.datetime.now().isoformat()}")
  print(f"Bus: {args.bus}")
  print("Targets:", ", ".join(f"{hex(addr)} ({describe_addr(addr)})" for addr in addrs))
  if args.include_functional:
    print("Functional clear enabled: 0x7DF")
  if args.read_only:
    print("Read-only mode: no clear requests will be sent.")
  print()

  for addr in addrs:
    try_read_dtcs(panda, addr, args.bus, "Before clear")
    print()

  if args.read_only:
    print("Done. No clear requests were sent.")
    return

  for addr in addrs:
    clear_dtcs(panda, addr, args.bus)
    print()

  if args.include_functional:
    clear_dtcs(panda, 0x7DF, args.bus)
    print()

  if not args.skip_after_read:
    for addr in addrs:
      try_read_dtcs(panda, addr, args.bus, "After clear")
      print()

  print("Done. Power-cycle the vehicle if warnings remain latched.")


def main() -> None:
  args = parse_args()
  args.output.parent.mkdir(parents=True, exist_ok=True)
  with args.output.open("a", buffering=1) as log_fp:
    stdout = sys.stdout
    stderr = sys.stderr
    sys.stdout = TeeStream(stdout, log_fp)
    sys.stderr = TeeStream(stderr, log_fp)
    try:
      print("=" * 80)
      print(f"argv: {' '.join(sys.argv)}")
      print(f"Logging to {args.output}")
      run(args)
    except BaseException:
      traceback.print_exc()
      raise
    finally:
      sys.stdout.flush()
      sys.stderr.flush()
      sys.stdout = stdout
      sys.stderr = stderr


if __name__ == "__main__":
  main()
