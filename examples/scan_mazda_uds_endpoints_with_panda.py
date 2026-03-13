#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import string
import sys
from pathlib import Path
from subprocess import CalledProcessError, check_output

from opendbc.car import uds
from opendbc.car.structs import CarParams
from panda import Panda


DEFAULT_ADDRS = tuple(range(0x700, 0x800))
DEFAULT_DIDS: tuple[tuple[str, uds.DATA_IDENTIFIER_TYPE], ...] = (
  ("vin", uds.DATA_IDENTIFIER_TYPE.VIN),
  ("active_session", uds.DATA_IDENTIFIER_TYPE.ACTIVE_DIAGNOSTIC_SESSION),
  ("boot_sw", uds.DATA_IDENTIFIER_TYPE.BOOT_SOFTWARE_IDENTIFICATION),
  ("app_sw", uds.DATA_IDENTIFIER_TYPE.APPLICATION_SOFTWARE_IDENTIFICATION),
  ("part_number", uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER),
  ("sw_number", uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_NUMBER),
  ("sw_version", uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER),
  ("supplier", uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_IDENTIFIER),
  ("ecu_serial", uds.DATA_IDENTIFIER_TYPE.ECU_SERIAL_NUMBER),
  ("system_name", uds.DATA_IDENTIFIER_TYPE.SYSTEM_NAME_OR_ENGINE_TYPE),
)
SESSION_BY_NAME: dict[str, uds.SESSION_TYPE] = {
  "default": uds.SESSION_TYPE.DEFAULT,
  "extended": uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC,
}


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Scan Mazda UDS physical addresses over Panda without sending destructive commands.")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus to scan")
  parser.add_argument("--can-speed-kbps", type=int, default=500, help="CAN speed in kbps")
  parser.add_argument("--start-addr", type=lambda x: int(x, 0), default=0x700, help="Start address, inclusive")
  parser.add_argument("--end-addr", type=lambda x: int(x, 0), default=0x7FF, help="End address, inclusive")
  parser.add_argument("--addr", action="append", type=lambda x: int(x, 0), help="Specific address to scan, may be passed multiple times")
  parser.add_argument("--session", choices=tuple(SESSION_BY_NAME), default="extended", help="Preferred diagnostic session")
  parser.add_argument("--try-default-on-fail", action="store_true", help="If preferred session fails, also try default session")
  parser.add_argument("--timeout", type=float, default=0.2, help="UDS request timeout in seconds")
  parser.add_argument("--output", type=Path, help="Optional JSON output file")
  return parser.parse_args()


def ensure_pandad_stopped() -> None:
  try:
    check_output(["pidof", "pandad"])
    print("pandad is running, please stop openpilot before direct Panda UDS scanning.")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1:
      raise


def iter_addrs(args: argparse.Namespace) -> list[int]:
  if args.addr:
    return sorted(set(args.addr))
  start_addr = min(args.start_addr, args.end_addr)
  end_addr = max(args.start_addr, args.end_addr)
  return list(range(start_addr, end_addr + 1))


def ascii_or_hex(data: bytes) -> str:
  if not data:
    return ""
  printable = set(string.printable.encode("ascii"))
  stripped = data.rstrip(b"\x00 ").lstrip(b"\x00 ")
  if stripped and all(byte in printable and byte not in b"\r\n\t\x0b\x0c" for byte in stripped):
    return stripped.decode("ascii", errors="replace")
  return data.hex()


def session_order(preferred: str, try_default_on_fail: bool) -> list[str]:
  names = [preferred]
  if try_default_on_fail and preferred != "default":
    names.append("default")
  return names


def probe_addr(panda: Panda, addr: int, bus: int, timeout: float, preferred_session: str, try_default_on_fail: bool) -> dict | None:
  result: dict[str, object] = {
    "addr": hex(addr),
    "session_attempts": [],
    "dids": {},
  }

  for session_name in session_order(preferred_session, try_default_on_fail):
    client = uds.UdsClient(panda, addr, bus=bus, timeout=timeout, tx_timeout=timeout, response_pending_timeout=max(timeout, 1.0))
    session_entry: dict[str, object] = {"session": session_name}

    try:
      client.diagnostic_session_control(SESSION_BY_NAME[session_name])
      session_entry["diagnostic_session_control"] = "ok"
    except uds.NegativeResponseError as e:
      session_entry["diagnostic_session_control"] = {
        "negative_response": str(e),
        "service_id": hex(e.service_id),
        "error_code": hex(e.error_code),
      }
      result["session_attempts"].append(session_entry)
      continue
    except Exception as e:
      session_entry["diagnostic_session_control"] = repr(e)
      result["session_attempts"].append(session_entry)
      continue

    try:
      client.tester_present()
      session_entry["tester_present"] = "ok"
    except uds.NegativeResponseError as e:
      session_entry["tester_present"] = {
        "negative_response": str(e),
        "service_id": hex(e.service_id),
        "error_code": hex(e.error_code),
      }
    except Exception as e:
      session_entry["tester_present"] = repr(e)

    any_did = False
    for did_name, did in DEFAULT_DIDS:
      try:
        data = client.read_data_by_identifier(did)
        result["dids"][did_name] = {
          "did": hex(int(did)),
          "raw_hex": data.hex(),
          "text": ascii_or_hex(data),
        }
        any_did = True
      except uds.NegativeResponseError as e:
        result["dids"][did_name] = {
          "did": hex(int(did)),
          "negative_response": str(e),
          "service_id": hex(e.service_id),
          "error_code": hex(e.error_code),
        }
      except Exception as e:
        result["dids"][did_name] = {
          "did": hex(int(did)),
          "error": repr(e),
        }

    result["session_attempts"].append(session_entry)
    if any_did:
      result["working_session"] = session_name
      return result

  if result["session_attempts"]:
    return result
  return None


def main() -> None:
  args = parse_args()
  ensure_pandad_stopped()

  panda = Panda()
  panda.set_can_speed_kbps(args.bus, args.can_speed_kbps)
  panda.set_safety_mode(CarParams.SafetyModel.elm327)

  addrs = iter_addrs(args)
  results: list[dict] = []

  print(f"Scanning {len(addrs)} addresses on bus {args.bus}...")

  try:
    for addr in addrs:
      result = probe_addr(panda, addr, args.bus, args.timeout, args.session, args.try_default_on_fail)
      if result is None:
        continue

      session_summary = ", ".join(
        f"{attempt['session']}={attempt['diagnostic_session_control']}"
        for attempt in result["session_attempts"]
      )
      working = result.get("working_session")
      did_hits = [
        f"{name}={payload['text']}"
        for name, payload in result["dids"].items()
        if isinstance(payload, dict) and "text" in payload
      ]
      print(f"{result['addr']}: sessions[{session_summary}] working_session={working} dids[{'; '.join(did_hits[:3])}]")
      results.append(result)
  finally:
    panda.set_safety_mode(CarParams.SafetyModel.silent)

  payload = {
    "bus": args.bus,
    "can_speed_kbps": args.can_speed_kbps,
    "addrs": [hex(addr) for addr in addrs],
    "results": results,
  }
  if args.output is not None:
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"\nWrote scan results to {args.output}")


if __name__ == "__main__":
  main()
