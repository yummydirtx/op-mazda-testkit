#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import importlib.util
import json
import sys
import time
import traceback
from collections import Counter, defaultdict
from dataclasses import replace
from pathlib import Path
from subprocess import CalledProcessError, check_output
from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.can.packer import CANPacker

for candidate in (Path("/data/opendbc_test"),):
  if candidate.exists():
    candidate_str = str(candidate)
    if candidate_str not in sys.path:
      sys.path.insert(0, candidate_str)

try:
  from opendbc.car.mazda.longitudinal_experimental import (
    CRZ_CTRL_ADDR,
    CRZ_EVENTS_ADDR,
    CRZ_INFO_ADDR,
    MazdaLongitudinalProfile,
    MazdaLongitudinalReplayMutator,
    command_stream_from_hex_sequence,
    decode_signal,
  )
except ModuleNotFoundError:
  helper_path = Path("/data/opendbc_test/opendbc/car/mazda/longitudinal_experimental.py")
  if not helper_path.exists():
    raise

  spec = importlib.util.spec_from_file_location("mazda_longitudinal_experimental", helper_path)
  if spec is None or spec.loader is None:
    raise RuntimeError(f"Unable to load Mazda helper module from {helper_path}")
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  CRZ_CTRL_ADDR = module.CRZ_CTRL_ADDR
  CRZ_EVENTS_ADDR = module.CRZ_EVENTS_ADDR
  CRZ_INFO_ADDR = module.CRZ_INFO_ADDR
  MazdaLongitudinalProfile = module.MazdaLongitudinalProfile
  MazdaLongitudinalReplayMutator = module.MazdaLongitudinalReplayMutator
  command_stream_from_hex_sequence = module.command_stream_from_hex_sequence
  decode_signal = module.decode_signal

from opendbc.car import uds
from opendbc.car.structs import CarParams
from opendbc.car.mazda.mazdacan import create_button_cmd
from opendbc.car.mazda.values import Buttons, MazdaFlags
from panda import Panda


SESSION_BY_NAME: dict[str, uds.SESSION_TYPE] = {
  "default": uds.SESSION_TYPE.DEFAULT,
  "programming": uds.SESSION_TYPE.PROGRAMMING,
  "extended": uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC,
  "safety": uds.SESSION_TYPE.SAFETY_SYSTEM_DIAGNOSTIC,
}
RADAR_TRACK_ADDRS: tuple[int, ...] = (0x361, 0x362, 0x363, 0x364, 0x365, 0x366)
RADAR_REPLAY_ADDRS: tuple[int, ...] = RADAR_TRACK_ADDRS + (0x499,)
RADAR_REPLAY_HZ = 10.0
DEFAULT_SAMPLE_ADDRS: tuple[int, ...] = (CRZ_INFO_ADDR, CRZ_CTRL_ADDR, CRZ_EVENTS_ADDR, 0x0FD, 0x167) + RADAR_REPLAY_ADDRS
STATUS_MESSAGES: tuple[tuple[str, int], ...] = (
  ("CRZ_CTRL", 50),
  ("CRZ_EVENTS", 50),
  ("CRZ_BTNS", 50),
  ("ENGINE_DATA", 100),
  ("PEDALS", 50),
  ("BRAKE", 50),
)
BUTTON_BY_NAME: dict[str, int] = {
  "none": Buttons.NONE,
  "set-plus": Buttons.SET_PLUS,
  "set-minus": Buttons.SET_MINUS,
  "resume": Buttons.RESUME,
  "cancel": Buttons.CANCEL,
}
STANDSTILL_PEDALS_SEQUENCE: tuple[bytes, ...] = (
  bytes.fromhex("97001455e000145c"),
  bytes.fromhex("97001455e0001352"),
  bytes.fromhex("97001455e00013c3"),
  bytes.fromhex("97001455e000144e"),
  bytes.fromhex("97001455e0001372"),
  bytes.fromhex("97001455e0001340"),
  bytes.fromhex("97001455e00012f5"),
  bytes.fromhex("97001455e00012d9"),
  bytes.fromhex("97001455e00012d9"),
  bytes.fromhex("97001455e00012ab"),
  bytes.fromhex("97001455e000126b"),
  bytes.fromhex("97001455e0001216"),
)
STANDSTILL_PEDALS_LATCH_FRAME = bytes.fromhex("40801775e0000de1")
CRZ_CTRL_MODE_BYTE_INDEXES: tuple[int, ...] = (0, 2, 3, 6)
STOCK_STOPGO_CTRL_STOPPING = bytes.fromhex("0a018b6000001000")
STOCK_STOPGO_CTRL_HOLD_ENTRY = bytes.fromhex("0a018b8000001000")
STOCK_STOPGO_CTRL_HOLD_LATCHED = bytes.fromhex("0a018b8000000000")
STOCK_STOPGO_CTRL_RELEASE = bytes.fromhex("0a018b4000001000")


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


def default_log_path() -> Path:
  root = Path("/data/tmp") if Path("/data/tmp").exists() else Path("/tmp")
  stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
  return root / f"mazda_crz_replacement_{stamp}.log"


def default_sample_path(log_path: Path) -> Path:
  return log_path.with_suffix(".samples.json")


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Hold Mazda radar in a diagnostic session and replace suppressed CRZ_INFO/CRZ_CTRL frames through Panda."
  )
  parser.add_argument("stream_json", type=Path, help="JSON file produced by extract_mazda_longitudinal_replay.py")
  parser.add_argument("--profile", choices=[profile.value for profile in MazdaLongitudinalProfile], help="Override the profile stored in the JSON")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus to send on")
  parser.add_argument("--can-speed-kbps", type=int, default=500, help="CAN bus speed in kbps")
  parser.add_argument("--duration", type=float, default=5.0, help="Replay duration in seconds, 0 for infinite")
  parser.add_argument("--info-accel-cmd", type=float, help="Optional CRZ_INFO.ACCEL_CMD override. Omit to replay captured CRZ_INFO unchanged.")
  parser.add_argument("--session", choices=tuple(SESSION_BY_NAME), default="programming", help="Diagnostic session to hold on the radar ECU")
  parser.add_argument("--radar-addr", type=lambda x: int(x, 0), default=0x764, help="Radar UDS address")
  parser.add_argument("--tester-present-interval", type=float, default=0.5, help="Seconds between raw 0x3E80 tester-present frames")
  parser.add_argument("--handoff-seconds", type=float, default=2.0, help="After stopping tester-present, keep sending 0x21b/0x21c for this long or until radar tracks return")
  parser.add_argument("--exit-default-session", action="store_true",
                      help="After handoff, explicitly request UDS default session on the radar before returning Panda to silent mode")
  parser.add_argument("--handoff-end-on", choices=("radar_tracks", "stock_pair", "stock_pair_and_tracks"), default="stock_pair_and_tracks",
                      help="Condition that ends the exit handoff early instead of waiting the full handoff timeout")
  parser.add_argument("--handoff-stock-pair-count", type=int, default=3,
                      help="How many stock 0x21b and 0x21c frames must be observed during handoff before stock-pair restoration is considered stable")
  parser.add_argument("--replay-pre-session-radar", action="store_true", help="Capture the current 0x361-0x366 + 0x499 radar sequence before entering programming session and replay it at 10 Hz while the radar is suppressed")
  parser.add_argument("--radar-replay-capture-seconds", type=float, default=1.0,
                      help="How much pre-session radar traffic to capture for --replay-pre-session-radar")
  parser.add_argument("--replace-events", action="store_true", help="Also send 0x21f CRZ_EVENTS at 50 Hz, 10 ms ahead of 0x21b/0x21c")
  parser.add_argument("--unsafe-patch-events", action="store_true", help="When overriding info_accel_cmd, also approximate-patch 0x21f. CRZ_EVENTS checksum semantics are still unresolved.")
  parser.add_argument("--target-speed-mph", type=float, help="Optional target set speed in mph. Requires --replace-events and --unsafe-patch-events.")
  parser.add_argument("--engage-button", choices=tuple(BUTTON_BY_NAME), default="none", help="Optional Mazda cruise button pulse to send after radar suppression starts. Keep cruise main on before starting the script.")
  parser.add_argument("--engage-delay", type=float, default=0.75, help="Seconds after radar suppression begins before the first engage button pulse")
  parser.add_argument("--engage-press-seconds", type=float, default=0.20, help="Length of each engage button press in seconds")
  parser.add_argument("--engage-repeat", type=int, default=1, help="How many engage button presses to send")
  parser.add_argument("--engage-repeat-interval", type=float, default=0.75, help="Seconds between repeated engage button presses")
  parser.add_argument("--status-interval", type=float, default=0.5, help="Print decoded Mazda CAN status every N seconds. Set 0 to disable.")
  parser.add_argument("--inject-standstill-pedals", action="store_true",
                      help="When speed reaches a full stop, replay a real stock 0x165 PEDALS standstill sequence.")
  parser.add_argument("--inject-standstill-pedals-latch", action="store_true",
                      help="After the standstill sequence, keep sending a latched stock 0x165 STANDSTILL=1/BRAKE_ON=0 frame while stopped.")
  parser.add_argument("--standstill-trigger-kph", type=float, default=0.05,
                      help="Trigger --inject-standstill-pedals once live ENGINE_DATA speed falls at or below this kph threshold.")
  parser.add_argument("--standstill-repeat", type=int, default=5,
                      help="How many times to replay the stock 0x165 standstill sequence after it triggers.")
  parser.add_argument("--stock-stopgo-21c", action="store_true",
                      help="Override 0x21c with a stock-like stop/HOLD state machine based on live speed, STANDSTILL, and resume/gas release.")
  parser.add_argument("--stock-stopgo-stopping-kph", type=float, default=3.0,
                      help="Below this speed, negative accel_cmd enters the stock stop approach 0x21c state.")
  parser.add_argument("--stock-stopgo-hold-entry-kph", type=float, default=0.35,
                      help="Below this speed, negative accel_cmd enters the stock HOLD-entry 0x21c state.")
  parser.add_argument("--stock-stopgo-holding-kph", type=float, default=0.05,
                      help="At or below this speed with STANDSTILL=1, enter the latched stock HOLD 0x21c state.")
  parser.add_argument("--stock-stopgo-hold-entry-accel-cmd", type=float, default=-700.0,
                      help="Minimum 0x21b ACCEL_CMD required to arm the HOLD-entry 0x21c state.")
  parser.add_argument("--stock-stopgo-release-seconds", type=float, default=0.75,
                      help="How long to hold the stock release 0x21c state after RES or gas is detected while latched.")
  parser.add_argument("--stock-stopgo-release-gas-threshold", type=float, default=1.0,
                      help="Treat gas above this percent as a HOLD release request in --stock-stopgo-21c mode.")
  parser.add_argument("--log-file", type=Path, default=default_log_path(), help="Local file to append runner output to so logs survive SSH disconnects")
  parser.add_argument("--sample-output", type=Path,
                      help="Optional JSON artifact path for timestamped raw-frame samples from active and handoff phases")
  parser.add_argument("--sample-addrs", type=str,
                      help="Comma-separated CAN addresses to sample into --sample-output. Defaults to 0x21b,0x21c,0x21f,0xfd,0x167,0x361-0x366,0x499")
  parser.add_argument("--sample-limit", type=int, default=80,
                      help="Maximum sampled RX frames to retain per address per phase in --sample-output")
  parser.add_argument("--all-output", action="store_true", help="Required. Sends with Panda allOutput safety mode and disables normal TX safety.")
  return parser.parse_args()


def ensure_pandad_stopped() -> None:
  try:
    check_output(["pidof", "pandad"])
    print("pandad is running, please stop openpilot before using direct Panda replay.")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1:
      raise


def load_stream(path: Path, profile_override: str | None):
  payload = json.loads(path.read_text())
  profile = MazdaLongitudinalProfile(profile_override or payload["profile"])
  return command_stream_from_hex_sequence(profile, payload["frames"])


def parse_addr_csv(value: str | None) -> tuple[int, ...]:
  if value is None or value.strip() == "":
    return DEFAULT_SAMPLE_ADDRS
  return tuple(int(part.strip(), 0) for part in value.split(",") if part.strip())


def finalize_phase_samples(samples: dict[int, list[dict[str, object]]]) -> dict[str, list[dict[str, object]]]:
  return {hex(addr): entries for addr, entries in sorted(samples.items())}


def append_sample(samples: dict[int, list[dict[str, object]]],
                  addr: int,
                  dat: bytes,
                  *,
                  phase_start: float,
                  sample_limit: int,
                  sample_addrs: set[int]) -> None:
  if addr not in sample_addrs or len(samples[addr]) >= sample_limit:
    return
  samples[addr].append({"t": round(time.monotonic() - phase_start, 6), "data": dat.hex()})


def send_tester_present_suppress_response(panda: Panda, bus: int, addr: int) -> None:
  panda.can_send(addr, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus)


def send_crz_pair(panda: Panda, bus: int, command_set) -> None:
  panda.can_send_many(
    [
      (CRZ_INFO_ADDR, command_set.raw_21b, bus),
      (CRZ_CTRL_ADDR, command_set.raw_21c, bus),
    ],
    timeout=0,
  )


def send_events(panda: Panda, bus: int, command_set) -> None:
  panda.can_send_many(
    [
      (CRZ_EVENTS_ADDR, command_set.raw_21f, bus),
    ],
    timeout=0,
  )


def send_button_press(panda: Panda, bus: int, packer: CANPacker, counter: int, button: int) -> None:
  dummy_cp = SimpleNamespace(flags=MazdaFlags.GEN1)
  addr, dat, msg_bus = create_button_cmd(packer, dummy_cp, counter, button)
  if msg_bus != bus:
    msg_bus = bus
  panda.can_send(addr, dat, msg_bus)


def send_pedals_frame(panda: Panda, bus: int, dat: bytes) -> None:
  panda.can_send(0x165, dat, bus)


def overlay_raw_21c_mode(raw: bytes, template: bytes) -> bytes:
  dat = bytearray(raw)
  for index in CRZ_CTRL_MODE_BYTE_INDEXES:
    dat[index] = template[index]
  return bytes(dat)


def apply_stock_stopgo_21c(command_set,
                           parser: CANParser | None,
                           args: argparse.Namespace,
                           now: float,
                           hold_latched: bool,
                           release_until: float) -> tuple[object, bool, float]:
  if parser is None:
    return command_set, hold_latched, release_until

  speed_kph = parser.vl["ENGINE_DATA"]["SPEED"]
  standstill = int(parser.vl["PEDALS"]["STANDSTILL"]) == 1
  gas = parser.vl["ENGINE_DATA"]["PEDAL_GAS"]
  resume_pressed = int(parser.vl["CRZ_BTNS"]["RES"]) == 1
  accel_cmd = decode_signal("CRZ_INFO", command_set.raw_21b, "ACCEL_CMD")

  if hold_latched and (resume_pressed or gas > args.stock_stopgo_release_gas_threshold or speed_kph > args.stock_stopgo_holding_kph):
    hold_latched = False
    release_until = now + args.stock_stopgo_release_seconds
  elif standstill and speed_kph <= args.stock_stopgo_holding_kph:
    hold_latched = True
    release_until = 0.0

  if hold_latched:
    raw_21c = overlay_raw_21c_mode(command_set.raw_21c, STOCK_STOPGO_CTRL_HOLD_LATCHED)
  elif release_until > now:
    raw_21c = overlay_raw_21c_mode(command_set.raw_21c, STOCK_STOPGO_CTRL_RELEASE)
  elif speed_kph <= args.stock_stopgo_hold_entry_kph and accel_cmd <= args.stock_stopgo_hold_entry_accel_cmd:
    raw_21c = overlay_raw_21c_mode(command_set.raw_21c, STOCK_STOPGO_CTRL_HOLD_ENTRY)
  elif speed_kph <= args.stock_stopgo_stopping_kph and accel_cmd < 0:
    raw_21c = overlay_raw_21c_mode(command_set.raw_21c, STOCK_STOPGO_CTRL_STOPPING)
  else:
    raw_21c = command_set.raw_21c
    if speed_kph > args.stock_stopgo_stopping_kph:
      release_until = 0.0

  return replace(command_set, raw_21c=raw_21c), hold_latched, release_until


def drain_can(panda: Panda,
              bus: int,
              parser: CANParser | None,
              rx_counts: Counter[int],
              *,
              sample_addrs: set[int] | None = None,
              sample_limit: int = 0,
              samples: dict[int, list[dict[str, object]]] | None = None,
              phase_start: float | None = None) -> Counter[int]:
  frames = []
  seen_addrs: Counter[int] = Counter()
  now = time.monotonic()
  for addr, dat, src in panda.can_recv():
    if src != bus:
      continue
    rx_counts[addr] += 1
    seen_addrs[addr] += 1
    frames.append((addr, bytes(dat), src))
    if sample_addrs and samples is not None and phase_start is not None and addr in sample_addrs and len(samples[addr]) < sample_limit:
      samples[addr].append({"t": round(now - phase_start, 6), "data": bytes(dat).hex()})
  if parser is not None and frames:
    parser.update((int(now * 1e9), frames))
  return seen_addrs


def handoff_ready(mode: str, seen_tracks: bool, stock_pair_counts: Counter[int], minimum_pair_count: int) -> bool:
  pair_ready = stock_pair_counts[CRZ_INFO_ADDR] >= minimum_pair_count and stock_pair_counts[CRZ_CTRL_ADDR] >= minimum_pair_count
  if mode == "radar_tracks":
    return seen_tracks
  if mode == "stock_pair":
    return pair_ready
  return seen_tracks and pair_ready


def print_status(prefix: str, parser: CANParser, rx_counts: Counter[int]) -> None:
  rx_crz_available = int(parser.vl["CRZ_CTRL"]["CRZ_AVAILABLE"])
  rx_crz_active = int(parser.vl["CRZ_CTRL"]["CRZ_ACTIVE"])
  has_lead = int(parser.vl["CRZ_CTRL"]["RADAR_HAS_LEAD"])
  speed_kph = parser.vl["ENGINE_DATA"]["SPEED"]
  standstill = int(parser.vl["PEDALS"]["STANDSTILL"])
  brake_on = int(parser.vl["PEDALS"]["BRAKE_ON"])
  gas = parser.vl["ENGINE_DATA"]["PEDAL_GAS"]
  crz_speed_kph = parser.vl["CRZ_EVENTS"]["CRZ_SPEED"]
  vehicle_acc_x = parser.vl["BRAKE"]["VEHICLE_ACC_X"]
  btn_counter = int(parser.vl["CRZ_BTNS"]["CTR"])
  track_total = sum(rx_counts[addr] for addr in RADAR_TRACK_ADDRS)
  print(
    f"{prefix} speed={speed_kph:.2f}kph veh_acc_x={vehicle_acc_x:.2f}m/s2 "
    f"rx_crz_available={rx_crz_available} rx_crz_active={rx_crz_active} "
    f"crz_speed={crz_speed_kph:.2f}kph standstill={standstill} brake_on={brake_on} gas={gas:.1f}% "
    f"lead={has_lead} btn_ctr={btn_counter} rx21b={rx_counts[CRZ_INFO_ADDR]} rx21c={rx_counts[CRZ_CTRL_ADDR]} "
    f"rx21f={rx_counts[0x21F]} rxfd={rx_counts[0x0FD]} rx167={rx_counts[0x167]} radar_tracks={track_total}"
  )


def print_tx_status(prefix: str, command_set) -> None:
  tx_crz_available = int(decode_signal("CRZ_CTRL", command_set.raw_21c, "CRZ_AVAILABLE"))
  tx_crz_active = int(decode_signal("CRZ_CTRL", command_set.raw_21c, "CRZ_ACTIVE"))
  tx_has_lead = int(decode_signal("CRZ_CTRL", command_set.raw_21c, "RADAR_HAS_LEAD"))
  tx_accel_cmd = decode_signal("CRZ_INFO", command_set.raw_21b, "ACCEL_CMD")
  tx_ctr1 = decode_signal("CRZ_INFO", command_set.raw_21b, "CTR1")
  tx_events_accel_cmd = decode_signal("CRZ_EVENTS", command_set.raw_21f, "ACCEL_CMD")
  tx_events_accel_low_res = decode_signal("CRZ_EVENTS", command_set.raw_21f, "ACCEL_CMD_LOW_RES")
  tx_events_ctr = decode_signal("CRZ_EVENTS", command_set.raw_21f, "CTR")
  print(
    f"{prefix} tx_crz_available={tx_crz_available} tx_crz_active={tx_crz_active} "
    f"tx_has_lead={tx_has_lead} tx_accel_cmd={tx_accel_cmd:.0f} tx_ctr1={tx_ctr1:.0f} "
    f"tx_events_accel_cmd={tx_events_accel_cmd:.0f} tx_events_accel_low_res={tx_events_accel_low_res:.0f} "
    f"tx_events_ctr={tx_events_ctr:.0f} raw21b={command_set.raw_21b.hex()} raw21c={command_set.raw_21c.hex()} raw21f={command_set.raw_21f.hex()}"
  )


def capture_radar_sequence(panda: Panda, bus: int, capture_seconds: float = 1.0) -> list[dict[int, bytes]]:
  deadline = time.monotonic() + capture_seconds
  burst: dict[int, bytes] = {}
  sequence: list[dict[int, bytes]] = []
  panda.can_clear(0xFFFF)

  while time.monotonic() < deadline:
    for addr, dat, src in panda.can_recv():
      if src == bus and addr in RADAR_REPLAY_ADDRS:
        burst[addr] = bytes(dat)
        if all(required_addr in burst for required_addr in RADAR_REPLAY_ADDRS):
          sequence.append(dict(burst))
          burst = {}
    time.sleep(0.001)

  if sequence:
    return sequence
  missing = [hex(addr) for addr in RADAR_REPLAY_ADDRS if addr not in burst]
  raise RuntimeError(f"Timed out waiting for radar replay sequence, missing {', '.join(missing)}")


def send_radar_snapshot(panda: Panda, bus: int, snapshot: dict[int, bytes]) -> None:
  panda.can_send_many([(addr, snapshot[addr], bus) for addr in RADAR_REPLAY_ADDRS], timeout=0)


def run(args: argparse.Namespace) -> None:
  if not args.all_output:
    print("--all-output is required for this script. This mode disables Panda TX safety checks.")
    sys.exit(1)
  if args.target_speed_mph is not None and not args.replace_events:
    print("--target-speed-mph requires --replace-events.")
    sys.exit(1)
  if args.target_speed_mph is not None and not args.unsafe_patch_events:
    print("--target-speed-mph requires --unsafe-patch-events because CRZ_SPEED lives in 0x21f.")
    sys.exit(1)
  if args.engage_repeat < 0:
    print("--engage-repeat must be >= 0.")
    sys.exit(1)
  if args.engage_button != "none" and args.engage_press_seconds <= 0:
    print("--engage-press-seconds must be > 0 when using --engage-button.")
    sys.exit(1)
  if args.handoff_stock_pair_count <= 0:
    print("--handoff-stock-pair-count must be > 0.")
    sys.exit(1)
  if args.standstill_repeat < 1:
    print("--standstill-repeat must be >= 1.")
    sys.exit(1)

  ensure_pandad_stopped()
  stream = load_stream(args.stream_json, args.profile)
  mutator = MazdaLongitudinalReplayMutator(stream)
  packer = CANPacker("mazda_2017")
  target_speed_kph = args.target_speed_mph * 1.609344 if args.target_speed_mph is not None else None
  sample_addrs = set(parse_addr_csv(args.sample_addrs)) if args.sample_output is not None else set()

  panda = Panda()
  panda.set_can_speed_kbps(args.bus, args.can_speed_kbps)
  panda.set_safety_mode(CarParams.SafetyModel.allOutput)
  status_parser = CANParser("mazda_2017", list(STATUS_MESSAGES), args.bus) if (args.status_interval > 0 or args.inject_standstill_pedals or args.inject_standstill_pedals_latch or args.stock_stopgo_21c) else None
  rx_counts: Counter[int] = Counter()
  phase_samples: dict[str, dict[str, list[dict[str, object]]]] = {}
  tx_phase_samples: dict[str, dict[str, list[dict[str, object]]]] = {}

  radar_sequence: list[dict[int, bytes]] | None = None
  if args.replay_pre_session_radar:
    print("Capturing pre-session radar sequence from current vehicle state.")
    radar_sequence = capture_radar_sequence(panda, args.bus, args.radar_replay_capture_seconds)
    print(f"Captured {len(radar_sequence)} radar snapshots for replay.")
    print("First radar snapshot:", " ".join(f"{hex(addr)}={dat.hex()}" for addr, dat in radar_sequence[0].items()))

  print(f"Entering radar session: addr={hex(args.radar_addr)} session={args.session}")
  client = uds.UdsClient(panda, args.radar_addr, bus=args.bus)
  client.diagnostic_session_control(SESSION_BY_NAME[args.session])
  send_tester_present_suppress_response(panda, args.bus, args.radar_addr)

  start_time = time.monotonic()
  next_send = start_time
  next_tester_present = start_time + args.tester_present_interval
  next_radar_send = start_time
  radar_sequence_index = 0
  next_status = start_time + args.status_interval if args.status_interval > 0 else float("inf")
  engage_start = start_time + args.engage_delay
  engage_button = BUTTON_BY_NAME[args.engage_button]
  engage_counter = 0
  press_end = 0.0
  next_press_start = engage_start

  print(f"Running Mazda CRZ replacement: profile={stream.profile.value} bus={args.bus} speed={args.can_speed_kbps}kbps duration={args.duration}s")
  if args.info_accel_cmd is None:
    print("Sending captured 0x21b and 0x21c only at 50 Hz while holding the radar diagnostic session.")
  else:
    print(f"Sending 0x21b and 0x21c only at 50 Hz while holding the radar diagnostic session. Overriding CRZ_INFO.ACCEL_CMD={args.info_accel_cmd}.")
  if args.replace_events:
    if args.info_accel_cmd is None:
      print("Also replaying captured 0x21f at 50 Hz, 10 ms ahead of 0x21b/0x21c.")
    else:
      patch_note = "with approximate 0x21f patching" if args.unsafe_patch_events else "with captured 0x21f preserved"
      print(f"Also sending 0x21f at 50 Hz, 10 ms ahead of 0x21b/0x21c, {patch_note}.")
  if target_speed_kph is not None:
    print(f"Target CRZ speed patch enabled: {target_speed_kph:.2f} kph ({args.target_speed_mph:.2f} mph).")
  if engage_button != Buttons.NONE:
    print(
      f"Will send {args.engage_button} cruise button pulse(s) after {args.engage_delay:.2f}s: "
      f"{args.engage_repeat} press(es), {args.engage_press_seconds:.2f}s each, {args.engage_repeat_interval:.2f}s apart."
    )
  if radar_sequence is not None:
    print(f"Replaying the pre-session radar sequence at {RADAR_REPLAY_HZ:.0f} Hz.")
  if args.stock_stopgo_21c:
    print(
      "Stock stop/HOLD 0x21c state machine enabled: "
      f"stopping<={args.stock_stopgo_stopping_kph:.2f}kph, "
      f"hold-entry<={args.stock_stopgo_hold_entry_kph:.2f}kph, "
      f"latched-hold<={args.stock_stopgo_holding_kph:.2f}kph."
    )
  print("Press Ctrl-C to stop.")
  standstill_sequence_armed = args.inject_standstill_pedals
  standstill_sequence_started = False
  standstill_sequence_index = 0
  standstill_repeat_count = 0
  next_standstill_send = start_time
  standstill_latch_active = False
  stock_stopgo_hold_latched = False
  stock_stopgo_release_until = 0.0

  try:
    phase = 0
    last_command_set = None
    active_samples: dict[int, list[dict[str, object]]] = defaultdict(list)
    active_tx_samples: dict[int, list[dict[str, object]]] = defaultdict(list)
    while args.duration == 0 or (time.monotonic() - start_time) < args.duration:
      now = time.monotonic()
      drain_can(
        panda,
        args.bus,
        status_parser,
        rx_counts,
        sample_addrs=sample_addrs,
        sample_limit=args.sample_limit,
        samples=active_samples,
        phase_start=start_time,
      )
      if standstill_sequence_armed and status_parser is not None and not standstill_sequence_started:
        if status_parser.vl["ENGINE_DATA"]["SPEED"] <= args.standstill_trigger_kph:
          standstill_sequence_started = True
          standstill_sequence_index = 0
          standstill_repeat_count = 0
          next_standstill_send = now
          print(
            f"Starting standstill PEDALS injection: speed={status_parser.vl['ENGINE_DATA']['SPEED']:.2f} kph "
            f"repeat={args.standstill_repeat}."
          )
      if standstill_sequence_started and now >= next_standstill_send:
        send_pedals_frame(panda, args.bus, STANDSTILL_PEDALS_SEQUENCE[standstill_sequence_index])
        if sample_addrs:
          append_sample(active_tx_samples, 0x165, STANDSTILL_PEDALS_SEQUENCE[standstill_sequence_index],
                        phase_start=start_time, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
        standstill_sequence_index += 1
        next_standstill_send = now + 0.01
        if standstill_sequence_index >= len(STANDSTILL_PEDALS_SEQUENCE):
          standstill_sequence_index = 0
          standstill_repeat_count += 1
          if standstill_repeat_count >= args.standstill_repeat:
            standstill_sequence_started = False
            standstill_sequence_armed = False
            standstill_latch_active = args.inject_standstill_pedals_latch
            if standstill_latch_active:
              print("Completed standstill PEDALS injection sequence; entering latched STANDSTILL replay.")
            else:
              print("Completed standstill PEDALS injection sequence.")
      elif standstill_latch_active and now >= next_standstill_send:
        if status_parser is not None and status_parser.vl["ENGINE_DATA"]["SPEED"] > args.standstill_trigger_kph:
          standstill_latch_active = False
          print("Stopping latched STANDSTILL replay because vehicle speed rose above the trigger threshold.")
        else:
          send_pedals_frame(panda, args.bus, STANDSTILL_PEDALS_LATCH_FRAME)
          if sample_addrs:
            append_sample(active_tx_samples, 0x165, STANDSTILL_PEDALS_LATCH_FRAME,
                          phase_start=start_time, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
          next_standstill_send = now + 0.02
      if now >= next_tester_present:
        send_tester_present_suppress_response(panda, args.bus, args.radar_addr)
        next_tester_present = now + args.tester_present_interval
      if radar_sequence is not None and now >= next_radar_send:
        current_snapshot = radar_sequence[radar_sequence_index]
        send_radar_snapshot(panda, args.bus, current_snapshot)
        if sample_addrs:
          for replay_addr, replay_dat in current_snapshot.items():
            append_sample(active_tx_samples, replay_addr, replay_dat, phase_start=start_time, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
        radar_sequence_index = (radar_sequence_index + 1) % len(radar_sequence)
        next_radar_send = now + (1.0 / RADAR_REPLAY_HZ)
      if engage_button != Buttons.NONE and engage_counter < args.engage_repeat:
        if now >= next_press_start and press_end == 0.0:
          press_end = now + args.engage_press_seconds
          engage_counter += 1
          print(f"Starting cruise button pulse {engage_counter}/{args.engage_repeat}: {args.engage_button}.")
        if press_end > 0.0:
          btn_counter = int(status_parser.vl["CRZ_BTNS"]["CTR"]) if status_parser is not None else 0
          send_button_press(panda, args.bus, packer, btn_counter, engage_button)
          if now >= press_end:
            press_end = 0.0
            next_press_start = now + args.engage_repeat_interval

      if args.replace_events and phase == 0:
        last_command_set = mutator.next_command_set(
          profile=stream.profile,
          info_accel_cmd=args.info_accel_cmd,
          crz_speed_kph=target_speed_kph,
          unsafe_patch_events=args.unsafe_patch_events,
        )
        if args.stock_stopgo_21c:
          last_command_set, stock_stopgo_hold_latched, stock_stopgo_release_until = apply_stock_stopgo_21c(
            last_command_set,
            status_parser,
            args,
            now,
            stock_stopgo_hold_latched,
            stock_stopgo_release_until,
          )
        send_events(panda, args.bus, last_command_set)
        if sample_addrs:
          append_sample(active_tx_samples, CRZ_EVENTS_ADDR, last_command_set.raw_21f,
                        phase_start=start_time, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
        phase = 1
        next_send += 0.01
      else:
        if not args.replace_events:
          last_command_set = mutator.next_command_set(
            profile=stream.profile,
            info_accel_cmd=args.info_accel_cmd,
            crz_speed_kph=target_speed_kph,
            unsafe_patch_events=False,
          )
          if args.stock_stopgo_21c:
            last_command_set, stock_stopgo_hold_latched, stock_stopgo_release_until = apply_stock_stopgo_21c(
              last_command_set,
              status_parser,
              args,
              now,
              stock_stopgo_hold_latched,
              stock_stopgo_release_until,
            )
        if last_command_set is None:
          raise RuntimeError("No command set available for CRZ pair send")
        elif args.replace_events and args.stock_stopgo_21c:
          last_command_set, stock_stopgo_hold_latched, stock_stopgo_release_until = apply_stock_stopgo_21c(
            last_command_set,
            status_parser,
            args,
            now,
            stock_stopgo_hold_latched,
            stock_stopgo_release_until,
          )
        send_crz_pair(panda, args.bus, last_command_set)
        if sample_addrs:
          append_sample(active_tx_samples, CRZ_INFO_ADDR, last_command_set.raw_21b,
                        phase_start=start_time, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
          append_sample(active_tx_samples, CRZ_CTRL_ADDR, last_command_set.raw_21c,
                        phase_start=start_time, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
        phase = 0
        next_send += 0.02 if not args.replace_events else 0.01

      sleep_time = next_send - time.monotonic()
      if sleep_time > 0:
        time.sleep(sleep_time)
      else:
        next_send = time.monotonic()
      if status_parser is not None and now >= next_status:
        print_status(f"t+{now - start_time:.1f}s", status_parser, rx_counts)
        if last_command_set is not None:
          print_tx_status(f"t+{now - start_time:.1f}s", last_command_set)
        next_status = now + args.status_interval

    if args.handoff_seconds > 0:
      if sample_addrs:
        phase_samples["active"] = finalize_phase_samples(active_samples)
        tx_phase_samples["active"] = finalize_phase_samples(active_tx_samples)
      print(f"Starting exit handoff for up to {args.handoff_seconds:.1f}s.")
      print(
        f"Handoff will end on {args.handoff_end_on} "
        f"(stock-pair threshold={args.handoff_stock_pair_count})."
      )
      handoff_deadline = time.monotonic() + args.handoff_seconds
      next_handoff_send = time.monotonic()
      next_handoff_radar_send = time.monotonic()
      handoff_radar_sequence_index = radar_sequence_index
      next_handoff_status = time.monotonic() + args.status_interval if args.status_interval > 0 else float("inf")
      last_command_set = mutator.next_command_set(
        profile=stream.profile,
        info_accel_cmd=args.info_accel_cmd,
        crz_speed_kph=target_speed_kph,
        unsafe_patch_events=args.unsafe_patch_events if args.replace_events else False,
      )
      if args.stock_stopgo_21c:
        last_command_set, stock_stopgo_hold_latched, stock_stopgo_release_until = apply_stock_stopgo_21c(
          last_command_set,
          status_parser,
          args,
          time.monotonic(),
          stock_stopgo_hold_latched,
          stock_stopgo_release_until,
        )
      handoff_phase = 0
      stock_pair_counts: Counter[int] = Counter()
      tracks_seen = False
      announced_tracks = False
      handoff_samples: dict[int, list[dict[str, object]]] = defaultdict(list)
      handoff_tx_samples: dict[int, list[dict[str, object]]] = defaultdict(list)
      handoff_start = time.monotonic()

      while time.monotonic() < handoff_deadline:
        now = time.monotonic()
        handoff_seen = drain_can(
          panda,
          args.bus,
          status_parser,
          rx_counts,
          sample_addrs=sample_addrs,
          sample_limit=args.sample_limit,
          samples=handoff_samples,
          phase_start=handoff_start,
        )
        if any(addr in handoff_seen for addr in RADAR_TRACK_ADDRS):
          tracks_seen = True
          if not announced_tracks:
            print("Radar tracks detected during handoff.")
            announced_tracks = True
        stock_pair_counts[CRZ_INFO_ADDR] += handoff_seen[CRZ_INFO_ADDR]
        stock_pair_counts[CRZ_CTRL_ADDR] += handoff_seen[CRZ_CTRL_ADDR]
        if handoff_ready(args.handoff_end_on, tracks_seen, stock_pair_counts, args.handoff_stock_pair_count):
          if args.handoff_end_on == "radar_tracks":
            print("Ending handoff: radar tracks detected again.")
          elif args.handoff_end_on == "stock_pair":
            print(
              "Ending handoff: stock 0x21b/0x21c restored "
              f"({stock_pair_counts[CRZ_INFO_ADDR]}/{stock_pair_counts[CRZ_CTRL_ADDR]} frames seen)."
            )
          else:
            print(
              "Ending handoff: radar tracks and stock 0x21b/0x21c restored "
              f"({stock_pair_counts[CRZ_INFO_ADDR]}/{stock_pair_counts[CRZ_CTRL_ADDR]} frames seen)."
            )
          break
        if now >= next_handoff_send:
          if args.replace_events and handoff_phase == 0:
            send_events(panda, args.bus, last_command_set)
            if sample_addrs:
              append_sample(handoff_tx_samples, CRZ_EVENTS_ADDR, last_command_set.raw_21f,
                            phase_start=handoff_start, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
            handoff_phase = 1
            next_handoff_send = now + 0.01
          else:
            if args.stock_stopgo_21c:
              last_command_set, stock_stopgo_hold_latched, stock_stopgo_release_until = apply_stock_stopgo_21c(
                last_command_set,
                status_parser,
                args,
                now,
                stock_stopgo_hold_latched,
                stock_stopgo_release_until,
              )
            send_crz_pair(panda, args.bus, last_command_set)
            if sample_addrs:
              append_sample(handoff_tx_samples, CRZ_INFO_ADDR, last_command_set.raw_21b,
                            phase_start=handoff_start, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
              append_sample(handoff_tx_samples, CRZ_CTRL_ADDR, last_command_set.raw_21c,
                            phase_start=handoff_start, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
            handoff_phase = 0
            next_handoff_send = now + (0.02 if not args.replace_events else 0.01)
        if radar_sequence is not None and now >= next_handoff_radar_send:
          current_snapshot = radar_sequence[handoff_radar_sequence_index]
          send_radar_snapshot(panda, args.bus, current_snapshot)
          if sample_addrs:
            for replay_addr, replay_dat in current_snapshot.items():
              append_sample(handoff_tx_samples, replay_addr, replay_dat,
                            phase_start=handoff_start, sample_limit=args.sample_limit, sample_addrs=sample_addrs)
          handoff_radar_sequence_index = (handoff_radar_sequence_index + 1) % len(radar_sequence)
          next_handoff_radar_send = now + (1.0 / RADAR_REPLAY_HZ)
        if status_parser is not None and now >= next_handoff_status:
          print_status(f"handoff+{args.handoff_seconds - (handoff_deadline - now):.1f}s", status_parser, rx_counts)
          print_tx_status(f"handoff+{args.handoff_seconds - (handoff_deadline - now):.1f}s", last_command_set)
          next_handoff_status = now + args.status_interval
        time.sleep(0.001)
      if sample_addrs:
        phase_samples["handoff"] = finalize_phase_samples(handoff_samples)
        tx_phase_samples["handoff"] = finalize_phase_samples(handoff_tx_samples)
    elif sample_addrs:
      phase_samples["active"] = finalize_phase_samples(active_samples)
      tx_phase_samples["active"] = finalize_phase_samples(active_tx_samples)
    if args.exit_default_session:
      try:
        print("Requesting radar return to UDS default session.")
        client.diagnostic_session_control(uds.SESSION_TYPE.DEFAULT)
        print("Radar default-session request accepted.")
      except Exception as e:
        print(f"Radar default-session request failed: {e!r}")
    if args.sample_output is not None:
      args.sample_output.parent.mkdir(parents=True, exist_ok=True)
      sample_payload = {
        "stream_json": str(args.stream_json),
        "profile": stream.profile.value,
        "bus": args.bus,
        "session": args.session,
        "duration": args.duration,
        "handoff_seconds": args.handoff_seconds,
        "radar_addr": hex(args.radar_addr),
        "replay_pre_session_radar": args.replay_pre_session_radar,
        "radar_replay_capture_seconds": args.radar_replay_capture_seconds,
        "sample_limit": args.sample_limit,
        "sample_addrs": [hex(addr) for addr in sorted(sample_addrs)],
        "captured_radar_sequence": [
          {hex(addr): dat.hex() for addr, dat in snapshot.items()}
          for snapshot in (radar_sequence or [])
        ],
        "phases": phase_samples,
        "tx_phases": tx_phase_samples,
      }
      args.sample_output.write_text(json.dumps(sample_payload, indent=2) + "\n")
      print(f"Wrote phase samples JSON to {args.sample_output}")
  finally:
    panda.set_safety_mode(CarParams.SafetyModel.silent)
    print("Panda returned to silent safety mode.")


def main() -> None:
  args = parse_args()
  args.log_file.parent.mkdir(parents=True, exist_ok=True)
  with args.log_file.open("a", buffering=1) as log_fp:
    stdout = sys.stdout
    stderr = sys.stderr
    sys.stdout = TeeStream(stdout, log_fp)
    sys.stderr = TeeStream(stderr, log_fp)
    try:
      print()
      print("=" * 80)
      print(f"Started at {dt.datetime.now().isoformat()} with argv: {' '.join(sys.argv)}")
      print(f"Logging to {args.log_file}")
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
