from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum

from opendbc.car.can_definitions import CanData
from opendbc.can.dbc import DBC
from opendbc.can.packer import set_value
from opendbc.can.parser import get_raw_value


MAZDA_LONG_DBC = DBC("mazda_2017")

CRZ_INFO_ADDR = 0x21B
CRZ_CTRL_ADDR = 0x21C
CRZ_EVENTS_ADDR = 0x21F
GAS_ADDR = 0x0FD
MORE_GAS_ADDR = 0x167

CRZ_CTRL_MODE_BYTE_INDEXES: tuple[int, ...] = (0, 2, 3, 6)


class MazdaLongitudinalProfile(str, Enum):
  STANDBY = "standby"
  ENGAGED_CRUISE = "engaged_cruise"
  ENGAGED_FOLLOW = "engaged_follow"
  STOP_GO_HOLD = "stop_go_hold"


CRZ_CTRL_MODE_TEMPLATES: dict[MazdaLongitudinalProfile, bytes] = {
  MazdaLongitudinalProfile.STANDBY: bytes.fromhex("02010b0000000000"),
  MazdaLongitudinalProfile.ENGAGED_CRUISE: bytes.fromhex("0a018b2000001000"),
  MazdaLongitudinalProfile.ENGAGED_FOLLOW: bytes.fromhex("0a018b4000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD: bytes.fromhex("0a018b6000001000"),
}


@dataclass(frozen=True)
class MazdaLongitudinalCommandSet:
  profile: MazdaLongitudinalProfile | None
  raw_21b: bytes
  raw_21c: bytes
  raw_21f: bytes
  raw_fd: bytes
  raw_167: bytes | None = None

  def as_can_msgs(self, bus: int = 0) -> list[tuple[int, bytes, int]]:
    messages = [
      (CRZ_INFO_ADDR, self.raw_21b, bus),
      (CRZ_CTRL_ADDR, self.raw_21c, bus),
      (CRZ_EVENTS_ADDR, self.raw_21f, bus),
      (GAS_ADDR, self.raw_fd, bus),
    ]
    if self.raw_167 is not None:
      messages.append((MORE_GAS_ADDR, self.raw_167, bus))
    return messages


@dataclass(frozen=True)
class MazdaLongitudinalCommandStream:
  profile: MazdaLongitudinalProfile
  commands: tuple[MazdaLongitudinalCommandSet, ...]

  def __post_init__(self) -> None:
    if len(self.commands) == 0:
      raise ValueError("MazdaLongitudinalCommandStream requires at least one command set")

  def __len__(self) -> int:
    return len(self.commands)


def command_set_from_hex_dict(profile: MazdaLongitudinalProfile | None, values: dict[str, str]) -> MazdaLongitudinalCommandSet:
  return MazdaLongitudinalCommandSet(
    profile=profile,
    raw_21b=bytes.fromhex(values["CRZ_INFO"]),
    raw_21c=bytes.fromhex(values["CRZ_CTRL"]),
    raw_21f=bytes.fromhex(values["CRZ_EVENTS"]),
    raw_fd=bytes.fromhex(values["GAS"]),
    raw_167=bytes.fromhex(values["MORE_GAS"]) if values.get("MORE_GAS") else None,
  )


def command_stream_from_hex_sequence(profile: MazdaLongitudinalProfile, sequence: list[dict[str, str]]) -> MazdaLongitudinalCommandStream:
  return MazdaLongitudinalCommandStream(
    profile=profile,
    commands=tuple(command_set_from_hex_dict(profile, values) for values in sequence),
  )


def _get_signal(message_name: str, signal_name: str):
  return MAZDA_LONG_DBC.name_to_msg[message_name].sigs[signal_name]


def decode_signal(message_name: str, raw: bytes, signal_name: str) -> float:
  sig = _get_signal(message_name, signal_name)
  raw_value = get_raw_value(raw, sig)
  if sig.is_signed:
    raw_value -= ((raw_value >> (sig.size - 1)) & 0x1) * (1 << sig.size)
  return raw_value * sig.factor + sig.offset


def patch_signal(message_name: str, raw: bytes, signal_name: str, value: float) -> bytes:
  sig = _get_signal(message_name, signal_name)
  encoded = int(math.floor((value - sig.offset) / sig.factor + 0.5))
  if encoded < 0:
    encoded = (1 << sig.size) + encoded

  dat = bytearray(raw)
  set_value(dat, sig, encoded)
  return bytes(dat)


def compute_inverted_sum_checksum(raw: bytes, checksum_index: int = 7) -> int:
  return (0xFF - (sum(raw[i] for i in range(len(raw)) if i != checksum_index) & 0xFF)) & 0xFF


def compute_crz_info_checksum_guess(raw: bytes) -> int:
  return compute_inverted_sum_checksum(raw, checksum_index=7)


def update_crz_info_checksum_guess(raw: bytes) -> bytes:
  dat = bytearray(raw)
  dat[7] = compute_crz_info_checksum_guess(dat)
  return bytes(dat)


def matches_crz_info_checksum_guess(raw: bytes) -> bool:
  return len(raw) == 8 and raw[7] == compute_crz_info_checksum_guess(raw)


def overlay_crz_ctrl_mode(raw: bytes, profile: MazdaLongitudinalProfile) -> bytes:
  template = CRZ_CTRL_MODE_TEMPLATES[profile]
  dat = bytearray(raw)
  for index in CRZ_CTRL_MODE_BYTE_INDEXES:
    dat[index] = template[index]
  return bytes(dat)


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def approximate_crz_events_accel_cmd(info_accel_cmd: float) -> int:
  return int(clip(round(info_accel_cmd / 8.0), -128, 127))


def approximate_crz_events_accel_low_res(info_accel_cmd: float) -> int:
  return int(clip(round(info_accel_cmd / 32.0) * 4, -128, 124))


def build_command_set(template: MazdaLongitudinalCommandSet,
                      profile: MazdaLongitudinalProfile | None = None,
                      *,
                      crz_info_accel_cmd: float | None = None,
                      crz_info_ctr1: float | None = None,
                      gas_cmd: float | None = None,
                      gas_ctr: float | None = None,
                      more_gas_ctr: float | None = None,
                      crz_events_accel_cmd: float | None = None,
                      crz_events_accel_low_res: float | None = None,
                      crz_events_crz_speed: float | None = None,
                      crz_events_ctr: float | None = None,
                      apply_crz_info_checksum_guess: bool = True,
                      allow_unsafe_events_patching: bool = False) -> MazdaLongitudinalCommandSet:
  raw_21b = template.raw_21b
  raw_21c = template.raw_21c
  raw_21f = template.raw_21f
  raw_fd = template.raw_fd
  raw_167 = template.raw_167

  if profile is not None:
    raw_21c = overlay_crz_ctrl_mode(raw_21c, profile)

  if crz_info_accel_cmd is not None:
    raw_21b = patch_signal("CRZ_INFO", raw_21b, "ACCEL_CMD", crz_info_accel_cmd)
  if crz_info_ctr1 is not None:
    raw_21b = patch_signal("CRZ_INFO", raw_21b, "CTR1", crz_info_ctr1)
  if apply_crz_info_checksum_guess and (crz_info_accel_cmd is not None or crz_info_ctr1 is not None):
    raw_21b = update_crz_info_checksum_guess(raw_21b)

  if gas_cmd is not None:
    raw_fd = patch_signal("GAS", raw_fd, "GAS_CMD", gas_cmd)
  if gas_ctr is not None:
    raw_fd = patch_signal("GAS", raw_fd, "CTR", gas_ctr)

  if raw_167 is not None and more_gas_ctr is not None:
    raw_167 = patch_signal("MORE_GAS", raw_167, "CTR", more_gas_ctr)

  if any(value is not None for value in (crz_events_accel_cmd, crz_events_accel_low_res, crz_events_crz_speed, crz_events_ctr)):
    if not allow_unsafe_events_patching:
      raise NotImplementedError("CRZ_EVENTS checksum semantics are still unresolved; pass allow_unsafe_events_patching=True to patch anyway.")
    if crz_events_accel_cmd is not None:
      raw_21f = patch_signal("CRZ_EVENTS", raw_21f, "ACCEL_CMD", crz_events_accel_cmd)
    if crz_events_accel_low_res is not None:
      raw_21f = patch_signal("CRZ_EVENTS", raw_21f, "ACCEL_CMD_LOW_RES", crz_events_accel_low_res)
    if crz_events_crz_speed is not None:
      raw_21f = patch_signal("CRZ_EVENTS", raw_21f, "CRZ_SPEED", crz_events_crz_speed)
    if crz_events_ctr is not None:
      raw_21f = patch_signal("CRZ_EVENTS", raw_21f, "CTR", crz_events_ctr)

  return replace(template, profile=profile or template.profile, raw_21b=raw_21b, raw_21c=raw_21c, raw_21f=raw_21f, raw_fd=raw_fd, raw_167=raw_167)


class MazdaLongitudinalReplayMutator:
  def __init__(self, stream: MazdaLongitudinalCommandStream):
    self.stream = stream
    self.index = 0
    self.phase = 0
    self.pending_command_set: MazdaLongitudinalCommandSet | None = None

  def reset(self) -> None:
    self.index = 0
    self.phase = 0
    self.pending_command_set = None

  def next_command_set(self,
                       *,
                       profile: MazdaLongitudinalProfile | None = None,
                       info_accel_cmd: float | None = None,
                       gas_cmd: float | None = None,
                       unsafe_patch_events: bool = False) -> MazdaLongitudinalCommandSet:
    base = self.stream.commands[self.index]
    self.index = (self.index + 1) % len(self.stream.commands)

    crz_events_accel_cmd = None
    crz_events_accel_low_res = None
    if unsafe_patch_events and info_accel_cmd is not None:
      crz_events_accel_cmd = approximate_crz_events_accel_cmd(info_accel_cmd)
      crz_events_accel_low_res = approximate_crz_events_accel_low_res(info_accel_cmd)

    return build_command_set(
      base,
      profile=profile,
      crz_info_accel_cmd=info_accel_cmd,
      gas_cmd=gas_cmd,
      crz_events_accel_cmd=crz_events_accel_cmd,
      crz_events_accel_low_res=crz_events_accel_low_res,
      allow_unsafe_events_patching=unsafe_patch_events,
    )

  def next_can_data(self,
                    *,
                    profile: MazdaLongitudinalProfile | None = None,
                    info_accel_cmd: float | None = None,
                    gas_cmd: float | None = None,
                    unsafe_patch_events: bool = False,
                    bus: int = 0) -> list[CanData]:
    command_set = self.next_command_set(
      profile=profile,
      info_accel_cmd=info_accel_cmd,
      gas_cmd=gas_cmd,
      unsafe_patch_events=unsafe_patch_events,
    )
    return [CanData(addr, dat, src) for addr, dat, src in command_set.as_can_msgs(bus)]

  def next_phase_can_data(self,
                          *,
                          profile: MazdaLongitudinalProfile | None = None,
                          info_accel_cmd: float | None = None,
                          gas_cmd: float | None = None,
                          unsafe_patch_events: bool = False,
                          bus: int = 0) -> list[CanData]:
    if self.phase == 0:
      self.pending_command_set = self.next_command_set(
        profile=profile,
        info_accel_cmd=info_accel_cmd,
        gas_cmd=gas_cmd,
        unsafe_patch_events=unsafe_patch_events,
      )
      self.phase = 1
      messages: list[tuple[int, bytes, int]] = [
        (GAS_ADDR, self.pending_command_set.raw_fd, bus),
        (CRZ_EVENTS_ADDR, self.pending_command_set.raw_21f, bus),
      ]
      if self.pending_command_set.raw_167 is not None:
        messages.insert(1, (MORE_GAS_ADDR, self.pending_command_set.raw_167, bus))
    else:
      if self.pending_command_set is None:
        raise RuntimeError("phase B requested before phase A")
      self.phase = 0
      messages = [
        (CRZ_INFO_ADDR, self.pending_command_set.raw_21b, bus),
        (CRZ_CTRL_ADDR, self.pending_command_set.raw_21c, bus),
      ]
      self.pending_command_set = None

    return [CanData(addr, dat, src) for addr, dat, src in messages]
