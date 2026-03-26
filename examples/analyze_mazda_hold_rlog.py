#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from opendbc.car.logreader import LogReader


DEFAULT_ADDRS: tuple[int, ...] = (0x50, 0x9A, 0x9B, 0x121, 0x274, 0x420, 0x436, 0x477)
KNOWN_LABELS: dict[int, str] = {
  0x50: "MSG_04",
  0x9A: "0x9A",
  0x9B: "MSG_14",
  0x121: "EPB",
  0x274: "0x274",
  0x420: "CHECK_AND_TEMP",
  0x436: "0x436",
  0x477: "BSM",
}


@dataclass(frozen=True)
class Change:
  t_s: float
  address: int
  before: str | None
  after: str


@dataclass(frozen=True)
class Cluster:
  start_s: float
  end_s: float
  changes: tuple[Change, ...]


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Analyze Mazda manual HOLD candidate CAN transitions from one or more rlogs.")
  parser.add_argument("rlogs", nargs="+", type=Path, help="Path(s) to rlog.zst files")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus/src to analyze")
  parser.add_argument("--cluster-gap", type=float, default=0.20, help="Seconds between changes before starting a new cluster")
  parser.add_argument("--max-clusters", type=int, default=20, help="Maximum number of clusters to print per log")
  parser.add_argument("--addrs", type=str,
                      help="Comma-separated CAN addresses to analyze. Defaults to 0x50,0x9a,0x9b,0x121,0x274,0x420,0x436,0x477")
  return parser.parse_args()


def parse_addrs(value: str | None) -> tuple[int, ...]:
  if value is None or value.strip() == "":
    return DEFAULT_ADDRS
  return tuple(int(part.strip(), 0) for part in value.split(",") if part.strip())


def changed_bytes(before: bytes, after: bytes) -> list[int]:
  return [i for i, (lhs, rhs) in enumerate(zip(before, after, strict=True)) if lhs != rhs]


def changed_bits(before: bytes, after: bytes) -> list[int]:
  out: list[int] = []
  for byte_index, (lhs, rhs) in enumerate(zip(before, after, strict=True)):
    diff = lhs ^ rhs
    for bit in range(8):
      if diff & (1 << bit):
        out.append(byte_index * 8 + bit)
  return out


def label_addr(address: int) -> str:
  label = KNOWN_LABELS.get(address)
  return f"{hex(address)} ({label})" if label is not None else hex(address)


def collect_changes(path: Path, bus: int, addrs: set[int]) -> tuple[list[Change], float]:
  if not path.is_file():
    raise FileNotFoundError(path)

  first_t = None
  last_t = None
  last_payload: dict[int, str] = {}
  changes: list[Change] = []

  for event in LogReader(str(path), only_union_types=True):
    if event.which() != "can":
      continue

    t_s = event.logMonoTime * 1e-9
    if first_t is None:
      first_t = t_s
    last_t = t_s

    rel_t = t_s - first_t
    for can_msg in event.can:
      if can_msg.src != bus or can_msg.address not in addrs:
        continue
      raw = bytes(can_msg.dat).hex()
      previous = last_payload.get(can_msg.address)
      if previous != raw:
        changes.append(Change(t_s=rel_t, address=can_msg.address, before=previous, after=raw))
        last_payload[can_msg.address] = raw

  if first_t is None or last_t is None:
    raise RuntimeError(f"No CAN data found in {path}")
  return changes, last_t - first_t


def cluster_changes(changes: list[Change], cluster_gap: float) -> list[Cluster]:
  if not changes:
    return []

  clusters: list[list[Change]] = [[changes[0]]]
  for change in changes[1:]:
    if (change.t_s - clusters[-1][-1].t_s) <= cluster_gap:
      clusters[-1].append(change)
    else:
      clusters.append([change])

  return [
    Cluster(start_s=cluster[0].t_s, end_s=cluster[-1].t_s, changes=tuple(cluster))
    for cluster in clusters
  ]


def print_cluster(cluster: Cluster) -> None:
  print(f"  cluster {cluster.start_s:.3f}s -> {cluster.end_s:.3f}s ({len(cluster.changes)} changes)")
  for change in cluster.changes:
    if change.before is None:
      print(f"    {label_addr(change.address):18} init -> {change.after}")
      continue
    before_b = bytes.fromhex(change.before)
    after_b = bytes.fromhex(change.after)
    print(
      f"    {label_addr(change.address):18} {change.before} -> {change.after} "
      f"bytes={changed_bytes(before_b, after_b)} bits={changed_bits(before_b, after_b)}"
    )


def print_summary(path: Path, duration_s: float, changes: list[Change], clusters: list[Cluster], max_clusters: int) -> None:
  print(f"{path.name}: duration={duration_s:.3f}s candidate_changes={len(changes)} clusters={len(clusters)}")

  by_addr: dict[int, int] = defaultdict(int)
  for change in changes:
    by_addr[change.address] += 1

  print("  per-address changes:")
  for address in sorted(by_addr):
    print(f"    {label_addr(address):18} {by_addr[address]}")

  print("  clusters:")
  for cluster in clusters[:max_clusters]:
    print_cluster(cluster)
  if len(clusters) > max_clusters:
    print(f"  ... truncated {len(clusters) - max_clusters} additional clusters")


def main() -> None:
  args = parse_args()
  addrs = set(parse_addrs(args.addrs))
  for index, path in enumerate(args.rlogs):
    if index:
      print()
    changes, duration_s = collect_changes(path, args.bus, addrs)
    clusters = cluster_changes(changes, args.cluster_gap)
    print_summary(path, duration_s, changes, clusters, args.max_clusters)


if __name__ == "__main__":
  main()
