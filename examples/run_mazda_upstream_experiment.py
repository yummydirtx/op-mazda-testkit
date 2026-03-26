#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ExperimentPreset:
  description: str
  replace_events: bool = False
  unsafe_patch_events: bool = False


EXPERIMENT_PRESETS: dict[str, ExperimentPreset] = {
  "session_only": ExperimentPreset(
    description="Hold the radar in programming session and prove which frames disappear without replacement.",
  ),
  "pair": ExperimentPreset(
    description="Replace only 0x21b and 0x21c while holding the radar session.",
  ),
  "pair_events_replay": ExperimentPreset(
    description="Replace 0x21b and 0x21c, and replay captured 0x21f at stock phase cadence.",
    replace_events=True,
  ),
  "pair_events_patch": ExperimentPreset(
    description="Replace 0x21b and 0x21c, replay 0x21f, and enable approximate 0x21f patching when overriding accel.",
    replace_events=True,
    unsafe_patch_events=True,
  ),
}

SESSION_CHOICES: tuple[str, ...] = ("default", "programming", "extended", "safety")
BUTTON_CHOICES: tuple[str, ...] = ("none", "set-plus", "set-minus", "resume", "cancel")


def script_path(name: str) -> Path:
  return Path(__file__).resolve().with_name(name)


def default_output_dir(experiment: str) -> Path:
  if Path("/data/mazda").exists():
    root = Path("/data/mazda/experiments")
  elif Path("/data/tmp").exists():
    root = Path("/data/tmp/mazda_experiments")
  else:
    root = Path("/tmp/mazda_experiments")

  stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
  return root / f"{stamp}_{experiment}"


def shell_join(argv: list[str]) -> str:
  return " ".join(shlex.quote(arg) for arg in argv)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Run the upstream-path Mazda longitudinal experiment ladder with consistent logs."
  )
  parser.add_argument("stream_json", type=Path, help="JSON file produced by extract_mazda_longitudinal_replay.py")
  parser.add_argument("--experiment", choices=tuple(EXPERIMENT_PRESETS), default="pair",
                      help="Which rung of the upstream-path ladder to execute")
  parser.add_argument("--profile", help="Optional profile override passed through to the replacement runner")
  parser.add_argument("--output-dir", type=Path, help="Directory to store experiment artifacts")
  parser.add_argument("--notes", help="Optional free-form note stored in summary.json")

  parser.add_argument("--bus", type=int, default=0, help="CAN bus for UDS and replay traffic")
  parser.add_argument("--can-speed-kbps", type=int, default=500, help="CAN bus speed in kbps")
  parser.add_argument("--radar-addr", type=lambda x: int(x, 0), default=0x764, help="Radar UDS address")
  parser.add_argument("--session", choices=SESSION_CHOICES, default="programming",
                      help="Diagnostic session to hold on the radar ECU")

  parser.add_argument("--probe-before-seconds", type=float, default=3.0,
                      help="Seconds to count target frames before entering the radar session in the session-only probe")
  parser.add_argument("--probe-during-seconds", type=float, default=3.0,
                      help="Seconds to hold tester-present active during the session-only probe")
  parser.add_argument("--probe-progress-interval", type=float, default=0.0,
                      help="Print a progress line every N seconds during the session-only hold; 0 disables")

  parser.add_argument("--duration", type=float, default=5.0,
                      help="Replacement-run duration in seconds, 0 for infinite")
  parser.add_argument("--tester-present-interval", type=float, default=0.5,
                      help="Seconds between raw 0x3E80 tester-present frames")
  parser.add_argument("--handoff-seconds", type=float, default=2.0,
                      help="After stopping tester-present, keep sending replacement traffic for this long")
  parser.add_argument("--status-interval", type=float, default=0.5,
                      help="Print decoded RX/TX status every N seconds during replacement; 0 disables")

  parser.add_argument("--info-accel-cmd", type=float,
                      help="Optional CRZ_INFO.ACCEL_CMD override passed to the replacement runner")
  parser.add_argument("--target-speed-mph", type=float,
                      help="Optional target CRZ speed patch in mph; requires the selected experiment to send 0x21f")
  parser.add_argument("--replay-pre-session-radar", action="store_true",
                      help="Capture the parked radar burst before suppression and replay it at 10 Hz during replacement")

  parser.add_argument("--engage-button", choices=BUTTON_CHOICES, default="none",
                      help="Optional Mazda cruise button pulse after suppression starts")
  parser.add_argument("--engage-delay", type=float, default=0.75,
                      help="Seconds after suppression begins before the first engage button pulse")
  parser.add_argument("--engage-press-seconds", type=float, default=0.20,
                      help="Length of each engage button press")
  parser.add_argument("--engage-repeat", type=int, default=1,
                      help="How many engage button presses to send")
  parser.add_argument("--engage-repeat-interval", type=float, default=0.75,
                      help="Seconds between repeated engage button presses")

  parser.add_argument("--skip-before-dtc", action="store_true",
                      help="Do not snapshot DTCs before the experiment")
  parser.add_argument("--skip-probe", action="store_true",
                      help="Do not run the session-only suppression probe before replacement")
  parser.add_argument("--skip-after-clear", action="store_true",
                      help="Do not run the post-experiment DTC clear/readback step")
  parser.add_argument("--include-functional-clear", action="store_true",
                      help="Also send a functional 0x7DF clear in the post-experiment DTC cleanup step")
  return parser.parse_args()


def write_summary(path: Path, payload: dict) -> None:
  path.write_text(json.dumps(payload, indent=2) + "\n")


def run_step(name: str, argv: list[str], summary: dict, *, log_path: Path) -> None:
  print()
  print("=" * 80)
  print(f"{name}")
  print(shell_join(argv))
  result = subprocess.run(argv, check=False)
  summary["steps"].append({
    "name": name,
    "argv": argv,
    "command": shell_join(argv),
    "log_path": str(log_path),
    "returncode": result.returncode,
  })
  if result.returncode != 0:
    raise SystemExit(result.returncode)


def build_dtc_read_cmd(args: argparse.Namespace, output: Path) -> list[str]:
  argv = [
    sys.executable,
    str(script_path("clear_mazda_warnings.py")),
    "--bus", str(args.bus),
    "--read-only",
    "--output", str(output),
  ]
  return argv


def build_probe_cmd(args: argparse.Namespace, output: Path) -> list[str]:
  return [
    sys.executable,
    str(script_path("test_mazda_radar_disable_with_panda.py")),
    "--bus", str(args.bus),
    "--addr", hex(args.radar_addr),
    "--can-speed-kbps", str(args.can_speed_kbps),
    "--before-seconds", str(args.probe_before_seconds),
    "--during-seconds", str(args.probe_during_seconds),
    "--progress-interval", str(args.probe_progress_interval),
    "--session", args.session,
    "--session-only",
    "--allow-aeb-risk",
    "--output", str(output),
  ]


def build_replacement_cmd(args: argparse.Namespace, preset: ExperimentPreset, output: Path) -> list[str]:
  argv = [
    sys.executable,
    str(script_path("run_mazda_crz_replacement_with_panda.py")),
    str(args.stream_json),
    "--bus", str(args.bus),
    "--can-speed-kbps", str(args.can_speed_kbps),
    "--duration", str(args.duration),
    "--session", args.session,
    "--radar-addr", hex(args.radar_addr),
    "--tester-present-interval", str(args.tester_present_interval),
    "--handoff-seconds", str(args.handoff_seconds),
    "--status-interval", str(args.status_interval),
    "--log-file", str(output),
    "--all-output",
  ]

  if args.profile:
    argv.extend(["--profile", args.profile])
  if args.info_accel_cmd is not None:
    argv.extend(["--info-accel-cmd", str(args.info_accel_cmd)])
  if args.target_speed_mph is not None:
    argv.extend(["--target-speed-mph", str(args.target_speed_mph)])
  if args.replay_pre_session_radar:
    argv.append("--replay-pre-session-radar")
  if preset.replace_events:
    argv.append("--replace-events")
  if preset.unsafe_patch_events:
    argv.append("--unsafe-patch-events")
  if args.engage_button != "none":
    argv.extend([
      "--engage-button", args.engage_button,
      "--engage-delay", str(args.engage_delay),
      "--engage-press-seconds", str(args.engage_press_seconds),
      "--engage-repeat", str(args.engage_repeat),
      "--engage-repeat-interval", str(args.engage_repeat_interval),
    ])

  return argv


def build_dtc_clear_cmd(args: argparse.Namespace, output: Path) -> list[str]:
  argv = [
    sys.executable,
    str(script_path("clear_mazda_warnings.py")),
    "--bus", str(args.bus),
    "--output", str(output),
  ]
  if args.include_functional_clear:
    argv.append("--include-functional")
  return argv


def main() -> None:
  args = parse_args()
  preset = EXPERIMENT_PRESETS[args.experiment]

  if not args.stream_json.is_file():
    raise FileNotFoundError(args.stream_json)
  if args.target_speed_mph is not None and not preset.replace_events:
    raise SystemExit("--target-speed-mph requires an experiment that sends 0x21f.")

  output_dir = (args.output_dir or default_output_dir(args.experiment)).resolve()
  output_dir.mkdir(parents=True, exist_ok=True)

  summary = {
    "created_at": dt.datetime.now().isoformat(),
    "experiment": args.experiment,
    "description": preset.description,
    "stream_json": str(args.stream_json.resolve()),
    "profile_override": args.profile,
    "notes": args.notes,
    "output_dir": str(output_dir),
    "steps": [],
  }
  summary_path = output_dir / "summary.json"
  write_summary(summary_path, summary)

  print(f"Experiment: {args.experiment}")
  print(preset.description)
  print(f"Artifacts: {output_dir}")
  print(f"Summary:   {summary_path}")

  if not args.skip_before_dtc:
    dtc_before = output_dir / "01_dtc_before.log"
    run_step("DTC snapshot before experiment", build_dtc_read_cmd(args, dtc_before), summary, log_path=dtc_before)
    write_summary(summary_path, summary)

  if not args.skip_probe:
    probe_log = output_dir / "02_probe_session_only.log"
    run_step("Session-only suppression probe", build_probe_cmd(args, probe_log), summary, log_path=probe_log)
    write_summary(summary_path, summary)

  if args.experiment != "session_only":
    replacement_log = output_dir / "03_replacement.log"
    run_step("Replacement runner", build_replacement_cmd(args, preset, replacement_log), summary, log_path=replacement_log)
    write_summary(summary_path, summary)

  if not args.skip_after_clear:
    dtc_after = output_dir / "04_dtc_after_clear.log"
    run_step("Post-experiment DTC clear and re-read", build_dtc_clear_cmd(args, dtc_after), summary, log_path=dtc_after)
    write_summary(summary_path, summary)

  print()
  print("Completed successfully.")
  print(f"Review artifacts under: {output_dir}")


if __name__ == "__main__":
  main()
