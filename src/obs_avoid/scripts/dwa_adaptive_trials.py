#!/usr/bin/env python3
"""Adaptive DWA trial runner.

Runs repeated simulation trials and tunes DWA parameters between trials based
on monitor failure reasons.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import shlex
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


DEFAULT_PROFILE: dict[str, Any] = {
    "v_max": 2.5,
    "vy_max": 2.0,
    "collision_radius": 0.32,
    "safety_radius": 0.80,
    "hard_clearance_margin": 0.05,
    "final_cmd_clearance_margin": 0.08,
    "w_clearance": 1.2,
    "w_progress": 2.5,
    "w_speed": 0.2,
    "w_yaw_rate": 0.25,
    "full_search_w_max": 0.70,
    "nx_samples": 9,
    "ny_samples": 7,
    "nw_samples": 11,
    "control_dt": 0.05,
    "sim_dt": 0.10,
    "horizon_sec": 2.4,
    "fallback_strafe_speed": 0.32,
    "fallback_forward_speed": 0.18,
    "bypass_strafe_speed": 0.45,
    "bypass_forward_speed": 0.10,
    "recovery_side_lock_sec": 1.2,
    "recovery_side_hysteresis": 0.40,
    "steer_first_forward_speed": 0.12,
    "face_goal_mix_with_dwa": 0.45,
}

INT_PARAM_KEYS = {
    "nx_samples",
    "ny_samples",
    "nw_samples",
}


def log(msg: str) -> None:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] {msg}", flush=True)


def run_shell(
    cmd: str,
    *,
    env: dict[str, str] | None = None,
    check: bool = False,
    capture_output: bool = False,
    cwd: str | None = None,
) -> subprocess.CompletedProcess[str]:
    merged = os.environ.copy()
    if env:
        merged.update(env)
    return subprocess.run(
        ["/bin/bash", "-lc", cmd],
        text=True,
        env=merged,
        cwd=cwd,
        capture_output=capture_output,
        check=check,
    )


def clamp(profile: dict[str, Any], key: str, lo: float, hi: float) -> None:
    profile[key] = max(lo, min(hi, float(profile[key])))


def as_ros_args(profile: dict[str, Any]) -> str:
    def format_float(v: float) -> str:
        s = f"{float(v):.6f}".rstrip("0").rstrip(".")
        if "." not in s and "e" not in s.lower():
            s += ".0"
        return s

    parts: list[str] = []
    for key in sorted(profile.keys()):
        value = profile[key]
        if isinstance(value, bool):
            token = "true" if value else "false"
        elif key in INT_PARAM_KEYS:
            token = str(value)
        else:
            token = format_float(float(value))
        parts.append(f"-p {key}:={token}")
    return " ".join(parts)


def tune_profile(
    profile: dict[str, Any],
    control: dict[str, Any],
    reason: str,
    repeats: int,
) -> list[str]:
    notes: list[str] = []
    gain = min(2.0, 1.0 + 0.25 * max(0, repeats - 1))

    if reason == "pred_clear_critical_consecutive":
        profile["collision_radius"] += 0.03 * gain
        profile["safety_radius"] += 0.05 * gain
        profile["hard_clearance_margin"] += 0.01 * gain
        profile["final_cmd_clearance_margin"] += 0.02 * gain
        profile["w_clearance"] *= 1.20
        profile["v_max"] *= 0.90
        profile["vy_max"] *= 0.90
        notes.append("increased clearance safety and reduced max speeds")

    elif reason == "planner_cmd_gap_consecutive":
        profile["nx_samples"] -= int(round(2 * gain))
        profile["ny_samples"] -= int(round(2 * gain))
        profile["nw_samples"] -= int(round(2 * gain))
        profile["control_dt"] += 0.01 * gain
        profile["sim_dt"] += 0.01 * gain
        profile["horizon_sec"] -= 0.20 * gain
        notes.append("reduced DWA compute load and widened timing step")

    elif reason == "no_goal_progress_timeout":
        profile["w_progress"] *= 1.25
        profile["w_speed"] *= 1.20
        profile["w_yaw_rate"] *= 0.85
        profile["full_search_w_max"] += 0.08 * gain
        profile["fallback_forward_speed"] += 0.05 * gain
        profile["bypass_forward_speed"] += 0.05 * gain
        profile["face_goal_mix_with_dwa"] -= 0.08 * gain
        notes.append("increased progress drive and reduced over-turn bias")

    elif reason == "recovery_streak_timeout":
        profile["recovery_side_lock_sec"] -= 0.20 * gain
        profile["recovery_side_hysteresis"] -= 0.08 * gain
        profile["fallback_strafe_speed"] += 0.05 * gain
        profile["bypass_strafe_speed"] += 0.06 * gain
        profile["steer_first_forward_speed"] += 0.04 * gain
        profile["full_search_w_max"] += 0.10 * gain
        notes.append("made recovery mode more decisive and less sticky")

    elif reason in {
        "startup_timeout",
        "missing_pose",
        "missing_planner_cmd",
        "missing_dwa_state",
        "missing_goal_or_inferred_d_goal",
    } or reason.startswith("launcher_rc_"):
        control["obs_avoid_delay_sec"] = min(45, int(control["obs_avoid_delay_sec"]) + 5)
        control["readiness_timeout_sec"] = min(90, int(control["readiness_timeout_sec"]) + 10)
        notes.append("extended startup delays and readiness timeout")

    else:
        notes.append("no tuning rule matched; kept profile unchanged")

    clamp(profile, "collision_radius", 0.20, 0.50)
    clamp(profile, "safety_radius", 0.60, 1.20)
    clamp(profile, "hard_clearance_margin", 0.01, 0.20)
    clamp(profile, "final_cmd_clearance_margin", 0.03, 0.25)
    clamp(profile, "w_clearance", 0.5, 4.0)
    clamp(profile, "v_max", 0.8, 3.5)
    clamp(profile, "vy_max", 0.5, 3.0)

    profile["nx_samples"] = int(max(5, min(11, int(profile["nx_samples"]))))
    profile["ny_samples"] = int(max(5, min(11, int(profile["ny_samples"]))))
    profile["nw_samples"] = int(max(7, min(13, int(profile["nw_samples"]))))
    clamp(profile, "control_dt", 0.03, 0.10)
    clamp(profile, "sim_dt", 0.05, 0.20)
    clamp(profile, "horizon_sec", 1.2, 3.2)
    clamp(profile, "w_progress", 1.0, 6.0)
    clamp(profile, "w_speed", 0.05, 1.2)
    clamp(profile, "w_yaw_rate", 0.05, 0.8)
    clamp(profile, "full_search_w_max", 0.4, 1.2)
    clamp(profile, "fallback_strafe_speed", 0.15, 0.75)
    clamp(profile, "fallback_forward_speed", 0.05, 0.45)
    clamp(profile, "bypass_strafe_speed", 0.2, 0.8)
    clamp(profile, "bypass_forward_speed", 0.05, 0.4)
    clamp(profile, "recovery_side_lock_sec", 0.2, 2.0)
    clamp(profile, "recovery_side_hysteresis", 0.05, 0.8)
    clamp(profile, "steer_first_forward_speed", 0.02, 0.4)
    clamp(profile, "face_goal_mix_with_dwa", 0.05, 0.9)
    return notes


def safe_write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as fp:
        json.dump(payload, fp, indent=2, sort_keys=True)
        fp.write("\n")
    os.replace(tmp, path)


def wait_for_tmux_session(session: str, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        p = run_shell(f"tmux has-session -t {shlex.quote(session)}")
        if p.returncode == 0:
            return True
        time.sleep(1.0)
    return False


def wait_for_topic(ros_setup: str, topic: str, timeout: float) -> bool:
    deadline = time.time() + timeout
    cmd = (
        f"set +u; source {shlex.quote(ros_setup)}; set -u; "
        f"ros2 topic list 2>/dev/null | grep -qx {shlex.quote(topic)}"
    )
    while time.time() < deadline:
        p = run_shell(cmd)
        if p.returncode == 0:
            return True
        time.sleep(1.0)
    return False


def capture_panes(session: str, out_dir: Path) -> None:
    pane_map = {
        "pane_dwa.log": f"{session}:obs_avoid.1",
        "pane_spiral.log": f"{session}:obs_avoid.0",
        "pane_px4.log": f"{session}:sim.0",
    }
    for fname, pane in pane_map.items():
        cmd = (
            f"tmux capture-pane -pt {shlex.quote(pane)} -S -200000 > "
            f"{shlex.quote(str(out_dir / fname))}"
        )
        run_shell(cmd)


def teardown(session: str) -> None:
    run_shell(f"tmux kill-session -t {shlex.quote(session)} || true")
    for pat in (
        "make px4_sitl gz_x500_lidar_2d_tilted",
        "ros2 launch mavros px4.launch",
        "ros2 run ros_gz_bridge parameter_bridge",
        "ros2 run obs_avoid local_planner_mode_a",
        "ros2 run obs_avoid spiral_mapping_mode",
    ):
        run_shell(f"pkill -f {shlex.quote(pat)} >/dev/null 2>&1 || true")


def launch_stack(
    script_dir: Path,
    session: str,
    world: str,
    ros_args: str,
    obs_avoid_delay_sec: int,
) -> int:
    cmd = (
        f"cd {shlex.quote(str(script_dir.parent))} && "
        f"SESSION={shlex.quote(session)} USE_TMUX=1 ATTACH_SESSION=0 "
        "RECREATE_SESSION=1 KILL_BEFORE_LAUNCH=1 ENABLE_MISSION_OBS_AVOID=0 "
        f"WORLD_NAME={shlex.quote(world)} OBS_AVOID_DELAY_SEC={int(obs_avoid_delay_sec)} "
        f"DWA_ROS_ARGS={shlex.quote(ros_args)} "
        "./scripts/start_sim_stack_terminator.sh"
    )
    proc = run_shell(cmd)
    return proc.returncode


def start_spiral(ros_setup: str, session: str, side: float, layers: int, z_step: float) -> None:
    run_cmd = (
        "set +u; "
        f"source {shlex.quote(ros_setup)}; "
        "set -u; "
        "ros2 run obs_avoid spiral_mapping_mode --ros-args "
        "-p use_sim_time:=true "
        "-p ask_mission_on_start:=false "
        "-p auto_start_default_mission:=true "
        f"-p default_square_size_m:={side} "
        f"-p default_z_layers:={layers} "
        f"-p default_z_step_m:={z_step}"
    )
    cmd = (
        f"tmux send-keys -t {shlex.quote(session + ':obs_avoid.0')} "
        f"{shlex.quote(run_cmd)} C-m"
    )
    run_shell(cmd)


def run_monitor(
    monitor_script: str,
    trial_root: Path,
    world: str,
    *,
    time_cap_sec: int,
    update_period_sec: int,
    readiness_timeout_sec: int,
    log_path: Path,
) -> int:
    cmd = (
        "set -o pipefail; "
        f"BENCH_ROOT={shlex.quote(str(trial_root))} "
        f"WORLD={shlex.quote(world)} "
        f"TIME_CAP_SEC={int(time_cap_sec)} "
        f"UPDATE_PERIOD_SEC={int(update_period_sec)} "
        f"READINESS_TIMEOUT_SEC={int(readiness_timeout_sec)} "
        f"{shlex.quote(monitor_script)} | tee {shlex.quote(str(log_path))}"
    )
    proc = run_shell(cmd)
    return proc.returncode


def pick_best_trial(trials: list[dict[str, Any]]) -> dict[str, Any]:
    def score(item: dict[str, Any]) -> tuple[int, float]:
        result = item.get("result", "unknown")
        elapsed = float(item.get("elapsed_sec", 0.0) or 0.0)
        if result == "cap_reached":
            return (3, elapsed)
        if result == "external_stop":
            return (2, elapsed)
        if result == "failed":
            return (1, elapsed)
        return (0, elapsed)

    best = max(trials, key=score)
    return best


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Adaptive DWA multi-trial runner")
    parser.add_argument("--worlds", default="walls,forest,windy")
    parser.add_argument("--trials-per-world", type=int, default=3)
    parser.add_argument("--bench-root", default=f"/tmp/dwa_adaptive_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    parser.add_argument("--monitor-script", default="/tmp/dwa_progress_monitor.py")
    parser.add_argument("--ros-setup", default="/home/lehaitrung/ros2_ws/install/setup.bash")
    parser.add_argument("--time-cap-sec", type=int, default=900)
    parser.add_argument("--update-period-sec", type=int, default=5)
    parser.add_argument("--readiness-timeout-sec", type=int, default=30)
    parser.add_argument("--obs-avoid-delay-sec", type=int, default=15)
    parser.add_argument("--spiral-side", type=float, default=40.0)
    parser.add_argument("--spiral-layers", type=int, default=2)
    parser.add_argument("--spiral-z-step", type=float, default=2.0)
    parser.add_argument("--session-prefix", default="dwa_adapt")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not Path(args.monitor_script).exists():
        print(f"[error] monitor script not found: {args.monitor_script}", file=sys.stderr)
        return 2

    script_dir = Path(__file__).resolve().parent
    bench_root = Path(args.bench_root)
    bench_root.mkdir(parents=True, exist_ok=True)

    worlds = [w.strip() for w in args.worlds.split(",") if w.strip()]
    if not worlds:
        print("[error] no worlds selected", file=sys.stderr)
        return 2

    scorecard: list[dict[str, Any]] = []
    global_history: dict[str, Any] = {"bench_root": str(bench_root), "worlds": {}}

    for world in worlds:
        log(f"=== world={world} adaptive trials start ===")
        profile = copy.deepcopy(DEFAULT_PROFILE)
        control = {
            "obs_avoid_delay_sec": int(args.obs_avoid_delay_sec),
            "readiness_timeout_sec": int(args.readiness_timeout_sec),
        }
        reason_counts: dict[str, int] = {}
        world_trials: list[dict[str, Any]] = []

        for trial_idx in range(1, args.trials_per_world + 1):
            trial_name = f"trial_{trial_idx:02d}"
            session = f"{args.session_prefix}_{world}_t{trial_idx:02d}"
            trial_root = bench_root / world / trial_name
            trial_root.mkdir(parents=True, exist_ok=True)

            ros_args = as_ros_args(profile)
            safe_write_json(
                trial_root / "profile_used.json",
                {
                    "world": world,
                    "trial": trial_idx,
                    "profile": profile,
                    "control": control,
                    "dwa_ros_args": ros_args,
                },
            )

            log(
                f"[{world}/{trial_name}] launch stack session={session} "
                f"delay={control['obs_avoid_delay_sec']} ros_args='{ros_args}'"
            )
            rc = launch_stack(script_dir, session, world, ros_args, int(control["obs_avoid_delay_sec"]))
            if rc != 0:
                log(f"[{world}/{trial_name}] launch failed rc={rc}")
                trial_result = {
                    "world": world,
                    "trial": trial_idx,
                    "result": "launch_failed",
                    "reason": f"launcher_rc_{rc}",
                    "elapsed_sec": 0.0,
                    "profile": copy.deepcopy(profile),
                    "control": copy.deepcopy(control),
                }
                world_trials.append(trial_result)
                reason_counts[trial_result["reason"]] = reason_counts.get(trial_result["reason"], 0) + 1
                notes = tune_profile(profile, control, "startup_timeout", reason_counts[trial_result["reason"]])
                safe_write_json(trial_root / "tuning_notes.json", {"notes": notes})
                teardown(session)
                time.sleep(2.0)
                continue

            if not wait_for_tmux_session(session, timeout=25.0):
                log(f"[{world}/{trial_name}] tmux session did not become ready")
                teardown(session)
                continue

            start_spiral(
                args.ros_setup,
                session,
                float(args.spiral_side),
                int(args.spiral_layers),
                float(args.spiral_z_step),
            )

            for topic in ("/mavros/local_position/pose", "/planner_cmd_vel", "/dwa/state"):
                ok = wait_for_topic(args.ros_setup, topic, timeout=60.0)
                if not ok:
                    log(f"[{world}/{trial_name}] topic wait timed out: {topic}")
                    break

            monitor_log = trial_root / "monitor.stdout.log"
            mon_rc = run_monitor(
                args.monitor_script,
                trial_root,
                world,
                time_cap_sec=int(args.time_cap_sec),
                update_period_sec=int(args.update_period_sec),
                readiness_timeout_sec=int(control["readiness_timeout_sec"]),
                log_path=monitor_log,
            )
            log(f"[{world}/{trial_name}] monitor exit rc={mon_rc}")

            progress_dir = trial_root / world / "progress"
            summary_path = progress_dir / "summary.json"
            summary: dict[str, Any] = {}
            if summary_path.exists():
                try:
                    summary = json.loads(summary_path.read_text(encoding="utf-8"))
                except Exception:
                    summary = {}

            capture_panes(session, trial_root)
            teardown(session)
            time.sleep(3.0)

            result = summary.get("result", "unknown")
            reason = summary.get("reason", f"monitor_rc_{mon_rc}")
            elapsed = float(summary.get("elapsed_sec", 0.0) or 0.0)

            trial_result = {
                "world": world,
                "trial": trial_idx,
                "result": result,
                "reason": reason,
                "elapsed_sec": elapsed,
                "monitor_exit_code": mon_rc,
                "profile": copy.deepcopy(profile),
                "control": copy.deepcopy(control),
                "summary_path": str(summary_path),
            }
            world_trials.append(trial_result)
            safe_write_json(trial_root / "trial_result.json", trial_result)

            if result == "cap_reached":
                log(f"[{world}/{trial_name}] cap reached; keeping profile and moving on.")
                continue

            reason_counts[reason] = reason_counts.get(reason, 0) + 1
            notes = tune_profile(profile, control, reason, reason_counts[reason])
            safe_write_json(
                trial_root / "tuning_notes.json",
                {
                    "reason": reason,
                    "reason_repeats": reason_counts[reason],
                    "notes": notes,
                    "next_profile": profile,
                    "next_control": control,
                },
            )
            log(f"[{world}/{trial_name}] tuned next profile due to reason='{reason}': {'; '.join(notes)}")

        best = pick_best_trial(world_trials) if world_trials else {}
        world_summary = {
            "world": world,
            "trials": world_trials,
            "best_trial": best,
            "final_profile": profile,
            "final_control": control,
        }
        global_history["worlds"][world] = world_summary
        safe_write_json(bench_root / world / "adaptive_world_summary.json", world_summary)

        if best:
            scorecard.append(
                {
                    "world": world,
                    "best_trial": best.get("trial"),
                    "result": best.get("result"),
                    "reason": best.get("reason"),
                    "elapsed_sec": best.get("elapsed_sec"),
                }
            )
        log(f"=== world={world} adaptive trials complete ===")

    safe_write_json(bench_root / "adaptive_history.json", global_history)
    safe_write_json(bench_root / "adaptive_scorecard.json", scorecard)
    log(f"adaptive scorecard: {bench_root / 'adaptive_scorecard.json'}")
    print(json.dumps(scorecard, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
