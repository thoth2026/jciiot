"""Standalone grasp strategy tester for JCIIOT FactorySorting scenes.

This script is similar in spirit to the Streamlit "Grasp Test" panel, but it
uses the robot-agent PickUpSkill path so we can test competition-side changes
without running the full move-pick-move-place task.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Any


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app-dir", default=str(_default_app_dir()))
    parser.add_argument("--task-index", type=int, default=0, help="0=L1, 1=L2, ... 4=L5")
    parser.add_argument("--object-name", default="", help="Override object name")
    parser.add_argument("--source", default="", help="Override source station")
    parser.add_argument("--mode", choices=["skill", "geometric"], default="skill")
    parser.add_argument("--offsets", default="0,0", help="Semicolon-separated dx,dy offsets, e.g. '0,0;0.05,0;-0.05,0'")
    parser.add_argument("--yaw-offsets", default="0", help="Semicolon-separated yaw offsets in radians")
    parser.add_argument("--timestamp", default=time.strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--out-dir", default="recordings/grasp_sweep")
    parser.add_argument("--no-record", action="store_true")
    args = parser.parse_args(argv)

    app_dir = Path(args.app_dir).resolve()
    _configure_paths(app_dir)

    report = run_sweep(args, app_dir)
    out_dir = (app_dir / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    report_path = out_dir / f"grasp_sweep_{args.timestamp}.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
    print(f"[grasp_sweep] wrote {report_path}")
    return 0 if any(item.get("success") for item in report.get("trials", [])) else 2


def run_sweep(args: argparse.Namespace, app_dir: Path) -> dict[str, Any]:
    task_cfg = _load_json(app_dir / "knowledge" / "task_config.json")
    tasks = task_cfg.get("tasks", [])
    if not (0 <= args.task_index < len(tasks)):
        raise RuntimeError(f"Invalid task-index {args.task_index}; expected 0-{len(tasks)-1}")

    task = tasks[args.task_index]
    source = args.source or task["source"]
    object_name = args.object_name or _primary_object_name(task.get("object"))
    semantic, _grid_file = _choose_map_files(app_dir, task["scene_prefix"])
    from robot_agent.core.map_loader import load_map_files
    from robot_agent.core.scene_context import SceneContext

    scene, _grid = load_map_files(semantic, _grid_file)
    scene_ctx = SceneContext.from_semantic_map(scene)
    grasp_pose = task_cfg.get("grasp_poses", {}).get(source, {})
    try:
        approach_xy = scene_ctx.approach_xy(source)
        base_pos = [float(approach_xy[0]), float(approach_xy[1]), 0.0]
        pose_source = "semantic_map_approach"
    except Exception:
        base_pos = [float(v) for v in grasp_pose.get("pos", [0.0, 0.0, 0.0])]
        pose_source = "task_config_grasp_pose"
    yaw = float(grasp_pose.get("yaw", -3.139453))

    offsets = _parse_xy_offsets(args.offsets)
    yaw_offsets = _parse_float_list(args.yaw_offsets)
    trials = []
    for dx, dy in offsets:
        for dyaw in yaw_offsets:
            trial_pose = {
                "xy": [base_pos[0] + dx, base_pos[1] + dy],
                "yaw": yaw + dyaw,
                "robot_base_pos": [base_pos[0] + dx, base_pos[1] + dy, base_pos[2] if len(base_pos) > 2 else 0.0],
                "robot_base_ori": [0.0, 0.0, yaw + dyaw],
            }
            trials.append(_run_one_trial(
                app_dir=app_dir,
                args=args,
                task=task,
                source=source,
                object_name=object_name,
                initial_base_pose=trial_pose,
            ))

    return {
        "timestamp": args.timestamp,
        "mode": args.mode,
        "task_index": args.task_index,
        "level": task.get("level"),
        "env_name": task.get("env_name"),
        "source": source,
        "object_name": object_name,
        "base_pose": {"pos": base_pos, "yaw": yaw, "source": pose_source},
        "offsets": offsets,
        "yaw_offsets": yaw_offsets,
        "trials": trials,
    }


def _run_one_trial(
    *,
    app_dir: Path,
    args: argparse.Namespace,
    task: dict[str, Any],
    source: str,
    object_name: str,
    initial_base_pose: dict[str, Any],
) -> dict[str, Any]:
    from robot_agent.core.map_loader import load_map_files
    from robot_agent.core.scene_context import SceneContext
    from robot_agent.core.types import ExecutionContext
    from robot_agent.environments import RobosuiteBackend
    from robot_agent.skills.pick_up import PickUpSkill, _geometric_site_grasp_fallback

    env_name = task["env_name"]
    semantic, grid_file = _choose_map_files(app_dir, task["scene_prefix"])
    scene, grid = load_map_files(semantic, grid_file)
    scene_ctx = SceneContext.from_semantic_map(scene)
    backend = None
    stream = io.StringIO()
    start = time.perf_counter()
    trajectory_path = ""
    try:
        with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
            backend = RobosuiteBackend(env_name=env_name, camera="birdview", drive_mode="direct")
            backend._scene_context = scene_ctx
            backend.reset()
            object_map = _build_object_map(backend, task, source, object_name)
            backend.set_physics_grasp_config(device="cpu", object_map=object_map)
            backend.reset()
            _set_backend_base_pose(backend, initial_base_pose)

            if not args.no_record:
                backend.start_recording()
                with contextlib.suppress(Exception):
                    backend._record_trajectory_frame()

            if args.mode == "geometric":
                result_payload = _geometric_site_grasp_fallback(backend, source, object_name)
                success = bool(result_payload.get("ok"))
                message = "geometric fallback direct"
            else:
                skill = PickUpSkill(backend=backend, scene_context=scene_ctx)
                result = skill.run(ExecutionContext(
                    task=f"test grasp {object_name} at {source}",
                    metadata={
                        "inputs": {
                            "target": source,
                            "object_name": object_name,
                            "grasp_initial_base_pose": initial_base_pose,
                        }
                    },
                ))
                success = bool(result.success)
                message = result.message
                result_payload = dict(result.payload)

            try:
                backend._record_trajectory_frame()
            except Exception:
                pass

            if not args.no_record:
                out_dir = (app_dir / args.out_dir).resolve()
                out_dir.mkdir(parents=True, exist_ok=True)
                status = "OK" if success else "FAIL"
                stem = (
                    f"{args.timestamp}_{task.get('level', 'L')}_"
                    f"{args.mode}_{source}_{object_name}_{status}"
                )
                trajectory_path = backend.save_trajectory(out_dir / f"{stem}.json")

        return {
            "success": success,
            "message": message,
            "initial_base_pose": initial_base_pose,
            "payload": result_payload,
            "trajectory": trajectory_path,
            "elapsed_sec": round(time.perf_counter() - start, 3),
            "log": stream.getvalue()[-12000:],
        }
    except Exception as exc:
        return {
            "success": False,
            "message": f"{type(exc).__name__}: {exc}",
            "initial_base_pose": initial_base_pose,
            "elapsed_sec": round(time.perf_counter() - start, 3),
            "traceback": traceback.format_exc(),
            "log": stream.getvalue()[-12000:],
        }
    finally:
        if backend is not None:
            with contextlib.suppress(Exception):
                backend.close()


def _default_app_dir() -> Path:
    return Path(__file__).resolve().parents[3]


def _configure_paths(app_dir: Path) -> None:
    for path in (
        app_dir / "src",
        app_dir,
        app_dir / "robomimic",
        app_dir / "robosuite" / "robosuite",
    ):
        s = str(path)
        if s not in sys.path:
            sys.path.insert(0, s)

    robosuite_inner = app_dir / "robosuite" / "robosuite" / "__init__.py"
    if robosuite_inner.exists():
        import robosuite as rs_patch

        rs_patch.__file__ = str(robosuite_inner)
        rs_patch.__path__ = [str(robosuite_inner.parent)]
        with open(robosuite_inner, encoding="utf-8") as handle:
            code = compile(handle.read(), str(robosuite_inner), "exec")
        exec(code, rs_patch.__dict__)


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _choose_map_files(app_dir: Path, scene_prefix: str) -> tuple[Path, Path]:
    map_dir = app_dir / "robosuite" / "robosuite" / "environments" / "factory_sorting" / "generated_maps"
    return (
        map_dir / f"{scene_prefix}_scene_regenerated_semantic_map.json",
        map_dir / f"{scene_prefix}_scene_regenerated_occupancy_grid.npy",
    )


def _primary_object_name(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, (list, tuple)):
        for item in value:
            if item:
                return str(item)
    return ""


def _build_object_map(backend: Any, task: dict[str, Any], source: str, object_name: str) -> dict[str, str]:
    object_map = {}
    for obj, info in (getattr(backend.env, "material_metadata", {}) or {}).items():
        if not isinstance(info, dict):
            continue
        port = str(info.get("port_name") or "")
        if not port:
            continue
        object_map[port] = obj
        if port.startswith("input_"):
            object_map["line_" + port.split("_", 1)[1]] = obj
        elif port.startswith("line_"):
            object_map["input_" + port.split("_", 1)[1]] = obj
    object_map[source] = object_name
    task_source = task.get("source")
    if task_source:
        object_map[str(task_source)] = object_name
    return object_map


def _set_backend_base_pose(backend: Any, pose: dict[str, Any]) -> None:
    import numpy as np
    from robot_agent.environments.robosuite_backend import (
        _set_base_world_yaw_direct,
        _set_base_xy_direct,
    )

    env = backend.env
    robot = env.robots[0]
    yaw = float(pose.get("yaw", 0.0))
    xy = pose.get("xy") or pose.get("base_world_xy")
    if xy is None:
        robot_base_pos = pose.get("robot_base_pos") or [0.0, 0.0, 0.0]
        xy = robot_base_pos[:2]
    _set_base_world_yaw_direct(env, robot, yaw)
    _set_base_xy_direct(env, robot, np.asarray(xy, dtype=float))
    env.sim.forward()


def _parse_xy_offsets(value: str) -> list[tuple[float, float]]:
    offsets = []
    for chunk in value.split(";"):
        if not chunk.strip():
            continue
        parts = [float(x.strip()) for x in chunk.split(",")]
        if len(parts) != 2:
            raise RuntimeError(f"Bad offset '{chunk}', expected dx,dy")
        offsets.append((parts[0], parts[1]))
    return offsets or [(0.0, 0.0)]


def _parse_float_list(value: str) -> list[float]:
    vals = [float(x.strip()) for x in value.split(";") if x.strip()]
    return vals or [0.0]


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
