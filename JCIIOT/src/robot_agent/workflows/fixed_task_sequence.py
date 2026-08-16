"""Run a fixed move-pick-move-place sequence for FactorySorting debugging.

This bypasses the LLM planner but uses the same agent, backend, maps, skills,
and trajectory recording path as the Streamlit task subprocess. It is intended
for validating grasp/place behavior when the local LLM service is unavailable.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app-dir", default=str(_default_app_dir()))
    parser.add_argument("--task-index", type=int, default=0, help="0=L1, 1=L2, ... 4=L5")
    parser.add_argument("--source", default="", help="Override source station")
    parser.add_argument("--target", default="", help="Override target station")
    parser.add_argument("--object-name", default="", help="Override object name")
    parser.add_argument("--all-objects", action="store_true", help="Transport every object listed for the task")
    parser.add_argument("--timestamp", default=time.strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--knowledge-enabled", default="true")
    args = parser.parse_args(argv)

    app_dir = Path(args.app_dir).resolve()
    _configure_paths(app_dir)
    result = run_fixed_sequence(args, app_dir)
    print(json.dumps(result, indent=2, ensure_ascii=False, default=str))
    return 0 if result.get("success") else 2


def run_fixed_sequence(args: argparse.Namespace, app_dir: Path) -> dict[str, Any]:
    from robot_agent.task_subprocess_runner import _build_agent, _scene_env_name

    task_cfg = _load_json(app_dir / "knowledge" / "task_config.json")
    tasks = task_cfg.get("tasks", [])
    if not (0 <= args.task_index < len(tasks)):
        raise RuntimeError(f"Invalid task-index {args.task_index}; expected 0-{len(tasks)-1}")

    task = tasks[args.task_index]
    source = args.source or task["source"]
    target = args.target or task["target"]
    objects = _object_names(task.get("object"))
    if args.object_name:
        objects = [args.object_name]
    elif not args.all_objects:
        objects = objects[:1]
    if not objects:
        raise RuntimeError("No object name available")

    # Keep the competition app behavior by default: use our geometric site grasp
    # before the weak demo BC policy tries to move the arms into the table.
    os.environ.setdefault("JCIIOT_GEOMETRIC_GRASP_FIRST", "1")
    os.environ.setdefault("JCIIOT_VISUAL_SITE_GRASP", "1")
    os.environ.setdefault("JCIIOT_STRICT_PHYSICS_GRASP", "1")
    os.environ.setdefault("JCIIOT_ALLOW_GRASP_QPOS_FALLBACK", "0")
    os.environ.setdefault("JCIIOT_SITE_GRASP_BASE_NUDGE", "1")
    os.environ.setdefault("JCIIOT_ACCEPT_CONTACT_LIFT_GRASP", "1")
    os.environ.setdefault("JCIIOT_STRICT_CARRY_ACTION", "0")
    os.environ.setdefault("JCIIOT_POST_PICK_RETREAT_M", "0.65")
    agent = _build_agent(
        app_dir,
        int(args.task_index),
        knowledge_enabled=str(args.knowledge_enabled).lower() in {"1", "true", "yes", "on"},
    )

    env_name = _scene_env_name(int(args.task_index))
    rec_dir = app_dir / "recordings" / env_name
    rec_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    outputs = []
    trajectory_path = ""
    status = "FAIL"
    try:
        agent.backend.start_recording()
        for object_name in objects:
            steps = _steps_for_object(source, target, object_name)
            for step in steps:
                out = agent._execute_step(step, len(outputs) + 1, outputs)
                outputs.append(out)
                print(
                    f"[fixed_task] {len(outputs)} {out.skill} "
                    f"success={out.success} message={out.message}",
                    flush=True,
                )
                if not out.success:
                    raise RuntimeError(f"Step failed: {out.skill}: {out.message}")

        with _suppress_errors():
            agent.backend._record_trajectory_frame()
        status = "OK"
    except Exception as exc:
        outputs.append(_error_step(str(exc)))
    finally:
        with _suppress_errors():
            trajectory_path = agent.backend.save_trajectory(
                rec_dir / f"trajectory_{args.timestamp}_{status}.json"
            )
        with _suppress_errors():
            agent.backend.close()

    result = {
        "timestamp": args.timestamp,
        "task_index": int(args.task_index),
        "level": task.get("level"),
        "env_name": env_name,
        "source": source,
        "target": target,
        "objects": objects,
        "status": status,
        "success": status == "OK",
        "elapsed_sec": round(time.perf_counter() - started, 3),
        "trajectory": trajectory_path,
        "steps": [_as_dict(out) for out in outputs],
    }
    result_path = rec_dir / f"result_{args.timestamp}.json"
    result_path.write_text(json.dumps(result, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
    result["result_json"] = str(result_path)
    return result


def _steps_for_object(source: str, target: str, object_name: str) -> list[dict[str, Any]]:
    return [
        {
            "skill_name": "move",
            "description": f"move to {source}",
            "inputs": {"target": source},
            "expected_output": f"reached {source}",
            "timeout": 300,
            "retries": 0,
        },
        {
            "skill_name": "pick_up",
            "description": f"pick {object_name}",
            "inputs": {"target": source, "object_name": object_name},
            "expected_output": "picked",
            "timeout": 300,
            "retries": 0,
        },
        {
            "skill_name": "move",
            "description": f"move to {target}",
            "inputs": {"target": target},
            "expected_output": f"reached {target}",
            "timeout": 300,
            "retries": 0,
        },
        {
            "skill_name": "place_down",
            "description": f"place {object_name}",
            "inputs": {"target": target},
            "expected_output": "placed",
            "timeout": 300,
            "retries": 0,
        },
    ]


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


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _object_names(value: Any) -> list[str]:
    if isinstance(value, str):
        return [value] if value else []
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value if item]
    return []


def _as_dict(value: Any) -> dict[str, Any]:
    if hasattr(value, "as_dict"):
        return value.as_dict()
    if isinstance(value, dict):
        return value
    return {"value": str(value)}


def _error_step(message: str) -> dict[str, Any]:
    return {
        "skill": "error",
        "description": "fixed task sequence failed",
        "success": False,
        "message": message,
        "payload": {"error": message},
    }


class _suppress_errors:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return True


if __name__ == "__main__":
    raise SystemExit(main())
