"""Render replay GIFs from saved FactorySorting trajectory JSON files."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


def render_trajectory_gif(
    json_path: str | Path,
    output_gif: str | Path,
    *,
    env_name: str,
    camera: str = "birdview",
    width: int = 640,
    height: int = 480,
    frame_start: int | None = None,
    frame_end: int | None = None,
) -> list[np.ndarray]:
    """Restore saved qpos frame-by-frame and render a GIF.

    This intentionally avoids ``RobosuiteBackend.replay_trajectory`` because
    that path can produce black frames after restoring mobile-base state on the
    competition scenes.
    """
    from robot_agent.environments.robosuite_backend import (
        RobosuiteBackend,
        _set_base_world_yaw_direct,
        _set_base_xy_direct,
    )

    data = json.loads(Path(json_path).read_text(encoding="utf-8"))
    frames = data.get("frames", [])
    if not frames:
        return []

    start = 0 if frame_start is None else max(0, int(frame_start))
    end = len(frames) if frame_end is None else min(len(frames), int(frame_end))
    frames = frames[start:end]
    if not frames:
        return []

    render_camera = "birdview" if camera == "follow" else camera
    backend = RobosuiteBackend(env_name=env_name, camera=render_camera, drive_mode="direct")
    backend.reset()
    env = backend.env
    robot = env.robots[0]

    object_joint_map: dict[str, str] = data.get("object_joints", {})
    joint_addr_cache: dict[str, int | None] = {}
    object_addr_cache: dict[str, tuple[int, int] | None] = {}
    base_joints = {
        "mobilebase0_joint_mobile_forward",
        "mobilebase0_joint_mobile_side",
        "mobilebase0_joint_mobile_yaw",
    }
    rendered: list[np.ndarray] = []

    try:
        for frame in frames:
            _restore_frame(
                env=env,
                robot=robot,
                frame=frame,
                object_joint_map=object_joint_map,
                joint_addr_cache=joint_addr_cache,
                object_addr_cache=object_addr_cache,
                base_joints=base_joints,
                set_base_world_yaw=_set_base_world_yaw_direct,
                set_base_xy=_set_base_xy_direct,
            )
            rendered.append(backend.capture_frame(camera=render_camera, width=width, height=height))
    finally:
        backend.close()

    if output_gif and rendered:
        out = Path(output_gif)
        out.parent.mkdir(parents=True, exist_ok=True)
        step = max(1, len(rendered) // 300)
        display = rendered[::step]
        pause = [Image.fromarray(display[-1])] * 5
        images = [Image.fromarray(frame) for frame in display] + pause
        images[0].save(
            out,
            format="GIF",
            save_all=True,
            append_images=images[1:],
            duration=100,
            loop=0,
        )

    return rendered


def _restore_frame(
    *,
    env: Any,
    robot: Any,
    frame: dict[str, Any],
    object_joint_map: dict[str, str],
    joint_addr_cache: dict[str, int | None],
    object_addr_cache: dict[str, tuple[int, int] | None],
    base_joints: set[str],
    set_base_world_yaw,
    set_base_xy,
) -> None:
    bp = frame.get("base_pose", {})
    jp = frame.get("joint_positions", {})

    ori = bp.get("orientation_xyzw", [])
    if len(ori) >= 4:
        qz, qw = float(ori[2]), float(ori[3])
        set_base_world_yaw(env, robot, 2.0 * math.atan2(qz, qw))

    pos = bp.get("position", [])
    if len(pos) >= 2:
        set_base_xy(env, robot, np.array([pos[0], pos[1]], dtype=float))

    for name, value in jp.items():
        if name in base_joints:
            continue
        if name not in joint_addr_cache:
            try:
                addr = env.sim.model.get_joint_qpos_addr(name)
                joint_addr_cache[name] = None if isinstance(addr, tuple) else int(addr)
            except Exception:
                joint_addr_cache[name] = None
        addr = joint_addr_cache.get(name)
        if addr is not None:
            env.sim.data.qpos[addr] = float(value)

    for name, values in frame.get("object_positions", {}).items():
        if name not in object_addr_cache:
            joint_name = object_joint_map.get(name, name)
            try:
                addr = env.sim.model.get_joint_qpos_addr(joint_name)
            except Exception:
                try:
                    addr = env.sim.model.get_joint_qpos_addr(name)
                except Exception:
                    addr = None
            object_addr_cache[name] = addr if isinstance(addr, tuple) else None
        addr = object_addr_cache.get(name)
        if addr is not None:
            env.sim.data.qpos[addr[0]:addr[1]] = values

    env.sim.forward()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("json_path")
    parser.add_argument("output_gif")
    parser.add_argument("--env-name", required=True)
    parser.add_argument("--camera", default="birdview")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frame-start", type=int, default=None)
    parser.add_argument("--frame-end", type=int, default=None)
    args = parser.parse_args(argv)

    render_trajectory_gif(
        args.json_path,
        args.output_gif,
        env_name=args.env_name,
        camera=args.camera,
        width=args.width,
        height=args.height,
        frame_start=args.frame_start,
        frame_end=args.frame_end,
    )
    print(args.output_gif)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
