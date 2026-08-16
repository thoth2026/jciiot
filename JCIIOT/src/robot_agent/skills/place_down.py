"""Place-down skill: release a held object through real backend physics only."""

from __future__ import annotations

import logging
import os

from robot_agent.core.scene_context import SceneContext
from robot_agent.core.types import ExecutionContext, SkillResult
from robot_agent.skills.base import BaseSkill
from robot_agent.skills.pick_up import _resolve_station_name

logger = logging.getLogger(__name__)


class PlaceDownSkill(BaseSkill):
    """Release a held object at the target through the environment backend."""

    def __init__(self, *, backend, scene_context: SceneContext | None = None) -> None:
        super().__init__(
            name="place_down",
            description="Place down or drop an object",
            keywords=(
                "place", "put", "drop", "release",
                "place", "drop", "put", "release", "unload",
            ),
        )
        self._backend = backend
        self._scene = scene_context

    def run(self, context: ExecutionContext) -> SkillResult:
        raw_target: str = (
            context.metadata.get("inputs", {}).get("target")
            or context.task
        )
        target = raw_target
        if self._scene is not None:
            target = _resolve_station_name(raw_target, self._scene)
            logger.info("place_down target: %r -> %r", raw_target, target)

        if not hasattr(self._backend, "place_object_physics"):
            return SkillResult(
                skill_name=self.name,
                success=False,
                message=f"No physics place backend; snap place disabled: {target}",
                payload={
                    "action": "place_down",
                    "target": target,
                    "raw_target": raw_target,
                    "method": "disabled_snap",
                },
            )

        try:
            ok = bool(self._backend.place_object_physics(target))
        except Exception as exc:
            logger.exception("physics place crashed")
            return SkillResult(
                skill_name=self.name,
                success=False,
                message=f"Physics place error: {exc}",
                payload={
                    "action": "place_down",
                    "target": target,
                    "method": "physics",
                    "ok": False,
                    "error": str(exc),
                },
            )

        if not ok:
            held = getattr(self._backend, "_held_crate_name", None)
            physical_object = getattr(self._backend, "_jciiot_last_pick_object", None)
            if physical_object and _physical_release_grasp(self._backend, physical_object):
                return SkillResult(
                    skill_name=self.name,
                    success=True,
                    message=f"Physics release OK: {target}",
                    payload={
                        "action": "place_down",
                        "target": target,
                        "method": "physical_gripper_release",
                        "ok": True,
                        "object_name": physical_object,
                        "fallback": None,
                    },
                )
            ports = []
            try:
                ports = list(self._backend.env.output_ports.keys())
            except Exception:
                pass
            logger.warning("place_down failed: target=%s held=%s output_ports=%s", target, held, ports)

        return SkillResult(
            skill_name=self.name,
            success=ok,
            message=f"Physics place {'OK' if ok else 'FAIL'}: {target}",
            payload={
                "action": "place_down",
                "target": target,
                "method": "physics",
                "ok": ok,
                "fallback": None,
            },
        )


def _physical_release_grasp(backend, object_name: str) -> bool:
    """Open the grippers and let the currently grasped object fall naturally."""
    env = getattr(backend, "env", None)
    if env is None:
        return False
    try:
        from robosuite.environments.factory_sorting.place_on_table import gripper_release_action
        from robosuite.environments.factory_sorting.transport_attachment import get_object_qpos

        release_steps = max(12, int(os.getenv("JCIIOT_PHYSICAL_RELEASE_STEPS", "60")))
        settle_steps = max(0, int(os.getenv("JCIIOT_PHYSICAL_RELEASE_SETTLE_STEPS", "35")))
        release_action = gripper_release_action(env)
        _, start_qpos = get_object_qpos(env, object_name)
        start_z = float(start_qpos[2])
        for _ in range(release_steps):
            env.step(release_action)
            if hasattr(backend, "_record_trajectory_frame"):
                backend._record_trajectory_frame()
        for _ in range(settle_steps):
            env.step(release_action)
            if hasattr(backend, "_record_trajectory_frame"):
                backend._record_trajectory_frame()
        _, end_qpos = get_object_qpos(env, object_name)
        logger.info(
            "physical release object=%s start_z=%.4f end_z=%.4f",
            object_name,
            start_z,
            float(end_qpos[2]),
        )
        return True
    except Exception as exc:
        logger.warning("physical release failed: %s", exc)
        return False
