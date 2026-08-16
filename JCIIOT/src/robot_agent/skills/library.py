"""Skill library — wired to a real or simulated backend.

All skills require a backend; there is no mock / no-op fallback.
"""

from __future__ import annotations

import os

import numpy as np

from robot_agent.core.memory import InMemoryStore
from robot_agent.core.scene_context import SceneContext
from robot_agent.environments.base import EnvBackend
from robot_agent.skills.base import BaseSkill
from robot_agent.skills.move import MoveSkill
from robot_agent.skills.pick_up import PickUpSkill
from robot_agent.skills.place_down import PlaceDownSkill
from robot_agent.skills.record_trajectory import RecordTrajectorySkill
from robot_agent.skills.analyze_supply import AnalyzeSupplySkill
from robot_agent.skills.knowledge_mgr import KnowledgeMgrSkill
from robot_agent.skills.memory_mgr import MemoryMgrSkill
from robot_agent.skills.read_document import ReadDocumentSkill


def _detect_vision_api_config() -> dict:
    """Detect vision API configuration from environment / robot_params.

    Priority: VLM-specific env vars > OPENAI_* env vars > robot_params.json > defaults.
    """
    cfg: dict = {
        "ollama_base_url": "http://localhost:11434",
        "vision_model": "qwen3-vl:8b",
        "api_type": "ollama",
        "api_key": "",
    }

    # ── Check VLM-specific environment variables first ──
    vlm_url = os.getenv("VLM_BASE_URL", "")
    vlm_key = os.getenv("VLM_API_KEY", "")
    vlm_model = os.getenv("VLM_MODEL", "")
    if vlm_url:
        from robot_agent.core.vision_client import _detect_api_type
        cfg["ollama_base_url"] = vlm_url
        cfg["api_type"] = "openai" if vlm_key else _detect_api_type(vlm_url)
        cfg["api_key"] = vlm_key
        if vlm_model:
            cfg["vision_model"] = vlm_model

    # ── Fallback: OPENAI_* env vars (set when text LLM backend is OpenAI) ──
    elif os.getenv("OPENAI_API_KEY", ""):
        cfg["api_type"] = "openai"
        cfg["api_key"] = os.getenv("OPENAI_API_KEY", "")
        cfg["ollama_base_url"] = os.getenv(
            "OPENAI_BASE_URL", "https://api.openai.com/v1",
        )
        openai_model = os.getenv("OPENAI_MODEL", "")
        if openai_model:
            cfg["vision_model"] = openai_model

    # ── Read from robot_params.json for vision-specific settings ──
    try:
        from pathlib import Path
        import json
        _rp = Path(__file__).resolve().parents[3] / "knowledge" / "robot_params.json"
        if _rp.exists():
            _data = json.loads(_rp.read_text(encoding="utf-8"))
            _llm = _data.get("llm", {}) if isinstance(_data, dict) else {}
            if isinstance(_llm, dict):
                if not vlm_url:
                    cfg["ollama_base_url"] = _llm.get(
                        "ollama_base_url", cfg["ollama_base_url"],
                    )
                if not vlm_model:
                    cfg["vision_model"] = _llm.get(
                        "vision_model", cfg["vision_model"],
                    )
    except Exception:
        pass

    return cfg


def wired_skills(
    backend: EnvBackend,
    scene_context: SceneContext,
    grid: np.ndarray,
    *,
    path_spacing: float = 0.35,
    memory_store: InMemoryStore | None = None,
) -> list[BaseSkill]:
    """Return skills wired to a real (or simulated) backend."""
    _vis_cfg = _detect_vision_api_config()
    skills: list[BaseSkill] = [
        MoveSkill(
            backend=backend,
            scene_context=scene_context,
            grid=grid,
            path_spacing=path_spacing,
        ),
        PickUpSkill(backend=backend, scene_context=scene_context),
        PlaceDownSkill(backend=backend, scene_context=scene_context),
        AnalyzeSupplySkill(
            backend=backend,
            scene_context=scene_context,
            grid=grid,
            path_spacing=path_spacing,
        ),
        RecordTrajectorySkill(backend=backend),
        KnowledgeMgrSkill(knowledge_root="knowledge"),
        ReadDocumentSkill(
            ollama_base_url=_vis_cfg["ollama_base_url"],
            vision_model=_vis_cfg["vision_model"],
            api_type=_vis_cfg["api_type"],
            api_key=_vis_cfg["api_key"],
        ),
    ]
    if memory_store is not None:
        skills.append(MemoryMgrSkill(store=memory_store))
    return skills
