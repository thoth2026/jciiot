"""Generate JCIIOT SOP knowledge markdown from the official DOCX files.

This workflow is a reviewable competition artifact: it reads the original
`sop+prompt/*.docx` files, optionally asks the configured VLM to describe
embedded images, asks the configured text LLM to rewrite the SOP into a compact
knowledge-base format, and writes `knowledge/sop*.md`.
"""

from __future__ import annotations

import argparse
import difflib
import json
import os
import re
import ssl
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_agent.core.openai_client import OpenAIClient
from robot_agent.skills.read_document import extract_docx_payload


@dataclass(frozen=True)
class SopCase:
    level: str
    docx_name: str
    output_name: str


SOP_CASES = [
    SopCase("L1", "JCIIOT 2026 case 1 SOP.docx", "sop1.md"),
    SopCase("L2", "JCIIOT 2026 case 3 SOP.docx", "sop2.md"),
    SopCase("L3", "JCIIOT 2026 case 5 SOP.docx", "sop3.md"),
    SopCase("L4", "JCIIOT 2026 case 7 SOP.docx", "sop4.md"),
    SopCase("L5", "JCIIOT 2026 case 9 SOP.docx", "sop5.md"),
]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app-root", default=str(_default_app_root()))
    parser.add_argument("--only", choices=[case.level for case in SOP_CASES])
    parser.add_argument("--no-vision", action="store_true", help="Skip VLM image analysis")
    parser.add_argument("--dry-run", action="store_true", help="Do not write knowledge files")
    parser.add_argument("--report", default="team_submission/knowledge_generation_report.md")
    parser.add_argument("--reference-dir", default=str(_default_app_root().parents[1] / "default_sop"))
    parser.add_argument("--max-text-chars", type=int, default=18000)
    args = parser.parse_args(argv)

    if os.getenv("JCIIOT_ALLOW_UNVERIFIED_SSL", "").lower() in {"1", "true", "yes"}:
        ssl._create_default_https_context = ssl._create_unverified_context

    app_root = Path(args.app_root).resolve()
    knowledge_root = app_root / "knowledge"
    sop_root = app_root / "sop+prompt"
    report_path = (app_root / args.report).resolve()
    reference_dir = Path(args.reference_dir).resolve()

    robot_params = _load_json(knowledge_root / "robot_params.json")
    task_config = _load_json(knowledge_root / "task_config.json")
    text_client, text_model = _build_text_client(robot_params)
    vision_cfg = _build_vision_config(robot_params, text_model)

    selected = [case for case in SOP_CASES if args.only in (None, case.level)]
    report: list[str] = [
        "# SOP Knowledge Generation Report",
        "",
        f"Generated at: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"Text model: {text_model}",
        f"Vision enabled: {not args.no_vision}",
        f"Vision model: {vision_cfg['model']}",
        f"Dry run: {args.dry_run}",
        "",
    ]

    generated_docs: list[tuple[SopCase, str]] = []
    for case in selected:
        docx_path = sop_root / case.docx_name
        output_path = knowledge_root / case.output_name
        print(f"[{case.level}] reading {docx_path}")
        payload = extract_docx_payload(
            docx_path,
            use_vision=not args.no_vision,
            vision_base_url=vision_cfg["base_url"],
            vision_model=vision_cfg["model"],
            api_type=vision_cfg["api_type"],
            api_key=vision_cfg["api_key"],
        )
        task_meta = _task_meta_for_level(task_config, case.level)
        prompt = _build_generation_prompt(case, payload, task_meta, args.max_text_chars)
        print(f"[{case.level}] asking text LLM to generate {case.output_name}")
        raw = text_client.generate(prompt, num_predict=4096, temperature=0.1)
        markdown = _clean_markdown(raw)
        markdown = _postprocess_markdown(markdown, case, payload, task_meta)
        generated_docs.append((case, markdown))

        similarity = _reference_similarity(reference_dir / case.output_name, markdown)
        report.extend(_case_report(case, payload, output_path, markdown, similarity))
        if not args.dry_run:
            output_path.write_text(markdown, encoding="utf-8")
            print(f"[{case.level}] wrote {output_path}")
        else:
            print(f"[{case.level}] dry-run complete; not writing {output_path}")

    if not args.dry_run and args.only is None:
        sop_main = _build_sop_main(generated_docs)
        (knowledge_root / "sop_main.md").write_text(sop_main, encoding="utf-8")
        print(f"[main] wrote {knowledge_root / 'sop_main.md'}")
    elif not args.dry_run:
        print("[main] skipped sop_main.md because --only was used")

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(report), encoding="utf-8")
    print(f"[report] wrote {report_path}")
    return 0


def _default_app_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _build_text_client(robot_params: dict[str, Any]) -> tuple[OpenAIClient, str]:
    llm_cfg = robot_params.get("llm", {}) if isinstance(robot_params, dict) else {}
    glm_key = os.getenv("GLM_API_KEY", "") or os.getenv("ZHIPU_API_KEY", "")
    openai_key = os.getenv("OPENAI_API_KEY", "")

    if glm_key and not openai_key:
        base_url = os.getenv("GLM_BASE_URL", "https://open.bigmodel.cn/api/paas/v4")
        model = os.getenv("GLM_MODEL", "glm-4.6v-flash")
        return OpenAIClient(api_key=glm_key, base_url=base_url, model=model, timeout=180.0), model

    api_key = openai_key or glm_key
    if not api_key:
        raise RuntimeError(
            "Missing API key. Set OPENAI_API_KEY, GLM_API_KEY, or ZHIPU_API_KEY "
            "in the terminal before running this workflow."
        )

    base_url = os.getenv("OPENAI_BASE_URL", llm_cfg.get("openai_base_url", "https://api.deepseek.com"))
    model = os.getenv("OPENAI_MODEL", llm_cfg.get("openai_model", "deepseek-v4-flash"))
    return OpenAIClient(api_key=api_key, base_url=base_url, model=model, timeout=180.0), model


def _build_vision_config(robot_params: dict[str, Any], text_model: str) -> dict[str, str]:
    llm_cfg = robot_params.get("llm", {}) if isinstance(robot_params, dict) else {}
    vlm_key = os.getenv("VLM_API_KEY", "") or os.getenv("OPENAI_API_KEY", "") or os.getenv("GLM_API_KEY", "") or os.getenv("ZHIPU_API_KEY", "")
    vlm_url = os.getenv("VLM_BASE_URL", "") or os.getenv("OPENAI_BASE_URL", "")
    explicit_model = os.getenv("VLM_MODEL", "") or os.getenv("OPENAI_MODEL", "") or os.getenv("GLM_MODEL", "")
    if not vlm_url and (os.getenv("GLM_API_KEY", "") or os.getenv("ZHIPU_API_KEY", "")):
        vlm_url = os.getenv("GLM_BASE_URL", "https://open.bigmodel.cn/api/paas/v4")
    if not vlm_url:
        vlm_url = llm_cfg.get("ollama_base_url", "http://localhost:11434")

    api_type = "openai" if vlm_key else "ollama"
    if explicit_model:
        model = explicit_model
    elif api_type == "openai":
        model = text_model
    else:
        model = llm_cfg.get("vision_model", text_model)
    return {"base_url": vlm_url, "model": model, "api_type": api_type, "api_key": vlm_key}


def _task_meta_for_level(task_config: dict[str, Any], level: str) -> dict[str, Any]:
    for task in task_config.get("tasks", []):
        if task.get("level") == level:
            return task
    return {"level": level}


def _build_generation_prompt(
    case: SopCase,
    payload: dict[str, Any],
    task_meta: dict[str, Any],
    max_text_chars: int,
) -> str:
    image_notes = "\n".join(
        f"- {name}: {desc}" for name, desc in payload.get("image_descriptions", {}).items()
    ) or "(No image descriptions available.)"
    table_text = payload.get("table_markdown") or "(No tables found.)"
    source_text = payload.get("text", "")[:max_text_chars]
    task_meta_json = json.dumps(task_meta, ensure_ascii=False, indent=2)

    return f"""You are generating competition knowledge-base Markdown for a JCIIOT robot.

Source document: {case.docx_name}
Target level: {case.level}
Output file: {case.output_name}

The Markdown must be newly generated from the DOCX content below. Do not copy
or imitate any existing human-written sop*.md file. Keep it compact and useful
for an LLM planner that must output exactly four robot skills:
move -> pick_up -> move -> place_down.

Use the execution metadata only to normalize internal station names and exact
object_name candidates; the SOP task meaning must come from the DOCX.

Required Markdown structure:
# <level> Task - <short task name>
Level: <level> (max <score if known> points)
Scene: <scene prefix if known>

## Task
One sentence with material, pick station, and place station.

## Station Mapping
- Pick Station ... = <internal source>, center if known
- Place Station ... = <internal target>, center if known
- Robot start if known
- Target object: <exact object name or candidates>

## Standard Workflow
1. Navigate to the pick station using `move`.
2. Pick the material using `pick_up` with both `target` and exact `object_name`.
3. Navigate to the place station using `move`.
4. Place the material using `place_down`.

## Visual / Layout Notes
Concise notes from DOCX images, especially obstacles and safe approach hints.

## Safety and Scoring Notes
Mention collision penalty, grasp prerequisite, stable carry, and accurate final placement.

## Planner Hints
- Use exact internal station names.
- Use exact object_name from the metadata/SOP.
- Do not add extra skills beyond the four-step workflow.

Execution metadata:
```json
{task_meta_json}
```

Extracted DOCX text:
```text
{source_text}
```

Extracted DOCX tables:
{table_text}

VLM image descriptions:
{image_notes}

Return only Markdown. No code fence.
"""


def _clean_markdown(text: str) -> str:
    cleaned = text.strip()
    if cleaned.startswith("```"):
        cleaned = re.sub(r"^```[a-zA-Z0-9_-]*\s*", "", cleaned)
        cleaned = re.sub(r"\s*```$", "", cleaned).strip()
    return cleaned


def _postprocess_markdown(
    markdown: str,
    case: SopCase,
    payload: dict[str, Any],
    task_meta: dict[str, Any],
) -> str:
    header = "<!-- AI-GENERATED FROM DOCX - DO NOT REPLACE WITH DEFAULT SOP -->"
    source = f"<!-- Source: {case.docx_name}; paragraphs={payload['paragraph_count']}; images={payload['image_count']}; vlm={payload['images_analyzed']} -->"
    body = markdown
    if not body.startswith("#"):
        body = f"# {case.level} Task - Generated SOP\n\n{body}"
    if "## Planner Hints" not in body:
        objects = task_meta.get("object", [])
        if isinstance(objects, list):
            object_text = "; ".join(str(item) for item in objects)
        else:
            object_text = str(objects)
        body += (
            "\n\n## Planner Hints\n"
            f"- Internal source: `{task_meta.get('source', '')}`\n"
            f"- Internal target: `{task_meta.get('target', '')}`\n"
            f"- Object candidates: `{object_text}`\n"
            "- Plan exactly: `move` -> `pick_up` -> `move` -> `place_down`.\n"
        )
    return f"{header}\n{source}\n\n{body.strip()}\n"


def _build_sop_main(generated_docs: list[tuple[SopCase, str]]) -> str:
    rows = []
    for case, markdown in generated_docs:
        scene = _match_line(markdown, r"^Scene:\s*(.+)$")
        task = _section_first_sentence(markdown, "Task")
        pick = _first_station_line(markdown, "Pick Station")
        place = _first_station_line(markdown, "Place Station")
        obj = _match_line(markdown, r"^- Target object:\s*(.+)$") or _match_line(markdown, r"^- Object candidates:\s*`?(.+?)`?$")
        rows.append((case.level, scene, task, pick, obj, place))

    lines = [
        "<!-- AI-GENERATED FROM DOCX - DO NOT REPLACE WITH DEFAULT SOP -->",
        "",
        "# Standard Operating Procedure (SOP)",
        "",
        "Task ID: MT-MOBILE-001",
        "Version: AI-DOCX-1.0",
        "",
        "## Standard Transport Workflow",
        "",
        "1. Navigate to Pick Station",
        "2. Pick material with `pick_up` using both `target` and exact `object_name`",
        "3. Navigate to Place Station with object held",
        "4. Place material with `place_down` and confirm stable placement",
        "",
        "## Task Coordinate Reference",
        "",
        "| Level | Scene | Task | Pick Station | Object Name | Place Station |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(_escape_cell(item) for item in row) + " |")
    lines.extend([
        "",
        "## CRITICAL pick_up Rules",
        "",
        "- `pick_up` requires BOTH `target` and `object_name`.",
        "- Use the exact object name from the current scene metadata or generated SOP.",
        "- Execute exactly four steps: `move`, `pick_up`, `move`, `place_down`.",
    ])
    return "\n".join(lines) + "\n"


def _match_line(markdown: str, pattern: str) -> str:
    match = re.search(pattern, markdown, flags=re.MULTILINE)
    return match.group(1).strip() if match else ""


def _section_first_sentence(markdown: str, section: str) -> str:
    match = re.search(rf"^## {re.escape(section)}\s+(.+?)(?:\n## |\Z)", markdown, flags=re.MULTILINE | re.DOTALL)
    if not match:
        return ""
    text = " ".join(line.strip("- ").strip() for line in match.group(1).splitlines() if line.strip())
    sentence = re.split(r"(?<=[.!?。])\s+", text, maxsplit=1)[0]
    return sentence[:180]


def _first_station_line(markdown: str, label: str) -> str:
    for line in markdown.splitlines():
        if label in line and line.lstrip().startswith("-"):
            return line.strip("- ").strip()
    return ""


def _escape_cell(value: str) -> str:
    return str(value or "").replace("|", ";").replace("\n", " ").strip()


def _reference_similarity(reference_path: Path, markdown: str) -> float | None:
    if not reference_path.exists():
        return None
    ref = reference_path.read_text(encoding="utf-8", errors="replace")
    return difflib.SequenceMatcher(None, ref, markdown).ratio()


def _case_report(
    case: SopCase,
    payload: dict[str, Any],
    output_path: Path,
    markdown: str,
    similarity: float | None,
) -> list[str]:
    similarity_text = "n/a" if similarity is None else f"{similarity:.3f}"
    return [
        f"## {case.level} -> {case.output_name}",
        "",
        f"- Source DOCX: `{case.docx_name}`",
        f"- Output: `{output_path}`",
        f"- Paragraphs: {payload['paragraph_count']}",
        f"- Tables: {payload['table_count']}",
        f"- Images: {payload['image_count']}",
        f"- VLM analyzed: {payload['images_analyzed']}",
        f"- Similarity to `default_sop/{case.output_name}`: {similarity_text}",
        f"- Generated chars: {len(markdown)}",
        "",
    ]


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
