"""Vision client — send images to multimodal LLM via Ollama or OpenAI-compatible API."""

from __future__ import annotations

import base64
import json
import logging
import urllib.request
from pathlib import Path

logger = logging.getLogger(__name__)


def extract_images_from_docx(path: str | Path) -> list[tuple[str, bytes]]:
    """Extract all images from a .docx file. Returns [(filename, bytes), ...]."""
    from docx import Document

    images = []
    doc = Document(str(path))
    for rel in doc.part.rels.values():
        if "image" in rel.reltype:
            name = rel.target_ref.split("/")[-1] if rel.target_ref else "image.png"
            images.append((name, rel.target_part.blob))
    return images


def _guess_image_mime(image_bytes: bytes) -> str:
    """Guess the MIME type of an image from its header bytes."""
    if image_bytes[:4] == b"\x89PNG":
        return "image/png"
    if image_bytes[:2] == b"\xff\xd8":
        return "image/jpeg"
    if image_bytes[:4] == b"RIFF" and image_bytes[8:12] == b"WEBP":
        return "image/webp"
    if image_bytes[:3] == b"GIF":
        return "image/gif"
    return "image/png"  # fallback


def ask_vision(
    prompt: str,
    images: list[bytes] | bytes,
    *,
    base_url: str = "http://localhost:11434",
    model: str = "qwen3-vl:8b",
    timeout: float = 60.0,
    api_type: str = "ollama",
    api_key: str = "",
) -> str:
    """Send image(s) + text prompt to a multimodal LLM, return response text.

    Args:
        prompt: Text prompt describing what to look for.
        images: Single image bytes or list of image bytes.
        base_url: API endpoint URL (Ollama or OpenAI-compatible).
        model: Model name.
        timeout: Request timeout in seconds.
        api_type: ``"ollama"`` (default) or ``"openai"``.
        api_key: API key (required for ``api_type="openai"``).

    Returns:
        Model response text.
    """
    img_list = images if isinstance(images, list) else [images]
    b64_list = [base64.b64encode(img).decode() for img in img_list]

    if api_type == "openai":
        return _ask_vision_openai(prompt, b64_list, img_list,
                                  base_url=base_url, model=model,
                                  timeout=timeout, api_key=api_key)
    return _ask_vision_ollama(prompt, b64_list,
                              base_url=base_url, model=model,
                              timeout=timeout)


def _ask_vision_ollama(
    prompt: str,
    b64_list: list[str],
    *,
    base_url: str,
    model: str,
    timeout: float,
) -> str:
    """Call Ollama /api/chat with images."""
    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": prompt,
                "images": b64_list,
            }
        ],
        "stream": False,
    }
    req = urllib.request.Request(
        f"{base_url.rstrip('/')}/api/chat",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    resp = json.loads(urllib.request.urlopen(req, timeout=timeout).read().decode("utf-8"))
    return resp["message"]["content"]


def _ask_vision_openai(
    prompt: str,
    b64_list: list[str],
    img_list: list[bytes],
    *,
    base_url: str,
    model: str,
    timeout: float,
    api_key: str,
) -> str:
    """Call OpenAI-compatible /chat/completions with vision."""
    content_parts: list[dict] = [
        {"type": "text", "text": prompt},
    ]
    for i, b64 in enumerate(b64_list):
        mime = _guess_image_mime(img_list[i])
        content_parts.append({
            "type": "image_url",
            "image_url": {
                "url": f"data:{mime};base64,{b64}",
            },
        })

    payload = {
        "model": model,
        "messages": [
            {"role": "user", "content": content_parts},
        ],
        "max_tokens": 1024,
        "stream": False,
    }

    endpoint = f"{base_url.rstrip('/')}/chat/completions"
    headers = {"Content-Type": "application/json"}
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"

    req = urllib.request.Request(
        endpoint,
        data=json.dumps(payload).encode("utf-8"),
        headers=headers,
    )
    resp = json.loads(urllib.request.urlopen(req, timeout=timeout).read().decode("utf-8"))
    return resp["choices"][0]["message"]["content"]


def _detect_api_type(base_url: str) -> str:
    """Auto-detect API type from URL pattern."""
    url_lower = base_url.lower()
    if "openai" in url_lower or "/v1" in url_lower:
        return "openai"
    for keyword in ("deepseek", "zhipu", "bigmodel", "together", "fireworks",
                    "openrouter", "groq", "mistral", "anthropic"):
        if keyword in url_lower:
            return "openai"
    return "ollama"


def ask_vision_auto(
    prompt: str,
    images: list[bytes] | bytes,
    *,
    base_url: str = "http://localhost:11434",
    model: str = "qwen3-vl:8b",
    timeout: float = 60.0,
    api_key: str = "",
) -> str:
    """Like :func:`ask_vision` but auto-detects API type from URL.

    Use this when the caller doesn't know which API type is configured.
    """
    api_type = _detect_api_type(base_url)
    logger.info("ask_vision_auto: detected %s from %s", api_type, base_url)
    return ask_vision(
        prompt, images,
        base_url=base_url, model=model,
        timeout=timeout, api_type=api_type, api_key=api_key,
    )


def read_docx_with_vision(
    path: str | Path,
    *,
    prompt: str = "Describe every object, station, and coordinate visible in this image.",
    base_url: str = "http://localhost:11434",
    model: str = "qwen3-vl:8b",
    timeout: float = 60.0,
    api_type: str = "ollama",
    api_key: str = "",
) -> str:
    """Extract images from a .docx file and describe them all with a vision model.

    Returns concatenated descriptions.
    """
    images = extract_images_from_docx(path)
    if not images:
        return "No images found in the document."

    results = []
    for name, img_data in images:
        desc = ask_vision(f"{prompt}\n[image: {name}]", img_data,
                          base_url=base_url, model=model,
                          timeout=timeout, api_type=api_type, api_key=api_key)
        results.append(f"## {name}\n{desc}")
    return "\n\n".join(results)
