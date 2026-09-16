from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def source_default_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "default.yaml"


def installed_default_path() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("mini_ground_control")) / "config" / "default.yaml"
    except Exception:
        return None


def default_config_path() -> Path:
    installed = installed_default_path()
    if installed is not None and installed.exists():
        return installed
    return source_default_path()


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    default_path = default_config_path()
    with default_path.open("r", encoding="utf-8") as stream:
        base = yaml.safe_load(stream) or {}

    if path is None:
        return base

    requested = Path(path).expanduser().resolve()
    if requested == default_path.resolve():
        return base
    with requested.open("r", encoding="utf-8") as stream:
        override = yaml.safe_load(stream) or {}
    return _merge(base, override)
