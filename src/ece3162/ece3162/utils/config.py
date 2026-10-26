"""YAML configuration loading with actionable errors."""

from pathlib import Path
from typing import Any
import yaml


class ConfigError(RuntimeError):
    pass


def load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    if not config_path.is_file():
        raise ConfigError(f"Configuration file not found: {config_path}")
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ConfigError(f"Invalid YAML in {config_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"Top-level YAML value must be a mapping: {config_path}")
    return data


def require_keys(config: dict[str, Any], *keys: str) -> None:
    missing = [key for key in keys if key not in config]
    if missing:
        raise ConfigError(f"Missing required configuration keys: {', '.join(missing)}")

