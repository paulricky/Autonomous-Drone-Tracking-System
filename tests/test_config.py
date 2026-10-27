from pathlib import Path
import pytest
from robotics_tools.utils.config import ConfigError, load_yaml, require_keys


ROOT = Path(__file__).resolve().parents[1]


def test_all_initial_configs_are_mappings() -> None:
    for name in ("planning", "mapping", "hand", "stereo", "trajectory", "flight", "simulation"):
        assert load_yaml(ROOT / "config" / f"{name}.yaml")


def test_missing_config_has_actionable_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="Configuration file not found"):
        load_yaml(tmp_path / "missing.yaml")


def test_required_keys() -> None:
    with pytest.raises(ConfigError, match="beta"):
        require_keys({"alpha": 1}, "alpha", "beta")

