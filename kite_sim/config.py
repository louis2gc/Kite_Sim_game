"""Load the standalone game's physical settings."""
from pathlib import Path
import yaml
from .physics.rigid_params import KiteParams

DEFAULT_CONFIG = Path(__file__).resolve().parents[1]/'configs/default.yaml'


def load_params(path=DEFAULT_CONFIG):
    with Path(path).open(encoding='utf-8') as file:
        data=yaml.safe_load(file)
    if not isinstance(data,dict):
        raise ValueError('Configuration must be a YAML mapping')
    return KiteParams(**data)
