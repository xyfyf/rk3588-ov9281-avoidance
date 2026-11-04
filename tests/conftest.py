import json
from pathlib import Path
import pytest
from omniavoid.dataset import demo_config


@pytest.fixture
def cfg():
    source = Path(__file__).resolve().parents[1] / "configs" / "rig.example.json"
    return demo_config(json.loads(source.read_text(encoding="utf-8")))
