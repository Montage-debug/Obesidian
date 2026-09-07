"""path_export schema 回归。"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "planning" / "src"))

from path_export import export_representative_path, load_deployed_poses


def test_export_and_load_roundtrip(tmp_path):
    case_meta = {"scene_id": "S1_Open", "obstacle_set": "scene_s1_open"}
    deployed = [[0.44, -0.24, 0.42], [0.72, 0.12, 0.18]]
    raw = [[0.44, -0.24, 0.42], [0.50, 0.0, 0.30], [0.72, 0.12, 0.18]]
    out = export_representative_path(
        tmp_path, 0, "SC-RRT", case_meta, 12, 0.014, deployed, raw
    )
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["run_id"] == 12
    assert data["scene_id"] == "S1_Open"
    assert len(data["raw_poses"]) == 3
    loaded = load_deployed_poses(out)
    assert len(loaded) == 2
