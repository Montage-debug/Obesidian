"""实验根目录解析：向上查找含 config/obstacles.yaml 的目录。"""

from pathlib import Path


def find_experiment_root(start: Path | None = None) -> Path:
    """从给定路径向上搜索实验根目录。"""
    p = (start or Path(__file__)).resolve()
    for _ in range(12):
        if (p / "config" / "obstacles.yaml").is_file():
            return p
        if p.parent == p:
            break
        p = p.parent
    raise FileNotFoundError("无法定位实验根目录（缺少 config/obstacles.yaml）")


EXP_ROOT = find_experiment_root()
