# EXPERIMENT_PROTOCOL v1（migration_v1，SC-only）

> 算法主证：Submission §3.1（MATLAB 2D/3D）。  
> 机械臂层：**仅 SC-RRT** TCP 迁移验证；不在臂上重复 Dynamic / Connect / Informed 统计对比。  
> **实验目标、指标、交互入口**：[`docs/EXPERIMENT.md`](docs/EXPERIMENT.md)

## 套件

| 套件 | 规模 | 用途 |
|------|------|------|
| `calibrate_migration_v1` | 4×5 | SC 场景门禁（Open≥90%, Clutter≥85%, Narrow/Dense≥80%） |
| `migration_v1` | 4×30 | 论文 Tab. 3.3 |

案例：`data/processed/app_4scenes_v1.json`  
障碍：`config/obstacles_app_v1.yaml`（`scene_s1_open` … `scene_s4_dense`）

## 统一预算（SC-RRT）

`max_iterations=10000`, `step_size=0.02` m, `goal_threshold=0.04` m, `safety_margin=0.006` m, 墙钟 10 s（含 postprocess）。  
与 3.1 实验共用同一后处理配置（`common_postprocess`）。

## Phase A → Phase B

1. 全量 `migration_v1` 写 CSV + 代表 JSON（成功 run 中 **planning_time 中位数**）。  
2. `run_migration_phase_b.py`：四 JSON 离线 IK → `phase_b_migration.csv`。  
3. `analyze_migration_sc.py`：Tab. 3.3；Phase B 列为**每场景 1 条代表路径**（表注见 `migration_v1_table.md`）。

Gazebo 物理回放**可选**，不阻塞 Tab. 3.3。
