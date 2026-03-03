# SC-RRT 项目 AI 编码指南

## 项目定位

本工作区是一篇关于 **SC-RRT（Self-Constrained RRT）路径规划算法**的学术论文研究项目，包含：
- 论文写作材料（伪代码 `.tex`、流程图、Word 文档）
- **MATLAB 实验代码**（多算法对比实验）
- **Python 实验代码**（PID参数与自适应采样实验）

---

## 算法核心概念（必读）

**SC-RRT** = 基于非对称双向超椭球体约束采样的双向RRT*。三个核心机制：

1. **ADCS（自适应双椭球约束采样）**：三分支采样策略——知情采样（椭球内）、跨树采样（Pareto节点+扰动）、全局随机采样。概率权重由 `gamma`（膨胀系数）和 `p`（知情比例）控制。
2. **动态交汇点**：用势场理论（两棵树前30%优质节点的加权质心）实时更新 `x_meet`，替代固定中点。
3. **SSFOR（搜索状态反馈调节）**：基于迭代进度自动切换三阶段策略（探索→开发→收敛），调整 `gamma` 和 `p`。

超椭球体数学定义：$\mathcal{E} = \{x : \|x - x_f\| + \|x - x_m\| \le \gamma \cdot c_{best}\}$

---

## 目录结构与双轨实现

| 目录 | 语言 | 用途 |
|------|------|------|
| `Dynamic-rrt算法与SC-RRT算法对比实验/四算法对比实验1-7/` | MATLAB | **当前最新**多算法对比实验 |
| `Dynamic-rrt算法与SC-RRT算法对比实验/SC-RRT独立算法实现/copilot-1.6/` | MATLAB | SC-RRT独立完整实现 |
| `PID参数实验/sc_rrt_python - 1.6/` | Python | **当前最新** PID/自适应采样Python实现 |
| `algo_sc_rrt.tex` + `render_pseudocode.py` | Python/LaTeX | 论文伪代码渲染工具 |

> 版本命名规律：`copilot-1.x`（MATLAB独立实现），`sc_rrt_python - 1.x`（Python实现），`四算法对比实验1-x`（MATLAB对比实验）。**始终在最大版本号目录下工作。**

---

## 关键文件

### MATLAB 对比实验（`四算法对比实验1-7/`）
- `batchCompareAlgorithms.m`：主入口，批量运行5种算法（RRT, RRT-Connect, RRT*, SC-RRT, Dynamic-RRT）
- `sc_rrt/`、`dynamic_rrt/`：各算法子目录，`addpath` 时由 `batchCompareAlgorithms.m` 自动添加
- `UnifiedMetricsRecorder.m`：统一性能记录器，输出 `.mat` 和 `.csv`
- `generate2DEnvironment.m` / `generate3DEnvironment.m`：环境生成，支持自适应障碍物放置

### Python 实现（`sc_rrt_python - 1.6/`）
- `src/sc_rrt_adaptive.py`：核心算法类 `SCRRTAdaptive`，`mode='adaptive'|'no_adaptive'`
- `src/adaptive_sampling_controller.py`：三阶段控制器 `AdaptiveSamplingController`，`SearchPhase` 枚举
- `src/geometry.py`：碰撞检测（沿路径均匀采样），障碍物格式：2D `(N,3)[x,y,r]`，3D `(N,4)[x,y,z,r]`
- `run_adaptive_experiment.py`：主入口

---

## 运行方式

### MATLAB 对比实验
```matlab
% 切换到实验目录后运行
results = batchCompareAlgorithms(20, 5000, 'Dimension', '2D', 'NumObstacles', 225);
results = batchCompareAlgorithms(10, 5000, 'Dimension', '3D', 'NumObstacles', 400);
```

### Python 实验（在 `sc_rrt_python - 1.6/` 目录）
```bash
python run_adaptive_experiment.py --baseline   # 快速验证（6-10 min）
python run_adaptive_experiment.py              # 完整实验（1-2 h）
```

### 伪代码渲染（根目录）
```bash
python render_pseudocode.py algo_sc_rrt.tex    # 在浏览器中渲染为带KaTeX的HTML
```

---

## 实验场景标准配置

| 场景 | 边界 | 障碍物数 | 半径范围 | 迭代次数 |
|------|------|---------|---------|---------|
| 2D | 1500×1500 | 225 | R22-38（自适应缩放） | 900-1200 |
| 3D | 1500³ | 400 | R35-55（自适应缩放） | 1200-1500 |

步长比例 `STEP_SIZE_RATIO = 0.005`；2D步长范围 8~70，3D 15~50。

---

## 统一性能度量（论文规范）

三个核心指标（`UnifiedMetricsRecorder` 和 Python `ResultAnalyzer` 均遵循）：
- `convergence_time`：**首次**找到可行解的时间（秒）
- `path_cost`：路径相邻节点欧氏距离之和（**不可用起终点直线距离代替**）
- `planning_time`：算法总运行时间

未找到路径时 `path_cost = Inf`，`convergence_time = Inf`。

---

## 三阶段自适应控制参数

| 阶段 | 触发条件 | `gamma` | `p_informed` |
|------|---------|---------|-------------|
| 探索（Exploration） | 无解 | 6.0 | 0.20 |
| 开发（Exploitation） | 有解且进度<70% | 3.5 | 0.50 |
| 收敛（Convergence） | 进度≥70% | 2.0 | 0.70 |

有效采样率低于 20% 时自动放宽约束（`gamma` 上调），范围 [1.5, 8.0]。
