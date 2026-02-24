# Dynamic RRT 算法实现

基于论文复现: **Dynamic RRT: Fast Feasible Path Planning in Randomly Distributed Obstacle Environments (2023)**

## 🎉 最新更新 (2026-02-07)

✅ **单次运行对比实验已完成！**
- 修复了SC-RRT字段名兼容性问题
- 生成高质量论文图片（PNG + EPS 300 DPI）
- 自动输出LaTeX格式统计表
- 完美适配Dynamic-RRT论文标准环境配置

👉 **快速开始**: 打开 [快速运行指南.txt](快速运行指南.txt) 或直接运行 `single_run_comparison`

---

## 📋 项目结构

```
Dynamic-RRT算法实现/
├── 核心算法文件
│   ├── DynamicRRT.m              # 主算法实现
│   ├── CalCostHat.m              # F̂(c)估计器（论文核心）
│   ├── SampleEllipsoid.m         # 超椭球采样
│   ├── ChooseNodePareto.m        # Pareto dominance选择
│   ├── Nearest.m                 # 最近邻搜索
│   ├── Steer.m                   # 节点扩展
│   ├── CheckCollision.m          # 碰撞检测
│   └── GenerateObstacles.m       # 障碍物生成
│
├── 测试脚本
│   ├── test_dynamic_rrt.m        # 单次测试脚本
│   └── test_interval_comparison.m # Interval参数对比实验
│
├── ⭐ 论文图片生成（推荐）
│   ├── single_run_comparison.m   # 单次运行对比（生成论文图片）
│   ├── 运行单次对比实验.bat      # 快速启动器
│   ├── 单次运行对比使用说明.md    # 详细使用文档
│   └── 快速运行指南.txt          # 快速参考卡片
│
├── 批量统计实验
│   ├── compare_algorithms.m      # 批量对比实验（已修复SC-RRT兼容性）
│   ├── run_comparison_experiment.m # 自动化实验（已修复变量问题）
│   ├── generate_paper_data.m     # 论文数据生成器
│   └── demo_comparison.m         # 算法演示脚本
│
└── 文档
    ├── README.md                 # 本文档
    ├── COMPARISON_GUIDE.md       # 对比实验使用指南
    ├── 实验系统完成报告.md        # 系统报告
    └── 文件清单.md               # 文件索引
```

## 🎯 核心特性

### 1. Informed Subset采样
- 使用 F̂(c) 估计构建超椭球约束
- 焦点为当前起点和目标点
- 主轴直径由 CalCostHat 动态计算

### 2. Pareto Dominance动态规划
- 周期性选择新起点分解子问题
- 基于 out-degree 和 F̂(x) 的多目标优化
- 概率性选择避免局部最优

### 3. 可调Interval参数
- 控制Pareto选择频率
- 实现收敛速度与路径质量的折中
- 论文推荐值: 4-8

## 🚀 快速开始

### ⭐ 方式1: 单次运行生成论文图片（推荐）

```matlab
% 在MATLAB命令窗口输入
cd 'e:\obsidian_知识库\01-项目实践\机械革命项目\SC-RRT小论文编写\Dynamic-rrt算法与SC-RRT算法对比实验\Dynamic-RRT算法实现'
single_run_comparison
```

**特点**：
- ✅ 运行一次即可，耗时10-30秒
- ✅ 弹出2个对话框：障碍物环境 + 算法对比结果
- ✅ 生成高质量EPS图片（300 DPI，适合论文）
- ✅ 自动输出LaTeX格式统计表
- ✅ 已修复SC-RRT字段名兼容性问题

**输出文件**：
- `paper_figures/environment_[时间戳].png/eps` - 障碍物环境图
- `paper_figures/algorithm_comparison_[时间戳].png/eps` - 算法对比图

**详细说明**: 查看 [单次运行对比使用说明.md](单次运行对比使用说明.md) 或 [快速运行指南.txt](快速运行指南.txt)

---

### 方式2: 批量统计实验（用于统计分析）

```matlab
% 快速测试（5次运行，验证系统）
run_comparison_experiment('quick_test', true)

% 标准实验（30次运行，约20分钟）
run_comparison_experiment('num_runs', 30)

% 论文标准（100次运行，约60分钟）
run_comparison_experiment('num_runs', 100)
```

**特点**：
- 多次运行获取统计数据（均值、标准差）
- 生成箱线图、雷达图、Wilcoxon统计检验
- 输出Excel表格和LaTeX代码
- 适合"实验结果与分析"章节

**详细说明**: 查看 [COMPARISON_GUIDE.md](COMPARISON_GUIDE.md)

---

### 方式3: Dynamic-RRT单独测试

```matlab
% 在MATLAB中运行
test_dynamic_rrt
```

**配置选项:**
- `env_idx`: 选择环境 (1-4)
  - 1: 2D固定障碍物
  - 2: 2D随机障碍物
  - 3: 3D固定障碍物
  - 4: 3D随机障碍物
- `interval`: Pareto选择间隔 (推荐4)

### 方式4: Interval参数对比

```matlab
% 对比不同interval值的效果（对应论文Table 2）
test_interval_comparison
```

这会测试 interval = [3, 4, 6, 8, 10, 20]，并与论文结果对比。

## 📊 论文环境配置

### Environment 1 (2D固定)
- 空间: 1500×1500
- 障碍物: 225个 (半径15)
- 起点: (400, 400)
- 终点: (1100, 1100)

### Environment 3 (3D固定)
- 空间: 1500×1500×1500
- 障碍物: 400个 (半径18)
- 起点: (1, 1, 1)
- 终点: (1100, 1100, 1100)

## 📈 预期结果

### Interval对比 (论文Table 2, 2D环境, 100次平均)

| Interval | 收敛时间(秒) | 路径长度 |
|----------|-------------|---------|
| 3        | 0.195       | 1136.0  |
| 4        | 0.034       | 1157.4  |
| 6        | 0.056       | 1170.1  |
| 8        | 0.064       | 1187.0  |
| 10       | 0.072       | 1197.5  |
| 20       | 0.108       | 1250.7  |

**趋势:**
- interval 从 20→4: 更快且更短
- interval = 3: 可能出现耗时反增（Pareto节点过少）

## 🔧 算法参数

### 关键参数

```matlab
params = struct();
params.MaxIterations = 10000;     % 最大迭代次数
params.Interval = 4;              % Pareto选择间隔
params.ParetoProb = 0.1;          % 非最优节点选择概率
params.EnableVisualization = true; % 可视化开关
```

### 自动参数

根据维度自动设置:
- **2D**: stepSize=30, goalThreshold=20
- **3D**: stepSize=40, goalThreshold=30

## 📝 函数接口

### 主函数

```matlab
[tree, path, success, metrics] = DynamicRRT(startPoint, goalPoint, bounds, obstacles, ...)
```

**输出:**
- `tree`: RRT树结构
- `path`: 最终路径 [N×m]
- `success`: 是否成功
- `metrics`: 性能指标
  - `convergenceTime`: 首次收敛时间
  - `pathLength`: 路径总长度
  - `iterations`: 迭代次数
  - `nodeCount`: 树节点数

### 核心函数

**CalCostHat**: 估计 F̂(c)
```matlab
F_hat = CalCostHat(x_start, x_goal, x_c, cost_c, tree, c_idx)
```

**SampleEllipsoid**: 超椭球内采样
```matlab
x_rand = SampleEllipsoid(xa, xb, bounds, c_hat, m)
```

**ChooseNodePareto**: Pareto选择
```matlab
[newStartIdx, newStart] = ChooseNodePareto(tree, x_goal, p_non_optimal, currentStartIdx)
```

## ✅ 复现验证标准

### 强判据（必须满足）
1. ✓ Informed subset采样正常工作
2. ✓ Pareto选择按interval触发
3. ✓ 概率性选择避免局部最优
4. ✓ 环境配置对齐论文Table 1

### 弱判据（趋势一致）
1. ✓ interval↓ → 时间↓, 长度↓
2. ✓ interval过小可能反增
3. ✓ 相对其他算法的优势

## 🐛 调试建议

### 如果路径规划失败
1. 检查障碍物是否阻断了所有路径
2. 增加 `MaxIterations`
3. 调整 `stepSize` 和 `goalThreshold`
4. 启用可视化观察搜索过程

### 如果性能不符合预期
1. 调整 `Interval` 参数
2. 检查 CalCostHat 估计是否合理
3. 确认超椭球采样范围正确
4. 验证Pareto选择逻辑

## 📚 论文关键点

### Section 3.1: F̂(c)估计
- 使用路径片段的MBR（最小包围矩形）
- 距离比例和体积缩放
- 避免退化为全空间采样

### Section 3.2: 动态规划
- Pareto向量: [-O(x), F̂(x)]
- O(x): out-degree (子节点数)
- 随机选择策略避免局部最优

### Section 4: 实验结论
- Dynamic RRT 兼顾速度与质量
- interval 是关键调节参数
- 相比Informed RRT*显著更快

## 📧 使用说明

1. **环境要求**: MATLAB R2018a 或更高版本
2. **运行时间**: 2D环境通常 < 1秒，3D环境可能需要几秒
3. **可复现性**: 使用固定seed确保结果可复现

## 🔍 与SC-RRT的区别

| 特性 | Dynamic RRT | SC-RRT |
|------|-------------|--------|
| 树结构 | 单树 | 双树 |
| 采样策略 | 超椭球 | 双椭球 |
| 优化机制 | Pareto选点 | PID+Pareto |
| 主要优势 | 快速收敛 | 路径质量 |

---

## 🔬 算法对比实验系统

### 快速开始对比实验

```matlab
% 一键运行完整对比实验（推荐）
run_comparison_experiment

% 快速测试模式（5次运行）
run_comparison_experiment('quick_test', true)

% 自定义运行次数
run_comparison_experiment('num_runs', 50)
```

### 对比算法

| 算法 | 说明 | 特点 |
|------|------|------|
| Dynamic-RRT (Interval=4) | 单树+informed subset | 快速收敛 |
| Dynamic-RRT (Interval=8) | 保守配置 | 路径质量更好 |
| SC-RRT Basic | 双树基础版 | 稳定可靠 |
| SC-RRT with PID | 双树+PID控制 | 自适应采样 |
| **SC-RRT Adaptive** | 双树+PID+Pareto | **完整优化** ✨ |

### 输出内容

实验完成后自动生成：
- ✅ **LaTeX表格** - 直接插入论文
- ✅ **高质量图表** (PNG + EPS) - 论文图表
- ✅ **Excel数据** - 数据备份
- ✅ **统计检验** - 显著性分析
- ✅ **性能雷达图** - 综合对比
- ✅ **实验报告** - 文字总结

### 详细文档

完整使用指南请查看: [COMPARISON_GUIDE.md](COMPARISON_GUIDE.md)

---

**复现状态**: ✅ 核心算法已实现，✅ 对比实验系统已完成

**推荐流程**: 
1. 运行 `test_dynamic_rrt.m` 验证Dynamic RRT
2. 运行 `run_comparison_experiment` 进行完整对比
3. 查看 `comparison_results/` 获取论文数据
