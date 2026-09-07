# SC-RRT 公平对比实验 v2

本目录是与历史 MATLAB/机械臂 Python 实验隔离的重构实验。原始文件未被覆盖；当时的两套代码和 SHA-256 清单保存在 `legacy_snapshot/20260903/`。

## 2026-09-07 补充材料

最新论文材料位于 `outputs/paper_completion_20260907/`。优先阅读其中的 `论文写作指南.md` 与 `结果解读.md`，使用新增地图聚类统计和等启动条件 SSFOR 对照。旧 `analyze_results.py` 的运行级检验不作为最新跨地图统计结论。

实现标签修订（保留原始运行ID与冻结源码，便于追溯）：

- `Dynamic-RRT` 是本地改写，候选筛选、估计代价等不等同原文 Algorithm 2–3；不能因旧类注释写了“复现”就称严格复现。
- `SC-RRT-no-ADCS` 实际关闭知情采样/反馈整组，不隔离纯非对称因素。
- 旧 `SC-RRT-no-SSFOR` 在首解即启用固定参数，启动早于完整方法。新增 `SC-RRT-fixed-matched` 对齐启动条件，作为主要反馈对照。
- SC 的 `_pareto_target` 是加权候选偏置，不是严格Pareto前沿。

## 锁定的公平性协议

- 主算法：RRT、RRT-Connect、RRT*、Informed-RRT*、Dynamic-RRT、SC-RRT。
- 消融：SC-RRT-no-ADCS 和 SC-RRT-no-SSFOR。
- 预算：5000 次 atomic extension attempt。每次包含目标选择后的 nearest、steer，非零步长再做边碰撞检查，最多插入一个节点。Connect 内部每推进一步都单独扣预算。
- 重布线不占扩展次数，其边检查计数和运行耗时保留；整体候选路径复核的底层检查未全部进入 `collision_checks`，该列不等同所有碰撞运算总量。
- 所有算法在首解后继续运行到 5000 次；同时报告首解和固定预算结果。
- 主表不运行任何算法专属后处理。
- 收敛检查点：100、250、500、1000、2000、3000、4000、5000。
- 环境与规划随机性分离。地图仅按几何规则和网格可行性生成，不用待比较算法筛图。
- 正式集：每个维度 10 张地图 × 每图 50 个规划种子，即每算法每维度 500 次。同地图—同种子为配对观测。
- 校准集：每维度 4 张地图 × 每图 5 个种子，与正式评价集种子及地图隔离。

## 指标

首解阶段报告成功率、首解扩展次数、时间、节点数、路径长度和效率。固定预算阶段报告已发现最短路径长度、效率、总时间、节点数、已计数边检查与重布线数。摘要统计同时保留 mean±SD 和 median[IQR]。最新配对差值使用地图→种子分层 bootstrap 与地图均值 signed-rank/Holm；成功率 Wilson 区间仅作不考虑聚类的描述性参考。5000次扩展不是固定秒数预算。

## 复现

```bash
python3 -m pytest -q tests
python3 run_benchmark.py --suite calibration --output results/calibration_main
python3 run_benchmark.py --suite formal --output results/formal_main
python3 run_benchmark.py --suite ablation --output results/formal_ablation
python3 analyze_results.py --main results/formal_main --ablation results/formal_ablation --output results/analysis
```

运行器会每完成一次规划立即追加 CSV；对同一输出目录重新执行会自动跳过已完成的三元组（地图、种子、算法）。
