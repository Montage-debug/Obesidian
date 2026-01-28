“# SC-RRT Algorithm Pseudocode for SCI Publication

---

## Algorithm 1: SC-RRT Bidirectional Path Planning Framework

```
Algorithm 1: SC-RRT Bidirectional Path Planning
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input:  x_start, x_goal, X_obs (obstacles), k_max
Output: Path π* or FAILURE

 1:  T_A ← {x_start}, T_B ← {x_goal}
 2:  x_meet ← (x_start + x_goal) / 2
 3:  c_A, c_B ← ∞, ∞
 4:  PID_state ← ∅, γ ← γ_max, p ← 0                ▷ Initialize PID controller
 5:  
 6:  for k = 1 to k_max do
 7:      /* Periodic dual-ellipsoid update */
 8:      if k mod τ = 0 then
 9:          x_meet ← PotentialFieldMeetPoint(T_A, T_B)
10:          (c_A, c_B) ← DualEllipsoidParams(T_A, T_B, x_meet)    ▷ Alg. 2
11:          (γ, p) ← PIDSamplingController(c_best, PID_state)     ▷ Alg. 4
12:      end if
13:      
14:      /* PID-controlled adaptive sampling */
15:      x_rand ← AdaptiveSample(x_start, x_meet, c_A, γ, p)       ▷ Alg. 2
14:      
15:      /* Pareto-optimal node selection for expansion */
16:      x_near ← ParetoNodeSelect(T_A, x_meet)                    ▷ Alg. 3
17:      x_new ← Steer(x_near, x_rand, δ)
18:      
21:      if CollisionFree(x_near, x_new) then
22:          F̂ ← G(x_new) + H(x_new)                               ▷ Standard A* cost
23:          T_A ← T_A ∪ {(x_new, parent: x_near, cost: F̂)}
22:          
23:          /* Attempt connection to opposite tree */
24:          x_connect ← NearestNode(T_B, x_new)
25:          if ||x_new - x_connect|| ≤ r_connect AND CollisionFree then
26:              π* ← ExtractPath(T_A, x_new, T_B, x_connect)
27:              return π*
28:          end if
29:      end if
30:      
31:      Swap(T_A, T_B)                                            ▷ Alternate trees
32:  end for
33:  return FAILURE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## Algorithm 2: Adaptive Dual-Ellipsoid Constrained Sampling

```
Algorithm 2: Adaptive Dual-Ellipsoid Constrained Sampling
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input:  T_A, T_B (search trees), x_start, x_goal, x_meet, β (buffer)
Output: x_sample (valid sample point)

/* ═══ Part A: Dual-Ellipsoid Parameter Computation ═══ */
 1:  c_min^A ← ||x_meet - x_start||
 2:  c_min^B ← ||x_goal - x_meet||
 3:  
 4:  /* Compute optimal semi-major axis for Ellipsoid A */
 5:  c_best^A ← min_{x_i ∈ T_A} { G(x_i) + ||x_i - x_meet|| }
 6:  
 7:  /* Compute optimal semi-major axis for Ellipsoid B */
 8:  c_best^B ← min_{x_j ∈ T_B} { G(x_j) + ||x_j - x_meet|| }
 9:  
10:  /* Apply buffer coefficient to ensure feasibility */
11:  c_best^A ← max(c_best^A, β · c_min^A)
12:  c_best^B ← max(c_best^B, β · c_min^B)
13:  
14:  /* Guarantee intersection reachability */
15:  if c_best^A + c_best^B < ||x_goal - x_start|| then
16:      γ ← 1.05 · ||x_goal - x_start|| / (c_best^A + c_best^B)
17:      c_best^A ← γ · c_best^A,  c_best^B ← γ · c_best^B
18:  end if

/* ═══ Part B: PID-Controlled Adaptive Sampling ═══ */
19:  if Random(0,1) < p AND c_best < ∞ then
20:      /* Informed sampling with PID expansion */
21:      c_expanded ← γ · c_best                     ▷ Apply PID expansion factor
22:      Select ellipsoid E with foci (f_1, f_2) and c_expanded
23:      x_c ← (f_1 + f_2) / 2, a ← c_expanded / 2
24:      b ← √(a² - ||f_2 - f_1||²/4)
25:      R ← RotationMatrix((f_2 - f_1) / ||f_2 - f_1||)
26:      L ← diag(a, b, ..., b)
27:      repeat
28:          x_ball ← UniformSampleUnitBall(n)
29:          x_sample ← R · L · x_ball + x_c
30:      until x_sample ∈ X_free ∩ Bounds
31:  else
32:      /* Global uniform sampling */
33:      x_sample ← UniformSample(Bounds)
34:  end if
35:  return x_sample
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## Algorithm 3: Pareto-Optimal Node Selection

```
Algorithm 3: Multi-Objective Pareto Node Selection
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input:  T (search tree), x_target, p_div (diversity probability)
Output: x_select (selected expansion node)

/* ═══ Part A: Multi-Objective Evaluation ═══ */
 1:  for each node x_i ∈ T do
 2:      v_i^(1) ← -Degree(x_i)                      ▷ Connectivity (maximize)
 3:      v_i^(2) ← F̂(x_i)                           ▷ Estimated cost (minimize)
 4:      v_i^(3) ← 1 - 1/max(G(x_i)/||x_i - x_0||, 1) ▷ Path tortuosity
 5:  end for

/* ═══ Part B: Pareto Dominance Classification ═══ */
 6:  P ← ∅                                           ▷ Pareto frontier set
 7:  for each x_i ∈ T do
 8:      dominated ← false
 9:      for each x_j ∈ T, j ≠ i do
10:          if v_j ≤ v_i (component-wise) AND v_j ≠ v_i then
11:              dominated ← true
12:              break
13:          end if
14:      end for
15:      if NOT dominated then
16:          P ← P ∪ {x_i}
17:      end if
18:  end for

/* ═══ Part C: Stochastic Selection with Diversity ═══ */
19:  if Random(0,1) < p_div AND (T \ P) ≠ ∅ then
20:      x_select ← RandomChoice(T \ P)              ▷ Diversity exploration
21:  else
22:      x_select ← RandomChoice(P)                  ▷ Pareto exploitation
23:  end if
24:  
25:  return x_select
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## Algorithm 4: PID-Controlled Adaptive Sampling

```
Algorithm 4: PID Sampling Controller
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input:  c_best (current best path cost), PID_state
Output: γ (expansion factor), p (informed sampling probability)

 1:  if PID_state = ∅ OR c_best = ∞ then
 2:      return (γ_max, 0)                        
 3:  end if
 4:  
 5:  /* Measure improvement efficiency */
 6:  y_k ← (c_hist[k-W] - c_best) / (c_hist[k-W] + ε)
 7:  ȳ_k ← ρ · ȳ_{k-1} + (1-ρ) · y_k              
 8:  
 9:  /* PID control */
10:  e_k ← y* - ȳ_k                                 
11:  I_k ← clip(I_{k-1} + e_k, I_min, I_max)       
12:  d_k ← ρ_d · d_{k-1} + (1-ρ_d) · (e_k - e_{k-1}) 
13:  u_k ← K_p · e_k + K_i · I_k + K_d · d_k
14:  
15:  /* Map to control variables */
16:  γ ← clip(γ_0 · exp(α_γ · u_k), γ_min, γ_max)  
17:  p ← clip(p_0 - α_p · tanh(u_k), p_min, p_max)  
18:  
19:  return (γ, p)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## Notation Reference Table

| Symbol | Description |
|--------|-------------|
| $T_A, T_B$ | Bidirectional search trees from start and goal |
| $x_{meet}$ | Dynamic meeting point between two trees |
| $c_{best}$ | Current best path cost |
| $c_{min}$ | Focal distance (Euclidean distance between foci) |
| $G(x)$ | Cost-to-come from tree root to node $x$ |
| $\hat{F}(x)$ | Estimated total cost (F = G + H) |
| $\beta$ | Ellipsoid buffer coefficient (default 1.2) |
| $\tau$ | Ellipsoid update interval (iterations) |
| $\delta$ | Step size for tree extension |
| $p_{div}$ | Non-Pareto selection probability (default 0.1) |
| $\mathcal{P}$ | Pareto frontier node set |
| **PID Parameters** |
| $\gamma$ | Ellipsoid expansion factor ($\gamma \geq 1$) |
| $p$ | Informed sampling probability ($0 \leq p \leq 1$) |
| $y_k$ | Improvement efficiency at iteration $k$ |
| $\bar{y}_k$ | Smoothed improvement efficiency (EMA) |
| $y^*$ | Target improvement efficiency (default 0.02) |
| $W$ | Sliding window size (default 50) |
| $K_p, K_i, K_d$ | PID gains (default 2.0, 0.2, 0.8) |

---

## PID Control Mechanism (Algorithm 4)

The PID controller dynamically adjusts sampling strategy based on path improvement efficiency:

**Measurement (Improvement Efficiency):**
$$y_k = \frac{c_{k-W} - c_k}{c_{k-W} + \epsilon}, \quad \bar{y}_k = \rho \bar{y}_{k-1} + (1-\rho) y_k$$

**PID Output:**
$$u_k = K_p e_k + K_i I_k + K_d d_k, \quad e_k = y^* - \bar{y}_k$$

**Control Variables:**
$$\gamma = \gamma_0 \cdot e^{\alpha_\gamma u_k}, \quad p = p_0 - \alpha_p \tanh(u_k)$$

**Behavioral Response:**
- Stagnation ($\bar{y}_k < y^*$): $u_k > 0 \Rightarrow \gamma \uparrow, p \downarrow$ (increase exploration)
- Rapid improvement ($\bar{y}_k > y^*$): $u_k < 0 \Rightarrow \gamma \downarrow, p \uparrow$ (increase exploitation)

---

# 中文注释版本

---

## 算法 1: SC-RRT 双向路径规划框架

```
算法 1: SC-RRT 双向路径规划
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入:  x_start (起点), x_goal (终点), X_obs (障碍物), k_max (最大迭代)
输出:  最优路径 π* 或 失败

 1:  T_A ← {x_start}, T_B ← {x_goal}          // 初始化双向树
 2:  x_meet ← (x_start + x_goal) / 2          // 初始交汇点为中点
 3:  c_A, c_B ← ∞, ∞                          // 椭球参数初始化
 4:  PID_state ← ∅, γ ← γ_max, p ← 0        // 初始化PID控制器
 5:  
 6:  for k = 1 to k_max do
 7:      /* 周期性更新双椭球体 */
 8:      if k mod τ = 0 then
 9:          x_meet ← 势场法更新交汇点(T_A, T_B)
10:          (c_A, c_B) ← 计算双椭球参数(...)           ▷ 算法 2
11:          (γ, p) ← PID采样控制器(c_best, PID_state)  ▷ 算法 4
12:      end if
13:      
14:      /* PID控制的自适应采样 */
15:      x_rand ← 自适应采样(x_start, x_meet, c_A, γ, p) ▷ 算法 2
14:      
15:      /* Pareto最优节点选择 */
16:      x_near ← Pareto节点筛选(T_A, x_meet)           ▷ 算法 3
17:      x_new ← 扩展(x_near, x_rand, 步长δ)
18:      
21:      if 无碰撞(x_near, x_new) then
22:          F̂ ← G(x_new) + H(x_new)                   // 标准A*代价
23:          T_A ← T_A ∪ {(x_new, 父节点: x_near, 代价: F̂)}
22:          
23:          /* 尝试连接对向树 */
24:          x_connect ← 最近节点(T_B, x_new)
25:          if ||x_new - x_connect|| ≤ r_connect 且 无碰撞 then
26:              π* ← 提取路径(T_A, x_new, T_B, x_connect)
27:              return π*
28:          end if
29:      end if
30:      
31:      交换(T_A, T_B)                                 // 双向交替扩展
32:  end for
33:  return 失败
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 算法 2: 自适应双椭球约束采样

```
算法 2: 自适应双椭球约束采样
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入:  T_A, T_B (搜索树), x_start, x_goal, x_meet, β (缓冲系数)
输出:  x_sample (有效采样点)

/* ═══ A部分: 双椭球参数计算 ═══ */
 1:  c_min^A ← ||x_meet - x_start||            // 椭球A焦距
 2:  c_min^B ← ||x_goal - x_meet||             // 椭球B焦距
 3:  
 4:  /* 计算椭球A的最优半长轴 (遍历树A所有节点) */
 5:  c_best^A ← min_{x_i ∈ T_A} { G(x_i) + ||x_i - x_meet|| }
 6:  
 7:  /* 计算椭球B的最优半长轴 (遍历树B所有节点) */
 8:  c_best^B ← min_{x_j ∈ T_B} { G(x_j) + ||x_j - x_meet|| }
 9:  
10:  /* 应用缓冲系数，确保椭球可行性 */
11:  c_best^A ← max(c_best^A, β · c_min^A)
12:  c_best^B ← max(c_best^B, β · c_min^B)
13:  
14:  /* 保证双椭球交集可达 */
15:  if c_best^A + c_best^B < ||x_goal - x_start|| then
16:      γ ← 1.05 · ||x_goal - x_start|| / (c_best^A + c_best^B)
17:      c_best^A ← γ · c_best^A,  c_best^B ← γ · c_best^B
18:  end if

/* ═══ B部分: PID控制的自适应采样 ═══ */
19:  if Random(0,1) < p 且 c_best < ∞ then
20:      /* 知情采样，使用PID膨胀系数 */
21:      c_expanded ← γ · c_best                 // 应用PID膨胀系数
22:      选择椭球 E，焦点为 (f_1, f_2)，半长轴为 c_expanded
23:      x_c ← (f_1 + f_2) / 2, a ← c_expanded / 2
24:      b ← √(a² - ||f_2 - f_1||²/4)
25:      R ← 旋转矩阵((f_2 - f_1) / ||f_2 - f_1||)
26:      L ← diag(a, b, ..., b)
27:      repeat
28:          x_ball ← 单位球内均匀采样(n维)
29:          x_sample ← R · L · x_ball + x_c
30:      until x_sample ∈ 自由空间 ∩ 边界内
31:  else
32:      /* 全局均匀采样 */
33:      x_sample ← 均匀采样(边界)
34:  end if
35:  return x_sample
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 算法 3: Pareto最优节点筛选

```
算法 3: 多目标Pareto节点筛选
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入:  T (搜索树), x_target (目标点), p_div (多样性概率)
输出:  x_select (选中的扩展节点)

/* ═══ A部分: 多目标评估 ═══ */
 1:  for 每个节点 x_i ∈ T do
 2:      v_i^(1) ← -节点度(x_i)                // 目标1: 连通性 (越大越好)
 3:      v_i^(2) ← F̂(x_i)                     // 目标2: 估计代价 (越小越好)
 4:      v_i^(3) ← 1 - 1/max(G(x_i)/||x_i-x_0||, 1)  // 目标3: 路径曲折度
 5:  end for

/* ═══ B部分: Pareto支配分类 ═══ */
 6:  P ← ∅                                     // Pareto前沿集合
 7:  for 每个 x_i ∈ T do
 8:      被支配 ← false
 9:      for 每个 x_j ∈ T, j ≠ i do
10:          if v_j 各分量 ≤ v_i 且 v_j ≠ v_i then   // j支配i
11:              被支配 ← true
12:              break
13:          end if
14:      end for
15:      if 未被支配 then
16:          P ← P ∪ {x_i}                     // 加入Pareto前沿
17:      end if
18:  end for

/* ═══ C部分: 随机选择策略 ═══ */
19:  if Random(0,1) < p_div 且 (T \ P) ≠ ∅ then
20:      x_select ← 随机选择(T \ P)            // 多样性探索
21:  else
22:      x_select ← 随机选择(P)                // Pareto前沿开发
23:  end if
24:  
25:  return x_select
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 算法 4: PID控制的自适应采样

```
算法 4: PID采样控制器
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入:  c_best (当前最优路径代价), PID_state
输出:  γ (膨胀系数), p (知情采样概率)

 1:  if PID_state = ∅ 或 c_best = ∞ then
 2:      return (γ_max, 0)                      // 初始最大探索
 3:  end if
 4:  
 5:  /* 测量改进效率 */
 6:  y_k ← (c_hist[k-W] - c_best) / (c_hist[k-W] + ε)
 7:  ȳ_k ← ρ · ȳ_{k-1} + (1-ρ) · y_k           // EMA平滑
 8:  
 9:  /* PID控制 */
10:  e_k ← y* - ȳ_k                              // 控制误差
11:  I_k ← clip(I_{k-1} + e_k, I_min, I_max)   // 积分项（抗饱和）
12:  d_k ← ρ_d · d_{k-1} + (1-ρ_d) · (e_k - e_{k-1}) // 滤波微分
13:  u_k ← K_p · e_k + K_i · I_k + K_d · d_k
14:  
15:  /* 映射到控制变量 */
16:  γ ← clip(γ_0 · exp(α_γ · u_k), γ_min, γ_max)  // 膨胀系数
17:  p ← clip(p_0 - α_p · tanh(u_k), p_min, p_max)  // 采样概率
18:  
19:  return (γ, p)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 符号说明表

| 符号 | 含义 |
|------|------|
| $T_A, T_B$ | 起点树和终点树 |
| $x_{meet}$ | 动态交汇点 |
| $c_{best}$ | 当前最优路径代价 |
| $c_{min}$ | 椭球焦距（焦点间欧氏距离） |
| $G(x)$ | 从树根到节点 $x$ 的实际路径代价 |
| $\hat{F}(x)$ | 估计总代价 (F = G + H) |
| $\beta$ | 椭球缓冲系数（默认1.2） |
| $\tau$ | 椭球更新周期（迭代次数） |
| $\delta$ | 树扩展步长 |
| $p_{div}$ | 非Pareto节点选择概率（默认0.1） |
| $\mathcal{P}$ | Pareto前沿节点集合 |
| **PID参数** |
| $\gamma$ | 椭球膨胀系数 ($\gamma \geq 1$) |
| $p$ | 知情采样概率 ($0 \leq p \leq 1$) |
| $y_k$ | 第 $k$ 次迭代的改进效率 |
| $\bar{y}_k$ | 平滑后的改进效率 (EMA) |
| $y^*$ | 目标改进效率（默认0.02） |
| $W$ | 滑动窗口大小（默认50） |
| $K_p, K_i, K_d$ | PID增益（默认2.0, 0.2, 0.8） |

---

*Generated for SC-RRT SCI Publication*
”这个伪代码你觉得怎么样，还能精简么