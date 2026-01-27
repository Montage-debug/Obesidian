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
 4:  
 5:  for k = 1 to k_max do
 6:      /* Periodic dual-ellipsoid update */
 7:      if k mod τ = 0 then
 8:          x_meet ← PotentialFieldMeetPoint(T_A, T_B)
 9:          (c_A, c_B) ← DualEllipsoidParams(T_A, T_B, x_meet)    ▷ Alg. 2
10:      end if
11:      
12:      /* Constrained sampling within active ellipsoid */
13:      x_rand ← DualEllipsoidSample(x_start, x_meet, c_A)        ▷ Alg. 2
14:      
15:      /* Pareto-optimal node selection for expansion */
16:      x_near ← ParetoNodeSelect(T_A, x_meet)                    ▷ Alg. 3
17:      x_new ← Steer(x_near, x_rand, δ)
18:      
19:      if CollisionFree(x_near, x_new) then
20:          F̂ ← AdaptivePIDCost(x_new, T_A, k)                    ▷ Eq. (8)-(12)
21:          T_A ← T_A ∪ {(x_new, parent: x_near, cost: F̂)}
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

/* ═══ Part B: Ellipsoid Sampling ═══ */
19:  Select ellipsoid E with foci (f_1, f_2) and c_best
20:  x_c ← (f_1 + f_2) / 2                           ▷ Ellipsoid center
21:  a ← c_best / 2                                  ▷ Semi-major axis
22:  b ← √(a² - ||f_2 - f_1||²/4)                   ▷ Semi-minor axis
23:  R ← RotationMatrix((f_2 - f_1) / ||f_2 - f_1||)
24:  L ← diag(a, b, ..., b)                          ▷ Scaling matrix
25:  
26:  repeat
27:      x_ball ← UniformSampleUnitBall(n)
28:      x_sample ← R · L · x_ball + x_c
29:  until x_sample ∈ X_free ∩ Bounds
30:  
31:  return x_sample
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

## Notation Reference Table

| Symbol | Description |
|--------|-------------|
| $T_A, T_B$ | Bidirectional search trees from start and goal |
| $x_{meet}$ | Dynamic meeting point between two trees |
| $c_{best}$ | Semi-major axis of ellipsoid (current best cost bound) |
| $c_{min}$ | Focal distance (Euclidean distance between foci) |
| $G(x)$ | Cost-to-come from tree root to node $x$ |
| $\hat{F}(x)$ | PID-modulated estimated total cost |
| $\beta$ | Ellipsoid buffer coefficient (default 1.2) |
| $\tau$ | Ellipsoid update interval (iterations) |
| $\delta$ | Step size for tree extension |
| $p_{div}$ | Non-Pareto selection probability (default 0.1) |
| $\mathcal{P}$ | Pareto frontier node set |

---

## Adaptive PID Cost Function (Referenced in Algorithm 1)

The adaptive PID cost modulation is defined as:

$$\hat{F}(x) = F(x) \cdot \psi, \quad \psi = 1 + w_\eta \cdot \tanh(u_{PID})$$

where:
$$u_{PID} = K_p(\alpha) \cdot e_k + K_i(\alpha) \cdot I_k + K_d(\alpha) \cdot D_k$$

**Adaptive Gain Scheduling:**
$$K_p(\alpha) = K_p^{base} \cdot (1 + \beta_p \cos(\pi\alpha))$$
$$K_i(\alpha) = K_i^{base} \cdot (1 - \beta_i \alpha^2)$$
$$K_d(\alpha) = K_d^{base} \cdot (1 + \beta_d \sin(\pi\alpha))$$

where $\alpha = k/k_{max}$ is the iteration progress ratio.

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
 4:  
 5:  for k = 1 to k_max do
 6:      /* 周期性更新双椭球体 */
 7:      if k mod τ = 0 then
 8:          x_meet ← 势场法更新交汇点(T_A, T_B)
 9:          (c_A, c_B) ← 计算双椭球参数(...)           ▷ 算法 2
10:      end if
11:      
12:      /* 在当前椭球内约束采样 */
13:      x_rand ← 双椭球采样(x_start, x_meet, c_A)      ▷ 算法 2
14:      
15:      /* Pareto最优节点选择 */
16:      x_near ← Pareto节点筛选(T_A, x_meet)           ▷ 算法 3
17:      x_new ← 扩展(x_near, x_rand, 步长δ)
18:      
19:      if 无碰撞(x_near, x_new) then
20:          F̂ ← 自适应PID代价(x_new, T_A, k)          ▷ 公式 (8)-(12)
21:          T_A ← T_A ∪ {(x_new, 父节点: x_near, 代价: F̂)}
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

/* ═══ B部分: 椭球内采样 ═══ */
19:  选择当前椭球 E，焦点为 (f_1, f_2)，半长轴为 c_best
20:  x_c ← (f_1 + f_2) / 2                     // 椭球中心
21:  a ← c_best / 2                            // 半长轴
22:  b ← √(a² - ||f_2 - f_1||²/4)             // 半短轴
23:  R ← 旋转矩阵((f_2 - f_1) / ||f_2 - f_1||)
24:  L ← diag(a, b, ..., b)                    // 缩放矩阵
25:  
26:  repeat
27:      x_ball ← 单位球内均匀采样(n维)
28:      x_sample ← R · L · x_ball + x_c       // 变换到椭球空间
29:  until x_sample ∈ 自由空间 ∩ 边界内
30:  
31:  return x_sample
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

## 符号说明表

| 符号 | 含义 |
|------|------|
| $T_A, T_B$ | 起点树和终点树 |
| $x_{meet}$ | 动态交汇点 |
| $c_{best}$ | 椭球半长轴（当前最优代价上界） |
| $c_{min}$ | 椭球焦距（焦点间欧氏距离） |
| $G(x)$ | 从树根到节点 $x$ 的实际路径代价 |
| $\hat{F}(x)$ | PID调制后的估计总代价 |
| $\beta$ | 椭球缓冲系数（默认1.2） |
| $\tau$ | 椭球更新周期（迭代次数） |
| $\delta$ | 树扩展步长 |
| $p_{div}$ | 非Pareto节点选择概率（默认0.1） |
| $\mathcal{P}$ | Pareto前沿节点集合 |

---

*Generated for SC-RRT SCI Publication*
”这个伪代码你觉得怎么样，还能精简么