# SC-RRT Algorithm 4: PID-Controlled Adaptive Sampling (Pseudocode)

**本文档是SC-RRT算法伪代码的补充，专门描述PID控制采样分布的机制**

---

## Algorithm 4: PID-Controlled Dynamic Informed Subset Sampling

```
Algorithm 4: PID-Controlled Adaptive Informed Sampling
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input:  c_best_k (current best path cost), PID_state, params
Output: γ_k (ellipsoid expansion factor), p_k (informed sampling probability)

/* ═══ Part A: Initialization (First Call) ═══ */
 1:  if PID_state = ∅ then
 2:      PID_state.c_hist ← []
 3:      PID_state.e_prev ← 0
 4:      PID_state.I ← 0
 5:      PID_state.d_filt ← 0
 6:      PID_state.ȳ ← 0
 7:      return (γ_max, 0)                      ▷ Max exploration initially
 8:  end if

/* ═══ Part B: Cost History Update ═══ */
 9:  PID_state.c_hist ← [PID_state.c_hist, c_best_k]
10:  
11:  if c_best_k = ∞ OR length(c_hist) < W + 1 then
12:      return (γ_max, 0)                      ▷ No solution yet: pure global sampling
13:  end if

/* ═══ Part C: Improvement Efficiency Measurement ═══ */
14:  c_old ← PID_state.c_hist[k - W]           ▷ Cost W iterations ago
15:  c_current ← c_best_k
16:  
17:  /* Compute normalized improvement rate */
18:  y_k ← clip((c_old - c_current)/(c_old + ε), 0, 1)
19:  
20:  /* EMA smoothing for stability */
21:  PID_state.ȳ ← ρ_y · PID_state.ȳ + (1 - ρ_y) · y_k

/* ═══ Part D: PID Error Computation ═══ */
22:  e_k ← y* - PID_state.ȳ                    ▷ Target efficiency - actual efficiency
23:  
24:  /* Filtered derivative (avoid noise) */
25:  PID_state.d_filt ← ρ_d · PID_state.d_filt + (1 - ρ_d) · (e_k - PID_state.e_prev)

/* ═══ Part E: Integral with Anti-Windup ═══ */
26:  I_cand ← clip(PID_state.I + e_k, I_min, I_max)
27:  
28:  /* Compute PID output with candidate integral */
29:  u_cand ← K_p · e_k + K_i · I_cand + K_d · PID_state.d_filt

/* ═══ Part F: Mapping to Control Variables ═══ */
30:  /* Expansion factor: larger when stagnant (u > 0) */
31:  γ_cand ← γ_0 · exp(α_γ · u_cand)
32:  γ_cand ← clip(γ_cand, γ_min, γ_max)
33:  
34:  /* Informed sampling probability: smaller when stagnant */
35:  p_cand ← p_0 - α_p · tanh(u_cand)
36:  p_cand ← clip(p_cand, p_min, p_max)

/* ═══ Part G: Anti-Windup Logic ═══ */
37:  saturated ← false
38:  
39:  /* Check γ saturation */
40:  if (γ_cand ≥ γ_max - ε AND e_k > 0) OR (γ_cand ≤ γ_min + ε AND e_k < 0) then
41:      saturated ← true
42:  end if
43:  
44:  /* Check p saturation */
45:  if (p_cand ≤ p_min + ε AND e_k > 0) OR (p_cand ≥ p_max - ε AND e_k < 0) then
46:      saturated ← true
47:  end if
48:  
49:  if saturated then
50:      /* Freeze integral, recompute with old I */
51:      u_k ← K_p · e_k + K_i · PID_state.I + K_d · PID_state.d_filt
52:      γ_k ← clip(γ_0 · exp(α_γ · u_k), γ_min, γ_max)
53:      p_k ← clip(p_0 - α_p · tanh(u_k), p_min, p_max)
54:  else
55:      PID_state.I ← I_cand                   ▷ Update integral
56:      γ_k ← γ_cand
57:      p_k ← p_cand
58:  end if

/* ═══ Part H: State Update ═══ */
59:  PID_state.e_prev ← e_k
60:  
61:  return (γ_k, p_k)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## Integration with Algorithm 1 (SC-RRT Bidirectional)

### Modification 1: Add PID Controller Call

在算法1的第5-13行增加PID控制器调用：

```
 5:  for k = 1 to k_max do
 6:      /* Periodic dual-ellipsoid update */
 7:      if k mod τ = 0 then
 8:          x_meet ← PotentialFieldMeetPoint(T_A, T_B)
 9:          (c_A, c_B) ← DualEllipsoidParams(T_A, T_B, x_meet)
10:          
11:          /* ★ NEW: PID-controlled sampling adaptation */
12:          (γ_A, p_A) ← PIDSamplingController(c_best, PID_state)    ▷ Alg. 4
13:      end if
14:      
15:      /* Constrained sampling with PID-controlled parameters */
16:      x_rand ← AdaptiveDualEllipsoidSample(x_start, x_meet, c_A, γ_A, p_A)
```

### Modification 2: Update Sampling Strategy

修改算法2的采样部分（第19-30行）：

```
/* ═══ Part B: Dynamic Ellipsoid Sampling with PID Control ═══ */
19:  if Random(0,1) < p_informed then
20:      /* Sample within expanded ellipsoid */
21:      Select ellipsoid E with foci (f_1, f_2) and c_best
22:      c_expanded ← γ · c_best                ▷ Apply PID expansion factor
23:      x_c ← (f_1 + f_2) / 2
24:      a ← c_expanded / 2                     ▷ Expanded semi-major axis
25:      b ← √(a² - ||f_2 - f_1||²/4)
26:      R ← RotationMatrix((f_2 - f_1) / ||f_2 - f_1||)
27:      L ← diag(a, b, ..., b)
28:      
29:      repeat
30:          x_ball ← UniformSampleUnitBall(n)
31:          x_sample ← R · L · x_ball + x_c
32:      until x_sample ∈ X_free ∩ Bounds
33:  else
34:      /* Global sampling (no ellipsoid constraint) */
35:      x_sample ← UniformSample(Bounds)
36:  end if
37:  
38:  return x_sample
```

---

## Complete Integrated Framework

```
Algorithm: SC-RRT with PID-Controlled Adaptive Sampling (Integrated)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input:  x_start, x_goal, X_obs, k_max
Output: Path π* or FAILURE

 1:  T_A ← {x_start}, T_B ← {x_goal}
 2:  x_meet ← (x_start + x_goal) / 2
 3:  c_A, c_B ← ∞, ∞
 4:  PID_state ← ∅                              ▷ Initialize PID controller
 5:  γ_A, p_A ← γ_max, 0                        ▷ Initial sampling params
 6:  
 7:  for k = 1 to k_max do
 8:      /* Periodic update */
 9:      if k mod τ = 0 then
10:          x_meet ← PotentialFieldMeetPoint(T_A, T_B)
11:          (c_A, c_B) ← DualEllipsoidParams(T_A, T_B, x_meet)
12:          
13:          /* Update best path cost */
14:          if path found then
15:              c_best ← PathLength(π_current)
16:          else
17:              c_best ← ∞
18:          end if
19:          
20:          /* PID-controlled sampling adaptation */
21:          (γ_A, p_A) ← PIDSamplingController(c_best, PID_state)  ▷ Alg. 4
22:      end if
23:      
24:      /* Adaptive sampling with PID control */
25:      if Random(0,1) < p_A AND c_A < ∞ then
26:          x_rand ← EllipsoidSample(x_start, x_meet, γ_A · c_A, bounds)
27:      else
28:          x_rand ← UniformSample(bounds)
29:      end if
30:      
31:      /* Standard RRT expansion */
32:      x_near ← NearestNode(T_A, x_rand)
33:      x_new ← Steer(x_near, x_rand, δ)
34:      
35:      if CollisionFree(x_near, x_new) then
36:          F̂ ← G(x_new) + H(x_new)             ▷ Standard A* cost (no PID modulation)
37:          T_A ← T_A ∪ {(x_new, parent: x_near, cost: F̂)}
38:          
39:          /* Try connection */
40:          x_connect ← NearestNode(T_B, x_new)
41:          if ||x_new - x_connect|| ≤ r_connect AND CollisionFree then
42:              π* ← ExtractPath(T_A, x_new, T_B, x_connect)
43:              c_best ← PathLength(π*)
44:              /* PID will adapt on next update based on this improvement */
45:          end if
46:      end if
47:      
48:      Swap(T_A, T_B), Swap(c_A, c_B)           ▷ Alternate trees
49:  end for
50:  
51:  return π* or FAILURE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## Parameter Table

| Parameter | Symbol | Default | Description |
|-----------|--------|---------|-------------|
| **PID Gains** |
| Proportional gain | $K_p$ | 2.0 | Response speed |
| Integral gain | $K_i$ | 0.2 | Steady-state elimination |
| Derivative gain | $K_d$ | 0.8 | Damping |
| **Measurement** |
| Window size | $W$ | 50 | Sliding window length |
| Target efficiency | $y^*$ | 0.02 | 2% improvement per window |
| Efficiency smoothing | $\rho_y$ | 0.9 | EMA factor |
| Derivative filtering | $\rho_d$ | 0.8 | Low-pass filter |
| **Control Bounds** |
| Integral limits | $I_{\min}, I_{\max}$ | -3.0, 3.0 | Anti-windup |
| Expansion factor | $\gamma_{\min}, \gamma_{\max}$ | 1.0, 4.0 | Ellipsoid size |
| Sampling probability | $p_{\min}, p_{\max}$ | 0.2, 0.95 | Informed sampling |
| **Mapping** |
| Initial expansion | $\gamma_0$ | 1.5 | Baseline |
| Expansion sensitivity | $\alpha_\gamma$ | 0.5 | Exponential rate |
| Initial probability | $p_0$ | 0.8 | Baseline |
| Probability sensitivity | $\alpha_p$ | 0.5 | Tanh rate |
| **Update** |
| Ellipsoid update interval | $\tau$ | 50 | Iterations |
| Epsilon | $\epsilon$ | $10^{-6}$ | Numerical guard |

---

## Notation Supplement

| Symbol | Description |
|--------|-------------|
| $\gamma_k$ | Ellipsoid expansion factor (>1 enlarges sampling region) |
| $p_k$ | Informed sampling probability (higher → more exploitation) |
| $y_k$ | Raw improvement efficiency at iteration $k$ |
| $\bar{y}_k$ | EMA-smoothed improvement efficiency |
| $y^*$ | Target improvement efficiency (setpoint) |
| $e_k$ | Control error: $y^* - \bar{y}_k$ |
| $I_k$ | Integral term (accumulated error) |
| $d_k$ | Filtered derivative of error |
| $u_k$ | PID output before mapping |
| $c_{\text{best}, k}$ | Best path cost at iteration $k$ |
| $c_{\text{old}}$ | Cost at $k-W$ (window start) |

---

## Control Flow Diagram

```
┌────────────────────────────────────────────────────────────┐
│           SC-RRT Main Loop (每次迭代)                        │
└──────────────────────┬─────────────────────────────────────┘
                       │
          ┌────────────▼─────────────┐
          │  每 τ 次迭代:             │
          │  1. 更新交汇点 x_meet     │
          │  2. 计算椭球参数 (c_A,c_B)│
          │  3. 调用PID控制器 ────────┼──────┐
          └────────────┬─────────────┘       │
                       │                     │
         ┌─────────────▼──────────┐   ┌─────▼───────────────────────┐
         │   生成采样点 x_rand     │   │ Algorithm 4: PID Controller  │
         │                         │   │                              │
         │  rand() < p_k?          │   │ 输入: c_best历史            │
         │    是: 椭球采样(γ_k膨胀)│◄──┤ 输出: (γ_k, p_k)           │
         │    否: 全局均匀采样     │   │                              │
         └─────────────┬───────────┘   │ 流程:                        │
                       │               │ 1. 测量改进率 y_k            │
         ┌─────────────▼──────────┐   │ 2. 计算误差 e_k = y* - y_k   │
         │  扩展树（标准A*代价）   │   │ 3. PID: u = Kp·e+Ki·I+Kd·d   │
         │  F = G + H (无PID调制)  │   │ 4. 映射: γ←f(u), p←g(u)      │
         └─────────────┬───────────┘   │ 5. 抗饱和处理                │
                       │               └──────────────────────────────┘
         ┌─────────────▼──────────┐
         │  尝试连接 T_A 和 T_B   │
         │  (更新 c_best)         │
         └────────────────────────┘
```

---

## Behavioral Analysis

### Scenario 1: 快速改进阶段
- **条件**: 算法找到多条改进路径，$\bar{y}_k > y^*$
- **误差**: $e_k < 0$（改进超出目标）
- **PID输出**: $u_k < 0$
- **结果**: $\gamma \downarrow$ （收紧椭球），$p \uparrow$ （增加知情采样）
- **解释**: 局部区域有效，加强开发（exploitation）

### Scenario 2: 卡顿阶段
- **条件**: 长时间无改进，$\bar{y}_k \ll y^*$
- **误差**: $e_k > 0$（改进不足）
- **PID输出**: $u_k > 0$（积分项累积）
- **结果**: $\gamma \uparrow$ （放大椭球），$p \downarrow$ （减少知情采样）
- **解释**: 陷入局部，增强探索（exploration）

### Scenario 3: 平稳过渡
- **条件**: $\bar{y}_k \approx y^*$
- **误差**: $e_k \approx 0$
- **PID输出**: 微分项主导（预测趋势）
- **结果**: 平滑调节，无震荡

---

## Key Differences from Previous Approach

| 特性 | 旧方法（代价调制） | 新方法（采样控制） |
|------|-------------------|-------------------|
| **被控对象** | 节点代价 $\hat{F}$ | 采样分布 $(γ, p)$ |
| **测量信号** | 单节点误差 | 窗口改进率 |
| **稳定性** | 噪声大（D项震荡） | 平滑（滤波+EMA） |
| **积分意义** | 模糊 | 明确：累积改进不足 |
| **物理解释** | 间接影响节点选择 | 直接控制探索-开发 |
| **闭环反馈** | 弱（代价不直接反馈） | 强（改进直接测量） |
| **调参难度** | 高（易震荡/饱和） | 低（鲁棒参数） |

---

## Example Execution Trace

```
迭代 | c_best | ȳ_k   | e_k    | u_k   | γ_k  | p_k   | 解释
-----|--------|-------|--------|-------|------|-------|------------------
0    | ∞      | 0     | -      | -     | 4.0  | 0.00  | 初始化：最大探索
100  | 1500   | 0.025 | -0.005 | -0.8  | 1.65 | 0.78  | 快速改进：收紧采样
200  | 1200   | 0.018 | 0.002  | 0.3   | 1.82 | 0.72  | 放缓：轻微放宽
300  | 1100   | 0.008 | 0.012  | 1.5   | 2.85 | 0.45  | 卡顿：显著放宽
400  | 1000   | 0.021 | -0.001 | -0.2  | 1.70 | 0.76  | 恢复：重新收紧
```

---

## Implementation Notes

1. **窗口大小选择**: 
   - 太小（W<20）：噪声大，PID震荡
   - 太大（W>100）：反应慢，滞后严重
   - 推荐：W=50（约每秒更新一次，假设100 Hz）

2. **目标效率设定**:
   - 太高（y*>0.05）：系统总是"不满足"，过度探索
   - 太低（y*<0.01）：容易满足，不足以推动改进
   - 推荐：y*=0.02（每50次迭代改进2%）

3. **增益调优策略**:
   - 先调Kp（P控制）观察响应速度
   - 加入Ki消除稳态误差（注意饱和）
   - 最后加Kd抑制震荡（使用滤波）

4. **边界条件**:
   - γ_max不宜过大（>6会导致无效采样）
   - p_min保留少量知情采样（>0.1避免完全盲目）

---

*本文档与 SC-RRT算法伪代码.md 配套使用*  
*版本: 1.0 (PID Sampling Control)*  
*日期: 2026-01-27*
