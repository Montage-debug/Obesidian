# SC-RRT 补充算法伪代码 (PID控制 & 椭球可视化)

---

## Algorithm 4: Adaptive PID Cost Modulation (详细版)

### 完整LaTeX文档版本

```latex
\documentclass[conference]{IEEEtran}
\usepackage{algorithm}
\usepackage{algorithmic}
\usepackage{amsmath}
\usepackage{amssymb}

\begin{document}

\begin{algorithm}[H]
\caption{Adaptive PID Cost Modulation}
\label{alg:pid}
\begin{algorithmic}[1]
\REQUIRE $\mathbf{x}_n$ (current node), Tree, $\mathbf{x}_{goal}$, $k$ (iteration), $k_{max}$
\REQUIRE $e_{k-1}$ (previous error), $I_{k-1}$ (integral), $\eta$ (search efficiency)
\ENSURE $\hat{F}(\mathbf{x}_n)$ (modulated cost), error info

\STATE \textbf{/* Step 1: Compute Basic A* Cost */}
\STATE $G(\mathbf{x}_n) \leftarrow$ Cost-to-come from tree root
\STATE $H(\mathbf{x}_n) \leftarrow \|\mathbf{x}_n - \mathbf{x}_{goal}\|$
\STATE $F(\mathbf{x}_n) \leftarrow G(\mathbf{x}_n) + H(\mathbf{x}_n)$

\STATE \textbf{/* Step 2: Adaptive Gain Scheduling */}
\STATE $\alpha \leftarrow k / k_{max}$ \COMMENT{Progress ratio $\in [0, 1]$}
\STATE $K_p^{base}, K_i^{base}, K_d^{base} \leftarrow (0.15, 0.02, 0.08)$
\STATE $\beta_p, \beta_i, \beta_d \leftarrow (0.5, 0.6, 0.4)$
\STATE $K_p(\alpha) \leftarrow K_p^{base} \cdot (1 + \beta_p \cos(\pi \alpha))$
\STATE $K_i(\alpha) \leftarrow K_i^{base} \cdot (1 - \beta_i \alpha^2)$
\STATE $K_d(\alpha) \leftarrow K_d^{base} \cdot (1 + \beta_d \sin(\pi \alpha))$

\IF{$\alpha < 0.3$}
    \STATE $stage \leftarrow$ "explore"
\ELSIF{$\alpha < 0.7$}
    \STATE $stage \leftarrow$ "balance"
\ELSE
    \STATE $stage \leftarrow$ "converge"
\ENDIF

\STATE \textbf{/* Step 3: Error Computation */}
\STATE $F_{est} \leftarrow G(\mathbf{x}_n) + H(\mathbf{x}_n)$
\STATE $e_{rel} \leftarrow (F_{est} - L_{best}) / L_{best}$ \COMMENT{Relative error}
\STATE $e_{norm} \leftarrow e_{rel} / (1 + |e_{rel}|)$ \COMMENT{Normalized error}
\STATE $e_k \leftarrow 0.3 \cdot |e_{rel}| + 0.7 \cdot e_{norm}$ \COMMENT{Weighted error}

\STATE \textbf{/* Step 4: Integral Update with Anti-Windup */}
\STATE $\lambda \leftarrow 0.95$, $e_{th} \leftarrow 0.5$
\IF{$|e_k| < e_{th}$}
    \STATE $I_k \leftarrow \lambda \cdot I_{k-1} + e_k$ \COMMENT{Conditional integration}
\ELSE
    \STATE $I_k \leftarrow \lambda \cdot I_{k-1}$ \COMMENT{Leakage only}
\ENDIF
\STATE $I_k \leftarrow \text{clamp}(I_k, -5.0, 5.0)$ \COMMENT{Saturation}

\STATE \textbf{/* Step 5: Derivative Computation */}
\STATE $D_k \leftarrow e_k - e_{k-1}$

\STATE \textbf{/* Step 6: Fuzzy Gain Fine-Tuning */}
\STATE $\sigma_e \leftarrow 0.2$, $\sigma_d \leftarrow 0.1$
\STATE $\mu_{small} \leftarrow \exp(-(e_k / \sigma_e)^2)$ \COMMENT{Small error membership}
\STATE $\mu_{large} \leftarrow 1 - \mu_{small}$
\STATE $\mu_{stable} \leftarrow \exp(-(D_k / \sigma_d)^2)$ \COMMENT{Stable derivative}
\STATE $\mu_{changing} \leftarrow 1 - \mu_{stable}$
\STATE $\Delta K_p \leftarrow \mu_{large} \mu_{changing} \cdot 0.05 - \mu_{small} \mu_{stable} \cdot 0.02$
\STATE $\Delta K_i \leftarrow \mu_{small} \mu_{stable} \cdot 0.005 - \mu_{large} \mu_{changing} \cdot 0.002$
\STATE $\Delta K_d \leftarrow \textsc{OscillationDamping}()$ \COMMENT{Detect oscillation}
\STATE $K_p \leftarrow K_p(\alpha) + \Delta K_p$
\STATE $K_i \leftarrow K_i(\alpha) + \Delta K_i$
\STATE $K_d \leftarrow K_d(\alpha) + \Delta K_d$

\STATE \textbf{/* Step 7: PID Output Computation */}
\STATE $u_{PID} \leftarrow K_p \cdot e_k + K_i \cdot I_k + K_d \cdot D_k$

\STATE \textbf{/* Step 8: Efficiency-Based Weighting */}
\IF{$\eta > 0.3$}
    \STATE $w_\eta \leftarrow 1.0$
\ELSIF{$\eta < 0.1$}
    \STATE $w_\eta \leftarrow 0.7$
\ELSE
    \STATE $w_\eta \leftarrow 0.85$
\ENDIF

\STATE \textbf{/* Step 9: Cost Modulation */}
\STATE $\psi \leftarrow 1 + w_\eta \cdot \tanh(u_{PID})$
\STATE $\hat{F}(\mathbf{x}_n) \leftarrow \text{clamp}(\psi \cdot F(\mathbf{x}_n), 0.3 F, 3.0 F)$

\RETURN $\hat{F}(\mathbf{x}_n)$, $\{e_k, I_k, D_k, K_p, K_i, K_d\}$
\end{algorithmic}
\end{algorithm}

\end{document}
```

---

## Algorithm 5: Ellipsoid Visualization (绘制双椭球体)

### 完整LaTeX文档版本

```latex
\documentclass[conference]{IEEEtran}
\usepackage{algorithm}
\usepackage{algorithmic}
\usepackage{amsmath}
\usepackage{amssymb}

\begin{document}

\begin{algorithm}[H]
\caption{Dual-Ellipsoid Visualization}
\label{alg:ellipsoid-vis}
\begin{algorithmic}[1]
\REQUIRE $\mathbf{f}_1, \mathbf{f}_2$ (foci), $c_{best}$ (semi-major axis), $m$ (dimension)
\REQUIRE $color$ (RGB color), $\alpha_{face}$ (transparency)
\ENSURE Ellipsoid plot handle

\STATE \textbf{/* Step 1: Compute Ellipsoid Parameters */}
\STATE $a \leftarrow c_{best} / 2$ \COMMENT{Semi-major axis}
\STATE $c \leftarrow \|\mathbf{f}_2 - \mathbf{f}_1\| / 2$ \COMMENT{Semi-focal distance}
\STATE $b \leftarrow \sqrt{a^2 - c^2}$ \COMMENT{Semi-minor axis}
\IF{$b^2 < 10^{-10}$}
    \RETURN $\emptyset$ \COMMENT{Degenerate ellipsoid}
\ENDIF

\STATE \textbf{/* Step 2: Compute Center and Orientation */}
\STATE $\mathbf{x}_c \leftarrow (\mathbf{f}_1 + \mathbf{f}_2) / 2$ \COMMENT{Ellipsoid center}
\STATE $\mathbf{\hat{e}} \leftarrow (\mathbf{f}_2 - \mathbf{f}_1) / \|\mathbf{f}_2 - \mathbf{f}_1\|$ \COMMENT{Principal axis}

\STATE \textbf{/* Step 3: Rotation Matrix */}
\IF{$m = 2$}
    \STATE $\theta \leftarrow \text{atan2}(\hat{e}_y, \hat{e}_x)$
    \STATE $\mathbf{R} \leftarrow \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$
\ELSE \COMMENT{3D case using Rodrigues' rotation}
    \STATE $\mathbf{x}_{axis} \leftarrow [1, 0, 0]^T$
    \STATE $\mathbf{k} \leftarrow \mathbf{x}_{axis} \times \mathbf{\hat{e}}$ \COMMENT{Rotation axis}
    \IF{$\|\mathbf{k}\| < 10^{-10}$}
        \IF{$\mathbf{x}_{axis} \cdot \mathbf{\hat{e}} > 0$}
            \STATE $\mathbf{R} \leftarrow \mathbf{I}_3$
        \ELSE
            \STATE $\mathbf{R} \leftarrow \text{diag}(-1, 1, -1)$ \COMMENT{180° rotation}
        \ENDIF
    \ELSE
        \STATE $\mathbf{k} \leftarrow \mathbf{k} / \|\mathbf{k}\|$ \COMMENT{Normalize}
        \STATE $\phi \leftarrow \arccos(\mathbf{x}_{axis} \cdot \mathbf{\hat{e}})$ \COMMENT{Rotation angle}
        \STATE $[\mathbf{K}]_\times \leftarrow$ skew-symmetric matrix of $\mathbf{k}$
        \STATE $\mathbf{R} \leftarrow \mathbf{I} + \sin\phi \cdot [\mathbf{K}]_\times + (1-\cos\phi) \cdot [\mathbf{K}]_\times^2$
    \ENDIF
\ENDIF

\STATE \textbf{/* Step 4: Generate Ellipsoid Surface */}
\IF{$m = 2$}
    \STATE $\theta \leftarrow \text{linspace}(0, 2\pi, 100)$
    \STATE $\mathbf{P}_{ellipse} \leftarrow [a \cos\theta; b \sin\theta]$ \COMMENT{$2 \times 100$}
    \STATE $\mathbf{P}_{world} \leftarrow \mathbf{R} \cdot \mathbf{P}_{ellipse} + \mathbf{x}_c$
    \STATE $h \leftarrow \textsc{Plot}(\mathbf{P}_{world}(1,:), \mathbf{P}_{world}(2,:), color, \alpha_{face})$
\ELSE
    \STATE $[\phi, \psi] \leftarrow \text{meshgrid}(\text{linspace}(0, \pi, 30), \text{linspace}(0, 2\pi, 30))$
    \STATE $X \leftarrow a \sin\phi \cos\psi$
    \STATE $Y \leftarrow b \sin\phi \sin\psi$
    \STATE $Z \leftarrow b \cos\phi$ \COMMENT{Oblate ellipsoid}
    \FOR{$i = 1$ to $30$}
        \FOR{$j = 1$ to $30$}
            \STATE $\mathbf{p}_{local} \leftarrow [X_{ij}, Y_{ij}, Z_{ij}]^T$
            \STATE $\mathbf{p}_{world} \leftarrow \mathbf{R} \cdot \mathbf{p}_{local} + \mathbf{x}_c$
            \STATE $X'_{ij}, Y'_{ij}, Z'_{ij} \leftarrow \mathbf{p}_{world}$
        \ENDFOR
    \ENDFOR
    \STATE $h \leftarrow \textsc{Surf}(X', Y', Z', color, \alpha_{face})$
\ENDIF

\RETURN $h$ \COMMENT{Graphics handle}
\end{algorithmic}
\end{algorithm}

\end{document}
```

---

## 中文注释版本

---

### 算法 4: 自适应PID代价调制 (详细版)

```
算法 4: 自适应PID代价调制
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入:  x_n (当前节点), Tree, x_goal, k (迭代次数), k_max
       e_{k-1} (上次误差), I_{k-1} (积分项), η (搜索效率)
输出:  F̂(x_n) (调制代价), 误差信息

/* 步骤 1: 计算基本A*代价 */
 1:  G(x_n) ← 从树根到当前节点的实际代价
 2:  H(x_n) ← ||x_n - x_goal||
 3:  F(x_n) ← G(x_n) + H(x_n)

/* 步骤 2: 自适应增益调度 */
 4:  α ← k / k_max                                   // 进度比例 ∈ [0, 1]
 5:  K_p^base, K_i^base, K_d^base ← (0.15, 0.02, 0.08)
 6:  β_p, β_i, β_d ← (0.5, 0.6, 0.4)
 7:  K_p(α) ← K_p^base · (1 + β_p cos(πα))          // 余弦调制
 8:  K_i(α) ← K_i^base · (1 - β_i α²)               // 二次衰减
 9:  K_d(α) ← K_d^base · (1 + β_d sin(πα))          // 正弦调制

10:  if α < 0.3 then
11:      stage ← "探索阶段"
12:  else if α < 0.7 then
13:      stage ← "平衡阶段"
14:  else
15:      stage ← "收敛阶段"
16:  end if

/* 步骤 3: 误差计算 */
17:  F_est ← G(x_n) + H(x_n)
18:  e_rel ← (F_est - L_best) / L_best              // 相对误差
19:  e_norm ← e_rel / (1 + |e_rel|)                 // 归一化误差
20:  e_k ← 0.3·|e_rel| + 0.7·e_norm                 // 加权误差

/* 步骤 4: 积分更新 (带抗饱和) */
21:  λ ← 0.95, e_th ← 0.5
22:  if |e_k| < e_th then
23:      I_k ← λ·I_{k-1} + e_k                      // 条件积分
24:  else
25:      I_k ← λ·I_{k-1}                            // 仅泄漏
26:  end if
27:  I_k ← clamp(I_k, -5.0, 5.0)                    // 限幅

/* 步骤 5: 微分计算 */
28:  D_k ← e_k - e_{k-1}

/* 步骤 6: 模糊增益微调 */
29:  σ_e ← 0.2, σ_d ← 0.1
30:  μ_small ← exp(-(e_k/σ_e)²)                     // 小误差隶属度
31:  μ_large ← 1 - μ_small
32:  μ_stable ← exp(-(D_k/σ_d)²)                    // 稳定微分
33:  μ_changing ← 1 - μ_stable
34:  ΔK_p ← μ_large·μ_changing·0.05 - μ_small·μ_stable·0.02
35:  ΔK_i ← μ_small·μ_stable·0.005 - μ_large·μ_changing·0.002
36:  ΔK_d ← 振荡阻尼检测()                          // 检测振荡
37:  K_p ← K_p(α) + ΔK_p
38:  K_i ← K_i(α) + ΔK_i
39:  K_d ← K_d(α) + ΔK_d

/* 步骤 7: PID输出计算 */
40:  u_PID ← K_p·e_k + K_i·I_k + K_d·D_k

/* 步骤 8: 效率加权 */
41:  if η > 0.3 then
42:      w_η ← 1.0
43:  else if η < 0.1 then
44:      w_η ← 0.7
45:  else
46:      w_η ← 0.85
47:  end if

/* 步骤 9: 代价调制 */
48:  ψ ← 1 + w_η · tanh(u_PID)
49:  F̂(x_n) ← clamp(ψ·F(x_n), 0.3F, 3.0F)

50:  return F̂(x_n), {e_k, I_k, D_k, K_p, K_i, K_d}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

### 算法 5: 椭球体可视化绘制

```
算法 5: 双椭球体可视化绘制
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入:  f_1, f_2 (焦点), c_best (半长轴), m (维度)
       color (RGB颜色), α_face (透明度)
输出:  椭球体绘图句柄

/* 步骤 1: 计算椭球参数 */
 1:  a ← c_best / 2                                 // 半长轴
 2:  c ← ||f_2 - f_1|| / 2                          // 半焦距
 3:  b ← √(a² - c²)                                 // 半短轴
 4:  if b² < 10^(-10) then
 5:      return ∅                                   // 椭球退化
 6:  end if

/* 步骤 2: 计算中心和方向 */
 7:  x_c ← (f_1 + f_2) / 2                          // 椭球中心
 8:  ê ← (f_2 - f_1) / ||f_2 - f_1||               // 主轴方向

/* 步骤 3: 旋转矩阵计算 */
 9:  if m = 2 then
10:      θ ← atan2(ê_y, ê_x)
11:      R ← [cos(θ)  -sin(θ)]
               [sin(θ)   cos(θ)]
12:  else                                           // 3D情况
13:      x_axis ← [1, 0, 0]ᵀ
14:      k ← x_axis × ê                             // 旋转轴
15:      if ||k|| < 10^(-10) then
16:          if x_axis · ê > 0 then
17:              R ← I₃
18:          else
19:              R ← diag(-1, 1, -1)                // 180°旋转
20:          end if
21:      else
22:          k ← k / ||k||                          // 归一化
23:          φ ← arccos(x_axis · ê)                 // 旋转角
24:          [K]× ← k的反对称矩阵
25:          R ← I + sin(φ)·[K]× + (1-cos(φ))·[K]×² // Rodrigues公式
26:      end if
27:  end if

/* 步骤 4: 生成椭球曲面 */
28:  if m = 2 then
29:      θ ← linspace(0, 2π, 100)
30:      P_ellipse ← [a·cos(θ); b·sin(θ)]           // 2×100
31:      P_world ← R · P_ellipse + x_c
32:      h ← 绘制(P_world(1,:), P_world(2,:), color, α_face)
33:  else
34:      [φ, ψ] ← meshgrid(linspace(0,π,30), linspace(0,2π,30))
35:      X ← a·sin(φ)·cos(ψ)
36:      Y ← b·sin(φ)·sin(ψ)
37:      Z ← b·cos(φ)                               // 扁椭球
38:      for i = 1 to 30 do
39:          for j = 1 to 30 do
40:              p_local ← [X_ij, Y_ij, Z_ij]ᵀ
41:              p_world ← R · p_local + x_c
42:              X'_ij, Y'_ij, Z'_ij ← p_world
43:          end for
44:      end for
45:      h ← Surf(X', Y', Z', color, α_face)
46:  end if

47:  return h                                        // 图形句柄
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 与主算法的集成说明

### 算法4 (PID控制) 在主算法中的调用位置

在主算法 Algorithm 1 的第11行调用：
```
11:  F̂ ← AdaptivePIDCost(x_new, k)    ▷ Algorithm 4
```

**关键特性：**
- **时变增益调度**：根据迭代进度α动态调整Kp, Ki, Kd
- **模糊微调**：基于误差和微分的隶属度函数进行精细调节
- **抗饱和机制**：条件积分 + 泄漏因子 + 限幅
- **效率自适应**：根据搜索效率η调整控制器影响权重

### 算法5 (椭球可视化) 在主算法中的调用位置

在主算法 Algorithm 1 更新椭球参数后调用（伪代码未体现，实现层面）：
```
// 在第5行计算椭球参数后
ellipsoid_A ← VisualizeEllipsoid(x_start, x_meet, c_A, ...)  ▷ Algorithm 5
ellipsoid_B ← VisualizeEllipsoid(x_meet, x_goal, c_B, ...)   ▷ Algorithm 5
```

**关键特性：**
- **Rodrigues旋转公式**：精确计算3D旋转矩阵
- **退化检测**：避免b²接近0时的数值问题
- **自适应分辨率**：2D用100点，3D用30×30网格

---

## 数学公式总结 (LaTeX)

### PID增益调度公式

```latex
\begin{align}
K_p(\alpha) &= K_p^{base} \cdot \left(1 + \beta_p \cos(\pi\alpha)\right) \\
K_i(\alpha) &= K_i^{base} \cdot \left(1 - \beta_i \alpha^2\right) \\
K_d(\alpha) &= K_d^{base} \cdot \left(1 + \beta_d \sin(\pi\alpha)\right)
\end{align}
```

### 模糊隶属度函数

```latex
\begin{align}
\mu_{small}(e) &= \exp\left(-\frac{e^2}{\sigma_e^2}\right) \\
\mu_{stable}(D) &= \exp\left(-\frac{D^2}{\sigma_d^2}\right)
\end{align}
```

### Rodrigues旋转公式

```latex
\mathbf{R} = \mathbf{I} + \sin\phi \cdot [\mathbf{k}]_\times + (1-\cos\phi) \cdot [\mathbf{k}]_\times^2
```

其中 $[\mathbf{k}]_\times$ 是向量 $\mathbf{k}$ 的反对称矩阵：
```latex
[\mathbf{k}]_\times = \begin{bmatrix}
0 & -k_z & k_y \\
k_z & 0 & -k_x \\
-k_y & k_x & 0
\end{bmatrix}
```

---

## 符号说明补充

| 符号 | 含义 |
|------|------|
| $\alpha$ | 迭代进度比例，$\alpha = k/k_{max}$ |
| $K_p, K_i, K_d$ | PID控制器的比例、积分、微分增益 |
| $e_k, I_k, D_k$ | 当前误差、积分项、微分项 |
| $\mu$ | 模糊隶属度函数值 $\in [0, 1]$ |
| $\eta$ | 搜索效率指标 |
| $w_\eta$ | 效率加权因子 |
| $\psi$ | 代价调制因子 |
| $\mathbf{R}$ | 旋转矩阵 (2D: $2\times2$, 3D: $3\times3$) |
| $[\mathbf{k}]_\times$ | 向量$\mathbf{k}$的反对称矩阵 |
| $a, b, c$ | 椭球的半长轴、半短轴、半焦距 |

---

*Generated for SC-RRT SCI Publication - Supplementary Algorithms*
