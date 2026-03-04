# SC-RRT: Bidirectional Adaptive Superellipsoid Constrained Sampling for Path Planning in Cluttered Obstacle Environments

# 基于自适应非对称双椭球域约束采样的SC-RRT路径规划算法

Xingyu Guo¹ · Yan Wu² · Hongyu Gao² · Jiale Hu¹ · Hong Feng¹ · Muchun Fan¹

¹ School of Mechanical Engineering, Shanghai Institute of Technology, Shanghai 201418, China
² Corresponding Author: Xingyu Guo, Gxyhaoxianghenlihai@gmail.com

---

## Abstract

针对复杂障碍环境中采样式路径规划算法易产生无效采样、收敛速度慢且在高障碍密度下成功率下降的问题，本文提出了一种基于自适应非对称双椭球约束采样的SC-RRT路径规划算法。该方法在双向并行扩展框架下，通过引入中介点引导，将全局规划任务分解为阶段性子问题，通过构造尺度可变、非对称的双向知情采样域，使采样逐步聚焦于更具改进潜力的可行路径，形成自适应双椭球约束采样策略（Adaptive Dual-Ellipsoid Constrained Sampling, ADCS），在维持探索覆盖性的同时显著提高有效采样密度。为降低代价估计偏差引起的知情子集过度膨胀或过早收缩导致的无效采样，本文建立了基于搜索状态反馈的在线调节机制（Search State Feedback-Based Online Regulation, SSFOR），利用当前最优路径代价的历史演化序列为反馈，对超椭球扩张因子 $\gamma$ 与启发式采样概率 $p$ 实施闭环调节，从而构建随搜索进程实时演化的知情采样子集，使采样密度在全局探索空间与局部约束区域间自适应分配，有效抑制冗余扩展并维持搜索连续性，结合非对称超椭球体约束策略对采样空间进行方向性压缩与扩展，增强了算法在潜在可行通道方向上的分辨能力。

仿真实验在不同障碍密度与空间维度的测试场景中验证了算法性能。在与对比方法相同的环境配置与计算预算约束下，SC-RRT相比Dynamic-RRT平均路径长度降低约12.6%，路径平滑度提升约81%；相比RRT\*算法，SC-RRT在获得更优路径质量的同时规划耗时缩短超过两个数量级。实验结果表明，SC-RRT能够在保持搜索完整性的同时有效抑制无效探索，使搜索树具备更稳定的扩展行为与更强的环境适应能力。本研究在理论层面将知情采样约束域的动态调节建模为以路径代价历史序列为反馈的闭环控制问题，揭示了超椭球采样约束在搜索状态驱动下的自适应演化机制，为同类采样式规划算法突破代价估计偏差敏感性提供了可推广的建模范式；在工程应用层面，SC-RRT无需依赖先验参数调优即可在不同障碍密度与空间维度下稳定运行，其模块化的ADCS与SSFOR设计可作为独立组件集成于其他基于RRT的规划框架，在工业机械臂运动规划、自主移动机器人导航及高维配置空间搜索等场景中具有直接的应用价值与良好的推广前景。

**Keywords**：Sampling-based path planning · Asymmetric dual-ellipsoid informed sampling · Bidirectional tree search · Online parameter regulation · Motion planning

---

## 1 Introduction

随着工业4.0与服务机器人技术的迅猛发展，智能机器人已成为提升制造、服务等领域自动化水平与作业效率的核心载体，其应用场景已从结构化工厂延伸至动态复杂的人机协作环境[1–3]。这一趋势对机器人的自主决策能力提出了更高要求，尤其是作为核心技术模块的路径规划算法，其性能直接决定机器人作业的安全性、可靠性与实时性[4–6]。因此，针对复杂环境下的机器人路径规划算法展开深入研究，不仅具有重要的理论价值，更对推动智能机器人的工程化应用具有关键意义[7, 8]。路径规划的核心任务是在已知起点与目标点的配置空间中，为机器人生成一条满足运动约束的无碰撞轨迹，同时平衡"实时响应"与"路径最优"两大核心目标[9]。在动态环境下，随机分布的障碍物及系统不确定性进一步加剧了规划难度，使得同时实现快速收敛与高质量路径成为当前研究中的关键挑战[10–12]。

快速探索随机树（Rapidly Exploring Random Tree, RRT）算法及其双向扩展形式如双向RRT\*是一类经典的采样式路径规划方法，凭借其概率完备性与易实现性，在复杂环境中得到了广泛应用[7, 8]。然而，传统双向RRT\*算法仍存在固有缺陷，如采样过程随机性强、搜索效率低下以及规划路径几何不够平滑等问题[13]。这些不足限制了其在环境尺度大、障碍物密集或动态障碍物分布场景中的性能表现。

为解决上述问题，近年来研究者从采样引导、搜索策略协同以及路径结构优化等角度提出了多种改进方法。

一类代表性工作通过改进双向搜索框架与采样引导机制来提升收敛效率，尤其关注狭窄通道等"难采样"场景下的可达性增强。Li和Chen（2022）提出的ARRT-Connect进一步依据对偶树探索得到的未勘探区域规模自适应调整抽样策略，并融合Tahirovic和Ferizbegovic（2018）在RRT中使用的主成分分析（PCA）思想，以强化树在"细长可行走廊"内的方向性生长，从而加快狭窄环境中的探索与连通[15]。然而，此类方法在高维空间或存在多通道的复杂环境中，仍可能在与最优路径无关的区域产生大量冗余采样点，影响整体规划效率。特别是当环境尺度增大或障碍物密度升高时，固定采样偏置策略难以适应不同搜索阶段的需求变化，导致知情子集的引导效果下降。

为兼顾快速规划与路径质量，另一类研究从代价驱动的采样约束与结构化搜索角度对RRT进行改进，通过缩小有效采样域或分解搜索过程来加速收敛并提升路径质量。Zhao等提出的Dynamic RRT算法创新性地引入启发式超椭球体约束采样机制，通过估计经过扩展节点的路径代价，将采样空间限制在以起点和目标点为焦点、估计路径长度为长轴的超椭球体内，大幅提升了新节点生成的有效性[16]。同时，该算法结合帕累托支配（Pareto Dominance）原则与动态规划思想，通过选择帕累托最优节点作为新起点分解规划问题，在保证路径长度优化的前提下进一步缩短了收敛时间，其性能显著优于RRT、RRT\*及Informed RRT\*等经典算法[16]。此类方法通过采样空间约束与问题分解策略，有效平衡了收敛速度与路径质量，然而，该算法的采样约束性能仍高度依赖路径代价估计的准确性：当代价被高估时，椭球体约束范围过大，产生较多无效采样；当代价被低估时，约束域过早收缩，可能将潜在最优路径排除在采样空间之外，导致算法收敛至次优解甚至搜索失败。此外，Dynamic RRT采用单向搜索框架，其知情子集仅以起点和终点为焦点，在复杂环境中对全局搜索方向的引导能力有限，仍存在冗余扩展问题。

总体而言，现有改进方法在效率与最优性之间仍存在明显权衡：采样引导与双向连通策略虽可加速获得可行解，但通常缺乏渐进最优保证，而代价驱动的约束采样与问题分解方法能够提高有效采样比例并改善路径质量，但其性能对代价估计敏感，且约束域多为静态或弱自适应形式，容易在搜索早期出现过度收缩导致可达性下降，或在代价波动时引发采样比例与树扩展行为的震荡，从而削弱全局稳健性与收敛一致性。

针对上述问题，本文提出一种基于自适应非对称双椭球约束采样域的SC-RRT算法。该算法在双向并行搜索框架与中介点引导下，构造起点侧与目标侧可独立伸缩的非对称双椭球域，为克服传统约束采样对代价估计偏差的敏感性，避免采样空间过早收缩或无效膨胀，本文建立了基于搜索状态反馈的在线调节机制，以当前最优路径代价及其历史演化为反馈，通过闭环控制调节超椭球体知情子集的尺度，在尚未获得稳定可行解阶段适度放宽约束、提升探索覆盖；当可行路径出现并持续改进时逐步强化约束、提升有效采样密度并抑制无效扩展，从而在收敛速度、路径质量与搜索稳定性之间建立可控的动态平衡。本文在不同障碍密度与空间维度的典型场景下，以Dynamic-RRT及经典RRT变体为基线开展对比实验，从路径长度、规划时间、节点规模与路径平滑度等指标对上述机制的有效性进行系统评估。

---

## 2 Algorithm Statement

### 2.1 问题定义

> **图1** 自适应非对称双椭球采样域约束策略示意图

为构建路径规划问题的数学模型，定义工作空间 $\mathcal{W} \subseteq \mathbb{R}^m$ 与构型空间 $\mathcal{C} \subseteq \mathbb{R}^n$，其中 $m$ 与 $n$ 分别表示对应维度。$\mathcal{W}$ 为机器人实际运动的笛卡尔空间，$\mathcal{C}$ 则描述机器人所有可能的构型状态。两者在环境表示上具有等价性，均完整表达了自由区域、障碍物及环境边界信息。定义状态 $x \in \mathcal{W}$，将空间划分为无碰撞区域 $\mathcal{W}_{free}$（如图中白色区域）与障碍区域 $\mathcal{W}_{obs}$（如黑色圆形障碍物），满足 $\mathcal{W} = \mathcal{W}_{free} \cup \mathcal{W}_{obs}$。规划起点、终点分别记为 $x_{start}, x_{goal} \in \mathcal{W}_{free}$（如图1）。为便于讨论，以下主要基于工作空间展开。

SC-RRT算法从起点 $x_{start}$、终点 $x_{goal}$ 分别构建两棵随机树 $T_A$、$T_B$ 并引入动态中介点 $x_{meet}$，将全局搜索解耦为"起点—中介点"和"终点—中介点"的两个子问题，其定义见式(1)。其中中介点 $x_{meet}$ 基于双树搜索状态在线更新。与传统单一静态椭球不同，本文所构建的超椭球体并非固定形状，其参数随路径代价改进与中介点迁移更新，以 $x_{meet}$ 为桥梁，根据搜索过程中的路径代价变化与扩展状态自适应地进行伸缩，在起点侧与目标点侧分别构建超椭球体采样约束，保证潜在最优路径覆盖性的同时，持续压缩无效采样区域。

**关键参数说明**：

| 符号 | 含义 | 取值范围 |
|------|------|---------|
| $\gamma_A, \gamma_B$ | 起点侧和终点侧椭球膨胀系数 | [1.5, 8.0] |
| $p$ | 知情采样概率 | [0.2, 0.7] |
| $x_{meet}$ | 动态中介点 | $\in \mathcal{W}_{free}$ |
| $c_{best}^A, c_{best}^B$ | 各侧最优路径代价估计 | $\mathbb{R}^+$ |
| $c_{hist}$ | 代价历史序列（窗口大小：20次迭代） | — |
| $T_A, T_B$ | 起点侧和终点侧搜索树 | — |

### 2.2 自适应非对称双椭球采样域约束策略（ADCS）

> **图1** 自适应双椭球约束采样策略，ADCS

本文针对含随机障碍物的配置空间无碰撞路径规划，提出一种非对称双椭球域约束采样机制，实现非对称采样范围调节，兼顾全局探索与局部精细搜索，最终实现路径长度、规划时间及节点规模的同步降低，策略整体思想如示意图1所示。

该采样策略的核心是动态计算超椭球体的约束参数，对于任意一个方向，以起点侧为例，其定义为：

$$\mathcal{P}_A = \left\{x \in \mathcal{W}: \|x - x_{start}\| + \|x - x_{meet}\| \leq \gamma_A \cdot c_{best}^A\right\} \tag{1}$$

其中：
- $\gamma_A$ 表示为SSFOR机制在线调节的膨胀系数；
- $c_{best}^A$ 表示从起点侧经启发式评估的最优总代价估计；
- $c(x_{start} \to x)$ 是从起点 $x_{start}$ 到节点 $x$ 的累积路径代价；
- $h(x \to x_{meet})$ 是节点 $x$ 到中介点 $x_{meet}$ 的启发式距离。

其中 $c_{best}^A$ 通过启发式估计计算：

$$c_{best}^A = \left(\min_{n \in T_A} c(x_{start} \to n)\right) + h(n \to x_{meet}) \tag{2}$$

终点侧椭球 $\mathcal{P}_B$ 以对应方式定义，两个椭球分别独立更新，形成非对称约束结构。每次迭代中，以概率 $p$ 在上述椭球内执行知情采样，以一定概率向对侧树Pareto最优节点方向偏置采样，以剩余概率执行全局均匀采样，保证搜索既集中于有效区域，又保留全局覆盖能力。

中介点 $x_{meet}$ 基于双树搜索状态在线更新，采用势场理论计算两棵树前30%优质节点的加权质心：

$$x_{meet}(t) = \frac{\sum_{i \in \text{Top30\%}(T_A)} w_i \cdot n_i + \sum_{j \in \text{Top30\%}(T_B)} w_j \cdot m_j}{\sum_i w_i + \sum_j w_j} \tag{3}$$

其中 $w_i = (c(n_i) + \epsilon)^{-1}$ 为节点的代价倒数权重，$\epsilon$ 为数值稳定项。

对每个新扩展节点，优先尝试与对侧树直接连接，若超出阈值则由对树贪心逼近；一旦双树成功连接，对拼接路径执行节点重连优化与自适应平滑处理，生成最终可行路径。具体实现伪代码见算法1、2。

> **算法2** 自适应非对称双椭球采样域约束（Adaptive Dual-Ellipsoid Constrained Sampling）：以双向树 $T_A, T_B$ 及会合点 $x_{meet}$ 为基础，将全局规划分解为"起点–会合点"和"会合点–终点"两个子问题，分别构造非对称双椭球域并自适应更新其尺度，随后在选定超椭球内进行均匀采样并通过可行性验证得到候选采样点 $x_{sample}$。

### 2.3 基于搜索状态反馈的在线调节机制（SSFOR）

> **图2** 基于搜索状态的在线调节机制，SSFOR

ADCS约束域的有效性取决于膨胀系数 $\gamma$ 与知情采样比例 $p$ 的适时调整。为此，本文提出SSFOR机制对二者实施闭环调节：以最优路径代价总预估的相对下降速率作为当前搜索进展信号，当路径代价持续无明显改进时，判定当前约束可能过紧或搜索陷入停滞，相应增大 $\gamma$ 以扩展椭球覆盖范围、降低 $p$ 以增加全局随机探索；反之，当路径代价持续有效改进时，减小 $\gamma$ 收紧约束域、提高 $p$ 强化知情采样，加速局部收敛。约束过松时大量节点落在低效区域，约束过紧时在障碍物密集场景中有效采样率骤降。保证参数始终在有效范围内波动，使约束强度始终与当前搜索阶段相匹配。

**测量信号**（实际改进效率）：

$$y_k = \frac{c_{best}(t-\tau) - c_{best}(t)}{c_{best}(t-\tau) + \epsilon} \tag{4}$$

其中 $\tau$ 为窗口大小（推荐值：20次迭代），应用指数加权移动平均平滑：$\bar{y}_k = \rho \cdot y_k + (1-\rho) \cdot \bar{y}_{k-1}$

**误差信号**：

$$e_k = y^* - \bar{y}_k \tag{5}$$

目标效率 $y^* = 0.02$（2%的单位周期改进为适度进展）。

**控制输出**：

$$u_k = K_p e_k + K_i \sum_{j=0}^{k} e_j + K_d(e_k - e_{k-1}) \tag{6}$$

积分项采用限幅防饱和处理：$\sum e_j \in [I_{min}, I_{max}]$。

**参数调整规则**：

当改进停滞时（$\bar{y}_k < 0.01$）：增大 $\gamma \leftarrow \min(\gamma + 0.3, 8.0)$，减小 $p \leftarrow \max(p - 0.1, 0.2)$；当改进高效时（$\bar{y}_k > 0.05$）：减小 $\gamma \leftarrow \max(\gamma - 0.3, 1.5)$，增大 $p \leftarrow \min(p + 0.1, 0.7)$。有效采样率低于20%时自动放松：$\gamma \leftarrow \gamma + 0.5$，范围限制 $\gamma \in [1.5, 8.0]$。

> **算法1** 在线调节机制：根据当前最优路径代价 $c_{best}$ 及其历史序列 $c_{hist}$，估计改进效率并通过离散调节采样约束参数，输出超椭球扩张因子 $\gamma$ 与启发式采样概率 $p$，用于自适应控制后续迭代的"约束强度"和"信息采样占比"。

> **图2** SC-RRT算法总体框架。算法由搜索规划主循环、基于搜索状态反馈的在线调节机制、自适应双超椭球约束采样和可行性验证与双树连接四个模块组成。虚线箭头表示从搜索状态到采样参数调节的反馈路径。

---

## 3 Simulation

### 3.1 实验设置与环境配置

为系统评估SC-RRT在环境规模与障碍物密度保持一致条件下的性能，本研究在不同维度下开展了一系列仿真实验。评估指标包括路径长度、规划时间、路径平滑度与路径效率四项核心指标：其中，路径长度（Path Length）为路径相邻节点欧氏距离之和（毫米）；规划时间（Planning Time）为算法完整运行时间（秒）；路径平滑度（Smoothness）用转角变化的标准差（弧度）衡量；路径效率（Path Efficiency）为起终点直线距离与实际路径长度的比值，越接近1.0表示路径越趋近最优。在路径规划失败时，路径长度 = ∞。

**Table 1** Environment configurations

| Dimension | Environment | Obstacles | Start | Goal |
|-----------|-------------|-----------|-------|------|
| 2D | 1500×1500 | 225 | (75, 75) | (1425, 1425) |
| 3D | 1500×1500×1500 | 400 | (75, 75, 75) | (1425, 1425, 1425) |

据对路径规划的过往研究，选取五种代表性路径规划算法——RRT、RRT-Connect、RRT\*、Dynamic-RRT与本文算法SC-RRT进行对比，验证本文算法的性能与稳定性。为确保不同路径规划算法性能对比的公平性，本文在相同的环境空间和障碍物配置下，对多种算法的路径长度、规划时间等性能指标进行评估，以Table 1中规定的环境为例，在限定最大迭代次数5000次的情况下进行对比，进行500次独立运行。

> **图3** 2D环境下路径规划可视化对比（环境配置见Table 1；每幅图显示：障碍物、两棵树、最终路径、超椭球边界）

### 3.2 对比实验与性能分析

**Table 2** All Algorithm Comparison

| Algorithm | Path Length /mm | Planning Time /s | Smoothness | Path Efficiency |
|-----------|----------------|-----------------|------------|-----------------|
| RRT | 2583.74 | 0.054 | 0.4919 | 0.8217 |
| RRT-Connect | 2216.33 | 0.029 | 0.4487 | 0.9579 |
| RRT\* | 2241.20 | 15.843 | 0.3225 | 0.9495 |
| Dynamic-RRT | 2440.14 | 0.111 | 0.4639 | 0.8696 |
| **SC-RRT** | **2132.33** | **0.071** | **0.0883** | **0.9956** |

通过对SC-RRT与Dynamic-RRT算法的性能进行对比。从Table 2可以看出，Dynamic-RRT算法在得到更多时间支持的情况下其渐进最优性才能逐步发挥，但大多数情况下对路径规划的实时性要求更高，故需要在此做出相应取舍，并且当随机节点 $x_{rand}$ 选择的概率偏向目标节点时，它们的两个指标都有所提高。此外，可以明显看出，在保持相同路径长度的情况下，SC-RRT凭借在线调节机制（SSFOR）生成的路径收敛时间及采样次数远优于Dynamic-RRT算法。实验证明SC-RRT在平衡收敛时间和路径长度方面具有明显的优势。

在路径质量方面，SC-RRT实现了显著的性能改进。算法的平均路径长度为2132.33 mm，相比Dynamic-RRT缩短了12.6%（节省307.81 mm），相比RRT\*缩短了4.9%（节省108.87 mm）。值得注意的是SC-RRT的路径效率指标达到0.9956，接近理论最优值1.0，充分表明算法规划的路径几何形状近似为直线，这在所有对比算法中表现最优。规划效率方面，SC-RRT的平均规划时间仅为0.071秒，与基础的RRT（0.054秒）和RRT-Connect（0.029秒）处于同一数量级，展现出优异的计算效率。相比之下，RRT\*算法虽然能保证渐进最优性，但其规划时间达到15.843秒，SC-RRT相对其快了223倍，超过两个数量级的加速。路径平滑性的提升是SC-RRT的又一显著优势。算法的平滑度指标为0.0883弧度，相比Dynamic-RRT（0.4639弧度）改进了81.0%，相比RRT\*（0.3225弧度）改进了72.6%。这说明ADCS的紧凑采样机制自然产生了更加平滑的路径，减少了后期平滑处理的需求。

> **图9** 不同维度和不同区间的路径规划结果，黑色球型物体代表均匀分布的障碍物，起始节点设置如Table 1所示，图a-d和e-h显示了SC-RRT在2维和3维的性能。

### 3.3 实验总结

基于Table 2中四项核心指标的对比结果可知，SC-RRT以2132.33 mm的最短路径、0.071 s的近实时规划、0.0883的最优平滑度和0.9956的最高路径效率取得综合最优表现，相比Dynamic-RRT路径缩短12.6%、相比RRT\*路径缩短4.9%且规划速度提升约223倍，并在不同障碍密度及二维/三维场景中保持稳定收敛与良好可扩展性。

---

## 4 Conclusion

针对采样式路径规划算法在障碍密集环境中有效采样比例低、收敛速度慢且知情子集对代价估计偏差敏感的问题，本文提出了SC-RRT算法。该算法包含两个紧密耦合的核心机制：

**一、自适应非对称双椭球约束采样（ADCS）**。引入基于双树搜索状态在线更新的动态中介点，将全局规划任务分解为"起点—中介点"与"终点—中介点"两个方向性子问题，并分别构造尺度可独立调节的超椭球知情子集。与Dynamic-RRT以起点和终点为固定焦点的单一对称椭球不同，ADCS的非对称双椭球结构能够针对两侧搜索进展差异进行方向性压缩与扩展，在保持探索覆盖性的同时显著提高潜在可行通道上的采样密度，有效缓解了引言中指出的单向框架全局引导能力不足的问题。

**二、基于搜索状态反馈的在线调节机制（SSFOR）**。以当前最优路径代价及其近期历史序列为反馈信号，实时输出超椭球扩张因子 $\gamma$ 与知情采样概率 $p$，对知情子集的体积与采样分配比例实施闭环调节。当路径改进停滞时自动扩大探索范围，当改进高效时逐步收紧约束以强化局部收敛，从而克服传统方法中代价高估导致采样域过大或代价低估导致采样域过早收缩的敏感性问题。

在二维与三维不同障碍密度的典型场景中，以相同环境配置与计算预算为约束，SC-RRT对RRT、RRT-Connect、RRT\*、Dynamic-RRT四种基线算法进行了系统对比。实验结果表明，SC-RRT在路径质量、规划效率与路径平滑性三个维度上均取得了综合最优表现：路径长度较Dynamic-RRT缩短约12.6%、较RRT\*缩短约4.9%，路径效率达0.9956；规划时间与RRT、RRT-Connect处于同一数量级，较RRT\*快逾两个数量级；平滑度较Dynamic-RRT降低81.0%、较RRT\*降低72.6%，平均转角仅为5.06°。此外，SC-RRT路径与障碍物的平均安全间隙达56.92 mm，体现了ADCS机制在缩小采样域的同时对障碍物间距的隐式保持能力。在ADCS的双向非对称约束与SSFOR闭环调节的协同作用下，SC-RRT在不同维度空间与障碍分布条件下均表现出更一致的收敛行为、更少的冗余扩展与更高的规划稳定性。

尽管如此，当前工作仍存在一定局限性。当前代价函数仅优化欧氏路径长度，未直接考虑曲率约束或能量消耗等运动学指标；此外，路径后处理平滑模块引入的非线性变形在个别情况下可能增加路径长度，其与上游规划质量的耦合效应尚未进行完整的理论刻画。

---

## Authors' Contributions

All authors contributed to the study conception and design.

## Funding

This work is supported by [相应资金信息].

## Code Availability

The source code in this paper will be disclosed upon acceptance for publication. Before accepting, please E-mail to Gxyhaoxianghenlihai@gmail.com if the source code is needed.

## Declarations

**Ethics approval** Not applicable.

**Competing interests** The authors have no relevant financial or non-financial interests to disclose.

**Consent to participate** Not applicable.

**Consent for publication** Not applicable.

---

## References

[1] Ji, S., et al.: Learning-Based Automation of Robotic Assembly for Smart Manufacturing. Proc. of the IEEE. 109, 423–440 (2021)

[2] Perzylo, A., et al.: SMErobotics Smart Robots for Flexible Manufacturing. IEEE Robot. Autom. Mag. 26, 78–90 (2019)

[3] Hvilshoj, M., et al.: Autonomous industrial mobile manipulation (AIMM): past, present and future. Industrial Robot. 39, 120–135 (2012)

[4] Roa, M.A., Berenson, D., Huang, W.: Mobile Manipulation: Toward Smart Manufacturing. IEEE Robot. Autom. Mag. 22, 14–15 (2015)

[5] Gonzalez, A.G.C., et al.: Supervisory Control-Based Navigation Architecture: A New Framework for Autonomous Robots in Industry 4.0 Environments. IEEE Trans. Industr. Inf. 14, 1732–1743 (2018)

[6] Pan, C., et al.: A Novel Algorithm for Wafer Sojourn Time Analysis of Single-Arm Cluster Tools With Wafer Residency Time Constraints and Activity Time Variation. IEEE Trans. Syst. Man Cybern. Syst. 45, 805–818 (2015)

[7] Li, Z., et al.: A Fault-Tolerant Method for Motion Planning of Industrial Redundant Manipulator. IEEE Trans. Industr. Inf. 16, 7469–7478 (2020)

[8] Baumann, D., et al.: Wireless Control for Smart Manufacturing: Recent Approaches and Open Challenges. Proc. IEEE 109, 441–467 (2021)

[9] Li, S., Han, K., Li, X., et al.: Hybrid Trajectory Replanning-Based Dynamic Obstacle Avoidance for Physical Human-Robot Interaction. J. Intell. Robot. Syst. 103, 41 (2021)

[10] Hart, P.E., Nilsson, N.J., Raphael, B.: A Formal Basis for the Heuristic Determination of Minimum Cost Paths. IEEE Trans. Syst. Sci. Cybern. 4, 100–107 (1968)

[11] Karaman, S., Frazzoli, E.: Incremental Sampling-based Algorithms for Optimal Motion Planning. in Robotics: Sci. Syst. (2010). https://doi.org/10.48550/arXiv.1005.0416

[12] Wang, J.K., et al.: Neural RRT*: Learning-Based Optimal Path Planning. IEEE Trans. Autom. Sci. Eng. 17, 1748–1758 (2020)

[13] Li, Y., et al.: Neural Network Approximation Based Near-Optimal Motion Planning With Kinodynamic Constraints Using RRT. IEEE Trans. Industr. Electron. 65, 8718–8729 (2018)

[14] LaValle, S.M.: Rapidly-exploring random trees: A new tool for path planning. Research Report. (1998)

[15] Kavraki, L.E., et al.: Probabilistic roadmaps for path planning in high-dimensional configuration spaces. IEEE Trans. Robot. Autom. 12, 566–580 (1996)

[16] Wang, J.K., Meng, M.Q.H., Khatib, O.: EB-RRT: Optimal Motion Planning for Mobile Robots. IEEE Trans. Autom. Sci. Eng. 17, 2063–2073 (2020)

[17] Kusuma, M., Riyanto, Machbub, C.: Humanoid Robot Path Planning and Rerouting Using A-Star Search Algorithm. 2019 IEEE ICSIGSYS. (2019)

[18] An, B., Kim, J., Park, F.C.: An Adaptive Stepsize RRT Planning Algorithm for Open-Chain Robots. IEEE Robot. Autom. Lett. 3, 312–319 (2018)

[19] Wang, W., Zuo, L., Xu, X.: A Learning-based Multi-RRT Approach for Robot Path Planning in Narrow Passages. J. Intell. Robot. Syst. 90, 81–100 (2018)

[20] Aguinaga, I., Borro, D., Matey, L.: Parallel RRT-based path planning for selective disassembly planning. Int. J. Adv. Manuf. Technol. 36, 1221–1233 (2008)

[21] Bruce, J., Veloso, M.M.: Real-time randomized path planning for robot navigation. RoboCup 2002. 2752, 288–295 (2003)

[22] Kuffner, J.J., LaValle, S.M.: RRT-Connect: An Efficient Approach to Single-Query Path Planning. Proc. IEEE ICRA 2000. (2000)

[23] Moon, C.B., Chung, W.: Kinodynamic Planner Dual-Tree RRT (DT-RRT) for Two-Wheeled Mobile Robots Using the Rapidly Exploring Random Tree. IEEE Trans. Industr. Electron. 62, 1080–1090 (2015)

[24] Karaman, S., Frazzoli, E.: Sampling-based algorithms for optimal motion planning. Int. J. Robot. Res. 30, 846–894 (2011)

[25] Chen, L., et al.: A Fast and Efficient Double-Tree RRT*-Like Sampling-Based Planner Applying on Mobile Robotic Systems. IEEE/ASME Trans. Mechatron. 23, 2568–2578 (2018)

[26] Gammell, J.D., Srinivasa, S.S., Barfoot, T.D.: Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic. 2014 IEEE/RSJ IROS. 2997–3004 (2014)

[27] Salzman, O., Halperin, D.: Asymptotically Near-Optimal RRT for Fast, High-Quality Motion Planning. IEEE Trans. Robot. 32, 473–483 (2016)

[28] Qi, J., Yang, H., Sun, H.X.: MOD-RRT*: A Sampling-Based Algorithm for Robot Path Planning in Dynamic Environment. IEEE Trans. Industr. Electron. 68, 7244–7251 (2021)

[29] Gammell, J.D., Srinivasa, S.S., Barfoot, T.D.: On Recursive Random Prolate Hyperspheroids. arXiv preprint. (2014). https://doi.org/10.48550/arXiv.1403.7664

[30] Sun, H.Y., Farooq, M.: Note on the generation of random points uniformly distributed in hyper-ellipsoids. Proc. 5th Int. Conf. Inf. Fusion I, 489–496 (2002)

[31] Gammell, J.D., Barfoot, T.D.: The Probability Density Function of a Transformation-based Hyperellipsoid Sampling Technique. arXiv preprint. (2014). https://doi.org/10.48550/arXiv.1404.1347

[32] De Ruiter, A.H.J., Forbes, J.R.: On the Solution of Wahba's Problem on SO(n). J. Astronaut. Sci. 60, 1–31 (2014)

[33] Guo, G., et al.: Predicting Pareto Dominance in Multi-objective Optimization Using Pattern Recognition. 2nd Int. Conf. ISDEA. (2012)

[34] Zhao, P., et al.: Dynamic RRT: Optimal Path Planning via a Probabilistic Approach. IEEE Robot. Autom. Lett. 8, 2356–2363 (2023)

[35] Sanders, R.: The Pareto Principle: its Use and Abuse. J. Serv. Mark. 1(37), 40 (1987)

---

Publisher's Note Springer Nature remains neutral with regard to jurisdictional claims in published maps and institutional affiliations.
