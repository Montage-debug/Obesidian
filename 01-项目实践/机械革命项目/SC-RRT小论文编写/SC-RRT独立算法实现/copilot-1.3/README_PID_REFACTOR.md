# SC-RRT算法PID采样控制重构说明

## 一、核心改进思想

### 1.1 原有架构的问题
- **旧方案**：PID调节单个节点的代价函数 F = G + H，通过调节因子影响节点选择
- **问题**：
  - 被控量不明确：单个节点误差难以稳定测量
  - 微分项噪声大：节点误差变化剧烈
  - 积分项容易饱和：没有明确的物理意义
  - PID没有真正参与闭环控制

### 1.2 新架构的核心
- **新方案**：PID控制"知情采样子集（informed subset）"的收紧/放宽
- **被控量**：近期路径改进效率 y_k（窗口统计）
- **控制量**：
  - γ (gamma): 椭球体膨胀系数 [1.0, 4.0]
  - p (p_informed): 知情采样概率 [0.2, 0.95]
- **控制逻辑**：
  - 改进缓慢 → 增大γ、减小p（放大采样范围、增加全局探索）
  - 改进顺利 → 减小γ、增大p（收紧采样范围、加强局部开发）

## 二、实现细节

### 2.1 新增模块：PIDSamplingController.m

**功能**：根据窗口统计的路径改进效率，动态调节采样策略

**关键参数**：
```matlab
WindowSize = 50           % 滑动窗口大小
TargetEfficiency = 0.02   % 目标改进效率（2%）
Kp = 2.0, Ki = 0.2, Kd = 0.8  % PID增益
RhoY = 0.9                % 改进率平滑系数
RhoD = 0.8                % 微分滤波系数
```

**核心流程**：
1. 维护代价历史窗口 c_hist
2. 计算窗口改进率：y_k = (c_old - c_current) / (c_old + ε)
3. EMA平滑：ȳ_k = ρ·ȳ_{k-1} + (1-ρ)·y_k
4. 计算误差：e_k = y* - ȳ_k
5. PID更新：u_k = Kp·e_k + Ki·I_k + Kd·d_k
6. 映射到控制量：
   - γ_k = γ₀·exp(α_γ·u_k)
   - p_k = p₀ - α_p·tanh(u_k)
7. 抗饱和处理：若γ或p达到边界且误差继续推向边界，冻结积分

### 2.2 修改的模块

#### SamplingModule.m
- 椭球体采样支持动态膨胀系数 gamma
- 函数签名：`ellipsoidSampling(startPoint, goalPoint, cBest, bounds, m, gamma)`
- 膨胀后的半长轴：a = (gamma * cBest) / 2

#### SC_RRT_Bidirectional.m
**主要改动**：
1. 初始化PID采样控制器状态 `pidSamplingState`
2. 在交汇点更新时，调用 `PIDSamplingController` 获取 gamma 和 p_informed
3. 采样策略改为：
   ```matlab
   if rand < pInformedA && ~isinf(c_best_A)
       randomPointA = SamplingModule('ellipsoid', ..., gammaA);
   else
       randomPointA = SamplingModule('uniform', bounds, goalPoint);
   end
   ```
4. 移除旧的代价调制代码，改用标准 F = G + H

## 三、测试验证

### 3.1 测试文件：test_pid_sampling.m

**测试场景**：
1. 初始化测试（无解状态）
2. 模拟路径改进过程：
   - 快速改进期：gamma↓, p↑
   - 改进放缓期：gamma↑, p↓
   - 卡顿期：gamma→max, p→min
   - 恢复改进期：gamma↓, p↑
3. 边界条件测试

### 3.2 运行主算法

```matlab
% 在 main.m 中配置
configParams.sc_rrt_mode = 'adaptive';  % 启用PID采样控制
configParams.env_idx = 2;               % 选择环境

% 运行
main
```

**预期输出示例**：
```
迭代200: L_best=1500.2, gamma=1.82, p=0.72, 效率=85%, y=0.0195
迭代400: L_best=1200.5, gamma=2.15, p=0.65, 效率=78%, y=0.0162
迭代600: L_best=1050.3, gamma=3.20, p=0.38, 效率=72%, y=0.0081
```

## 四、理论优势

### 4.1 闭环控制的合理性
- **明确的被控量**：窗口统计的改进效率，物理意义清晰
- **稳定的测量**：窗口平滑避免了单点噪声
- **有意义的积分**：累积的改进不足会持续推动探索
- **有效的微分**：改进趋势变化反映算法动态

### 4.2 自适应性
- 算法自动感知搜索状态
- 卡顿时自动放宽采样（增强探索）
- 顺利时自动收紧采样（加速收敛）
- 无需手动调参适应不同环境

### 4.3 与双椭球体的协同
- PID控制采样分布，双椭球体定义采样空间
- γ动态调节椭球体大小，p动态调节使用频率
- 两者配合实现探索-开发平衡的自适应调度

## 五、后续工作

### 5.1 伪代码编写
- 编写 Algorithm 4: PID-Controlled Adaptive Sampling
- 确保与现有Algorithm 1-3接口对齐
- 包含完整的数学符号和注释

### 5.2 性能对比实验
- 对比 basic / pid / adaptive 三种模式
- 统计：规划时间、路径长度、采样效率、成功率
- 可视化：γ和p的动态变化曲线

### 5.3 参数调优指南
- Kp：主要影响响应速度（建议1.5-3.0）
- Ki：影响稳态误差消除（建议0.1-0.5）
- Kd：影响超调和稳定性（建议0.5-1.2）
- WindowSize：影响反应灵敏度（建议30-100）

## 六、代码文件清单

| 文件 | 状态 | 说明 |
|------|------|------|
| PIDSamplingController.m | ✅ 新增 | PID采样控制器 |
| SamplingModule.m | ✅ 修改 | 支持gamma膨胀 |
| SC_RRT_Bidirectional.m | ✅ 修改 | 集成PID采样控制 |
| test_pid_sampling.m | ✅ 新增 | 单元测试 |
| main.m | ⚠️ 无需修改 | 主程序 |
| CostModule.m | ⚠️ 已弃用 | 旧PID代价调制（已移除调用） |
| adaptivePIDGains.m | ⚠️ 已弃用 | 旧自适应增益（已移除调用） |

## 七、使用说明

### 7.1 快速测试
```matlab
cd copilot-1.3
test_pid_sampling  % 单元测试PID控制器
```

### 7.2 运行完整算法
```matlab
cd copilot-1.3
main  % 运行主程序（默认adaptive模式）
```

### 7.3 对比实验
```matlab
% 修改 main.m 中的配置
configParams.sc_rrt_mode = 'basic';     % 基础模式（无PID）
configParams.sc_rrt_mode = 'adaptive';  % 自适应模式（PID采样控制）
```

---

**重构完成时间**：2026-01-27  
**版本**：copilot-1.3 (PID Sampling Control)  
**核心创新**：PID控制采样分布而非节点代价
