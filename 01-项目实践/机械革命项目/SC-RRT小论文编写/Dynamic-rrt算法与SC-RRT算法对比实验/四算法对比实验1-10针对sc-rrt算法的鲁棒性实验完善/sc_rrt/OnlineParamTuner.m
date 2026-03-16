function [tunerState, selectedParams] = OnlineParamTuner(tunerState, context, reward, varargin)
% OnlineParamTuner - 基于LinUCB上下文多臂老虎机的SSFOR参数在线自适应调优
%
% 核心算法：LinUCB (Li et al., 2010) 上下文老虎机
%   在搜索过程中，将SSFOR-PID控制器的参数选择建模为上下文老虎机问题。
%   每次SSFOR更新时：
%     1. 提取搜索状态上下文特征 x_t ∈ R^d
%     2. 利用LinUCB选择最优参数臂 a_t = argmax_{a} (θ_a' x_t + α√(x_t' A_a^{-1} x_t))
%     3. 观测奖励 r_t（搜索性能改进）
%     4. 在线更新模型参数：A_a ← γA_a + x_t x_t', b_a ← γb_a + r_t x_t
%
%   替代原始硬编码 Kp=2.0, Ki=0.2, Kd=0.8，实现学习式SSFOR参数自适应。
%
% 上下文特征 (6维 + 1偏置 = 7维):
%   x = [α, Δc, balance, hasSol, γ_norm, p, 1]'
%   α       - 搜索进度比 (iter/maxIter) ∈ [0,1]
%   Δc      - 近期代价改进率 ∈ [0,1]
%   balance - 双树节点比 sizeA/(sizeA+sizeB) ∈ [0,1]
%   hasSol  - 是否已找到可行解 ∈ {0,1}
%   γ_norm  - 当前椭球膨胀系数归一化 (γ-1)/3 ∈ [0,1]
%   p       - 当前知情采样概率 ∈ [0,1]
%
% 臂空间 (K=8 种参数配置):
%   {Kp, Ki, Kd, TargetEfficiency, CrossTreeRatio}
%   覆盖从保守探索到积极收敛的完整参数谱
%
% 输入:
%   tunerState - 调优器状态结构体（首次传[]自动初始化）
%   context    - 搜索状态上下文结构体:
%                .alpha            搜索进度比
%                .cost_improvement 近期代价改进率
%                .tree_balance     双树节点比
%                .has_solution     是否有可行解
%                .gamma_current    当前gamma值
%                .p_current        当前p_informed值
%   reward     - 上一轮参数配置的奖励信号（首次传NaN）
%
% 可选参数:
%   'ExplorationRate' - LinUCB探索系数α（默认1.0，越大越倾向探索）
%   'WarmupRounds'    - 预热轮数（默认16，确保每臂至少2次）
%   'ForgetFactor'    - 遗忘因子γ（默认0.995，适应非平稳搜索过程）
%
% 输出:
%   tunerState     - 更新后的调优器状态
%   selectedParams - 选中的参数配置结构体:
%                    .Kp, .Ki, .Kd       SSFOR-PID增益
%                    .TargetEfficiency    PID目标效率
%                    .CrossTreeRatio      对树连接采样比例
%                    .arm_index           选中的臂编号
%
% 参考文献:
%   [1] Li, L., et al. "A contextual-bandit approach to personalized news 
%       article recommendation." WWW 2010.

%% ========== 高性能参数解析 ==========
explorationRate = 1.0;       % LinUCB α参数（探索-利用权衡）
warmupRounds = 16;           % 预热轮数（K=8 × 2 = 16）
forgetFactor = 0.995;        % 遗忘因子（使模型逐渐遗忘旧经验）

ni = 1;
while ni <= length(varargin)
    if ischar(varargin{ni})
        switch varargin{ni}
            case 'ExplorationRate', explorationRate = varargin{ni+1}; ni = ni + 2;
            case 'WarmupRounds',    warmupRounds = varargin{ni+1};    ni = ni + 2;
            case 'ForgetFactor',    forgetFactor = varargin{ni+1};    ni = ni + 2;
            otherwise, ni = ni + 2;
        end
    else
        ni = ni + 1;
    end
end

%% ========== 臂定义：SSFOR参数配置空间 ==========
% 8种参数配置覆盖完整的PID增益与采样策略谱
% 列: [Kp, Ki, Kd, TargetEfficiency, CrossTreeRatio]
armConfigs = [
    1.0,  0.10, 0.3,  0.010, 0.10;   % Arm 1: 温和保守 — 大范围慢探索
    1.5,  0.15, 0.6,  0.015, 0.12;   % Arm 2: 中度保守 — 平衡偏探索
    2.0,  0.20, 0.8,  0.020, 0.15;   % Arm 3: 默认平衡 — 原始硬编码基线
    2.5,  0.20, 1.0,  0.025, 0.15;   % Arm 4: 中度积极 — 适度加速收敛
    3.0,  0.30, 1.2,  0.030, 0.18;   % Arm 5: 积极收敛 — 快速压缩探索域
    3.5,  0.15, 1.5,  0.035, 0.20;   % Arm 6: 高微分响应 — 快速适应变化
    1.5,  0.30, 0.4,  0.015, 0.25;   % Arm 7: 高积分+高连接 — 稳态精度
    2.5,  0.10, 1.5,  0.025, 0.08;   % Arm 8: 高微分低连接 — 灵敏独立搜索
];

K = size(armConfigs, 1);  % 臂数量 = 8
d = 7;                    % 上下文维度 = 6特征 + 1偏置

%% ========== 状态初始化 ==========
if isempty(tunerState)
    tunerState = struct();
    
    % LinUCB核心参数：每臂维护 A_k (d×d) 和 b_k (d×1)
    tunerState.A = cell(K, 1);
    tunerState.b = cell(K, 1);
    for k = 1:K
        tunerState.A{k} = eye(d);       % A_k = I_d (正则化初始)
        tunerState.b{k} = zeros(d, 1);  % b_k = 0
    end
    
    % 决策追踪
    tunerState.round = 0;               % 总决策轮次
    tunerState.last_arm = 0;            % 上一轮选择的臂
    tunerState.last_context = [];       % 上一轮的上下文向量
    
    % 统计信息
    tunerState.arm_counts = zeros(K, 1);       % 各臂累计选择次数
    tunerState.arm_total_reward = zeros(K, 1); % 各臂累计奖励
    tunerState.reward_history = [];            % 奖励时间序列
    tunerState.arm_history = [];               % 选臂时间序列
    tunerState.param_history = [];             % 参数历史矩阵 (N×5)
    
    % 保存配置
    tunerState.armConfigs = armConfigs;
    tunerState.K = K;
    tunerState.d = d;
end

%% ========== 用上一轮的奖励更新模型 ==========
if ~isnan(reward) && tunerState.last_arm > 0 && ~isempty(tunerState.last_context)
    a = tunerState.last_arm;
    x = tunerState.last_context;  % d×1 上下文向量
    
    % LinUCB在线更新（带遗忘因子以适应非平稳搜索过程）
    %   A_a ← γ_forget · A_a + x · x'     (协方差矩阵累积)
    %   b_a ← γ_forget · b_a + r · x       (奖励加权上下文累积)
    tunerState.A{a} = forgetFactor * tunerState.A{a} + x * x';
    tunerState.b{a} = forgetFactor * tunerState.b{a} + reward * x;
    
    % 数值稳定性：确保A矩阵正定（添加微小正则化）
    min_eig = min(real(eig(tunerState.A{a})));
    if min_eig < 1e-6
        tunerState.A{a} = tunerState.A{a} + 1e-6 * eye(d);
    end
    
    % 统计更新
    tunerState.arm_total_reward(a) = tunerState.arm_total_reward(a) + reward;
    tunerState.reward_history = [tunerState.reward_history, reward];
end

%% ========== 构造上下文特征向量 ==========
% 将所有特征归一化到 [0, 1] 保证LinUCB数值稳定
x_features = [
    max(0, min(1, context.alpha));                          % 搜索进度 [0,1]
    max(0, min(1, context.cost_improvement));               % 代价改进 [0,1]
    max(0, min(1, context.tree_balance));                   % 树平衡度 [0,1]
    double(context.has_solution);                           % 有解标志 {0,1}
    max(0, min(1, (context.gamma_current - 1.0) / 3.0));   % gamma归一化 [0,1]
    max(0, min(1, context.p_current));                      % p_informed [0,1]
];
x = [x_features; 1.0];  % 追加偏置项，总维度 d=7

%% ========== LinUCB臂选择 ==========
tunerState.round = tunerState.round + 1;

if tunerState.round <= warmupRounds
    % ===== 预热阶段：Round-Robin轮替 =====
    % 每个臂至少被选 warmupRounds/K ≈ 2 次，积累初始样本
    arm = mod(tunerState.round - 1, K) + 1;
else
    % ===== LinUCB主策略 =====
    % 对每个臂计算UCB值：UCB_a = θ_a' x + α · √(x' A_a^{-1} x)
    ucb_values = zeros(K, 1);
    
    for k = 1:K
        % 岭回归解：θ_k = A_k^{-1} · b_k
        A_inv = tunerState.A{k} \ eye(d);
        theta_k = A_inv * tunerState.b{k};
        
        % 利用项：线性预测期望奖励
        exploitation = theta_k' * x;
        
        % 探索项：UCB不确定性宽度
        uncertainty = x' * A_inv * x;
        exploration = explorationRate * sqrt(max(0, uncertainty));
        
        ucb_values(k) = exploitation + exploration;
    end
    
    % 选择UCB值最大的臂（微小随机扰动打破平局）
    ucb_values = ucb_values + rand(K, 1) * 1e-10;
    [~, arm] = max(ucb_values);
end

%% ========== 记录决策 ==========
tunerState.last_arm = arm;
tunerState.last_context = x;
tunerState.arm_counts(arm) = tunerState.arm_counts(arm) + 1;
tunerState.arm_history = [tunerState.arm_history, arm];

%% ========== 输出选中的参数配置 ==========
selectedParams = struct();
selectedParams.Kp              = armConfigs(arm, 1);
selectedParams.Ki              = armConfigs(arm, 2);
selectedParams.Kd              = armConfigs(arm, 3);
selectedParams.TargetEfficiency = armConfigs(arm, 4);
selectedParams.CrossTreeRatio  = armConfigs(arm, 5);
selectedParams.arm_index       = arm;

% 记录参数选择历史
tunerState.param_history = [tunerState.param_history; armConfigs(arm, :)];

end
