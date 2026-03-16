function [pidState, gamma, p_informed] = PIDSamplingController(pidState, c_best, varargin)
% PIDSamplingController - SSFOR在线调节机制：PID控制知情采样子集的动态调度
%
% 核心思想（论文Algorithm 1 - SSFOR）：
%   用离散PID控制器动态调节"超椭球膨胀系数gamma"和"知情采样概率p_informed"，
%   被控量为"近期路径代价改进效率"。
%
%   1. 测量量 y_k：滑动窗口内路径代价改进率（EMA平滑）
%   2. 误差 e_k：目标效率 y* - 实际效率 ybar
%   3. PID输出 u_k 映射到 gamma 和 p：
%      - 改进停滞（e_k > 0）→ 增大gamma（扩大探索）、减小p（降低知情占比）
%      - 改进顺利（e_k < 0）→ 减小gamma（收紧约束）、增大p（强化局部收敛）
%   4. 抗积分饱和（Anti-windup）确保稳定性
%
% 输入：
%   pidState  - PID状态结构体（首次传入[]自动初始化）
%   c_best    - 当前最优路径代价（Inf表示尚无可行解）
%   varargin  - 可选Name-Value参数（手动解析，无inputParser开销）
%
% 输出：
%   pidState    - 更新后的PID状态
%   gamma       - 超椭球膨胀系数 ∈ [GammaMin, GammaMax]
%   p_informed  - 知情采样概率   ∈ [PMin, PMax]

%% ========== 高性能参数解析（替代inputParser） ==========
% 默认值
WindowSize = 50;
TargetEfficiency = 0.02;
Kp = 2.0;  Ki = 0.2;  Kd = 0.8;
IMin = -3.0;  IMax = 3.0;
RhoY = 0.9;   RhoD = 0.8;
Gamma0 = 1.5;  GammaMin = 1.0;  GammaMax = 4.0;  AlphaGamma = 0.5;
P0 = 0.8;  PMin = 0.2;  PMax = 0.95;  AlphaP = 0.5;
Epsilon = 1e-6;

ni = 1;
while ni <= length(varargin)
    if ischar(varargin{ni})
        switch varargin{ni}
            case 'WindowSize',        WindowSize = varargin{ni+1};        ni = ni + 2;
            case 'TargetEfficiency',  TargetEfficiency = varargin{ni+1};  ni = ni + 2;
            case 'Kp',               Kp = varargin{ni+1};               ni = ni + 2;
            case 'Ki',               Ki = varargin{ni+1};               ni = ni + 2;
            case 'Kd',               Kd = varargin{ni+1};               ni = ni + 2;
            case 'IMin',             IMin = varargin{ni+1};             ni = ni + 2;
            case 'IMax',             IMax = varargin{ni+1};             ni = ni + 2;
            case 'RhoY',             RhoY = varargin{ni+1};             ni = ni + 2;
            case 'RhoD',             RhoD = varargin{ni+1};             ni = ni + 2;
            case 'Gamma0',           Gamma0 = varargin{ni+1};           ni = ni + 2;
            case 'GammaMin',         GammaMin = varargin{ni+1};         ni = ni + 2;
            case 'GammaMax',         GammaMax = varargin{ni+1};         ni = ni + 2;
            case 'AlphaGamma',       AlphaGamma = varargin{ni+1};       ni = ni + 2;
            case 'P0',               P0 = varargin{ni+1};               ni = ni + 2;
            case 'PMin',             PMin = varargin{ni+1};             ni = ni + 2;
            case 'PMax',             PMax = varargin{ni+1};             ni = ni + 2;
            case 'AlphaP',           AlphaP = varargin{ni+1};           ni = ni + 2;
            case 'Epsilon',          Epsilon = varargin{ni+1};          ni = ni + 2;
            otherwise,               ni = ni + 2;
        end
    else
        ni = ni + 1;
    end
end

%% ========== 初始化状态 ==========
if isempty(pidState)
    pidState = struct();
    pidState.c_hist = [];           % 代价历史序列
    pidState.e_prev = 0;            % 上次误差
    pidState.I = 0;                 % 积分累积项
    pidState.d_filt = 0;            % 滤波微分项
    pidState.ybar = 0;              % EMA平滑后的改进率
    pidState.iter_count = 0;        % 调用计数
    pidState.current_gamma = GammaMax;
    pidState.current_p = 0.0;
    pidState.current_y = 0;
    pidState.current_error = 0;
    
    % 初始输出：尚无可行解时最大探索
    gamma = GammaMax;
    p_informed = 0.0;
    return;
end

%% ========== 更新代价历史 ==========
pidState.iter_count = pidState.iter_count + 1;
pidState.c_hist = [pidState.c_hist, c_best];

%% ========== 计算改进效率（测量量 y_k） ==========
if isinf(c_best) || length(pidState.c_hist) < WindowSize + 1
    % 数据不足或无可行解：维持最大探索策略
    gamma = GammaMax;
    p_informed = 0.0;
    pidState.current_gamma = gamma;
    pidState.current_p = p_informed;
    return;
end

% 滑动窗口内的代价改进率
c_old = pidState.c_hist(end - WindowSize);
y_k = max(0, min(1, (c_old - c_best) / (c_old + Epsilon)));

% EMA指数平滑
pidState.ybar = RhoY * pidState.ybar + (1 - RhoY) * y_k;

%% ========== PID误差 ==========
e_k = TargetEfficiency - pidState.ybar;

%% ========== 滤波微分（一阶低通） ==========
pidState.d_filt = RhoD * pidState.d_filt + (1 - RhoD) * (e_k - pidState.e_prev);

%% ========== 积分候选（带饱和限幅） ==========
I_cand = pidState.I + e_k;
I_cand = max(IMin, min(IMax, I_cand));

%% ========== PID输出 ==========
u_cand = Kp * e_k + Ki * I_cand + Kd * pidState.d_filt;

%% ========== 映射到 gamma 和 p_informed ==========
% gamma: 膨胀系数（改进停滞→增大, 改进顺利→减小）
gamma_cand = Gamma0 * exp(AlphaGamma * u_cand);
gamma_cand = max(GammaMin, min(GammaMax, gamma_cand));

% p_informed: 知情采样概率（改进停滞→减小, 改进顺利→增大）
p_cand = P0 - AlphaP * tanh(u_cand);
p_cand = max(PMin, min(PMax, p_cand));

%% ========== 抗积分饱和（Anti-windup） ==========
saturated = false;

if (gamma_cand >= GammaMax - 1e-6 && e_k > 0) || ...
   (gamma_cand <= GammaMin + 1e-6 && e_k < 0)
    saturated = true;
end

if (p_cand <= PMin + 1e-6 && e_k > 0) || ...
   (p_cand >= PMax - 1e-6 && e_k < 0)
    saturated = true;
end

if saturated
    % 冻结积分，使用旧积分值重新计算
    u_k = Kp * e_k + Ki * pidState.I + Kd * pidState.d_filt;
    
    gamma = Gamma0 * exp(AlphaGamma * u_k);
    gamma = max(GammaMin, min(GammaMax, gamma));
    
    p_informed = P0 - AlphaP * tanh(u_k);
    p_informed = max(PMin, min(PMax, p_informed));
else
    % 正常更新积分
    pidState.I = I_cand;
    gamma = gamma_cand;
    p_informed = p_cand;
end

%% ========== 更新状态记录 ==========
pidState.e_prev = e_k;
pidState.current_gamma = gamma;
pidState.current_p = p_informed;
pidState.current_y = pidState.ybar;
pidState.current_error = e_k;

end
