function [pidState, gamma, p_informed] = PIDSamplingController(pidState, c_best, varargin)
% PIDSamplingController - PID控制知情采样子集的动态绘制
%
% 核心思想：用PID控制器动态调节"椭球体膨胀系数gamma"和"知情采样概率p_informed"，
%           而不是调节节点代价。被控量是"近期路径改进效率"。
%
% 输入：
%   pidState    - PID状态结构体（首次调用传入[]自动初始化）
%   c_best      - 当前最优路径代价（Inf表示尚无可行解）
%   varargin    - 可选参数：
%       'Initialize'     - 是否初始化（默认false）
%       'WindowSize'     - 滑动窗口大小（默认50）
%       'TargetEfficiency' - 目标改进效率（默认0.02）
%       'Kp', 'Ki', 'Kd' - PID增益（默认2.0, 0.2, 0.8）
%       'Gamma0'         - 初始膨胀系数（默认1.5）
%       'GammaMin'       - 最小膨胀系数（默认1.0）
%       'GammaMax'       - 最大膨胀系数（默认4.0）
%       'P0'             - 初始知情采样概率（默认0.8）
%       'PMin'           - 最小知情采样概率（默认0.2）
%       'PMax'           - 最大知情采样概率（默认0.95）
%
% 输出：
%   pidState    - 更新后的PID状态
%   gamma       - 当前椭球体膨胀系数
%   p_informed  - 当前知情采样概率
%
% 原理说明：
%   1. 测量量y_k：窗口内的路径代价改进率（平滑后）
%   2. 误差e_k：目标效率 - 实际效率
%   3. PID输出u_k控制gamma和p的变化：
%      - 若改进缓慢（e_k>0）→增大gamma、减小p（增强探索）
%      - 若改进顺利（e_k<0）→减小gamma、增大p（加强开发）
%
% 参考文献：
%   基于"PID控制知情子集体积/采样概率的闭环调度"方案

%% ========== 参数解析 ==========
p = inputParser;
addParameter(p, 'Initialize', false, @islogical);
addParameter(p, 'WindowSize', 50, @isnumeric);
addParameter(p, 'TargetEfficiency', 0.02, @isnumeric);
addParameter(p, 'Kp', 2.0, @isnumeric);
addParameter(p, 'Ki', 0.2, @isnumeric);
addParameter(p, 'Kd', 0.8, @isnumeric);
addParameter(p, 'IMin', -3.0, @isnumeric);
addParameter(p, 'IMax', 3.0, @isnumeric);
addParameter(p, 'RhoY', 0.9, @isnumeric);     % y平滑系数
addParameter(p, 'RhoD', 0.8, @isnumeric);     % d滤波系数
addParameter(p, 'Gamma0', 1.5, @isnumeric);
addParameter(p, 'GammaMin', 1.0, @isnumeric);
addParameter(p, 'GammaMax', 4.0, @isnumeric);
addParameter(p, 'AlphaGamma', 0.5, @isnumeric);
addParameter(p, 'P0', 0.8, @isnumeric);
addParameter(p, 'PMin', 0.2, @isnumeric);
addParameter(p, 'PMax', 0.95, @isnumeric);
addParameter(p, 'AlphaP', 0.5, @isnumeric);
addParameter(p, 'Epsilon', 1e-6, @isnumeric); % 防除零
parse(p, varargin{:});

params = p.Results;

%% ========== 初始化状态 ==========
if params.Initialize || isempty(pidState)
    pidState = struct();
    pidState.c_hist = [];           % 代价历史
    pidState.e_prev = 0;            % 上次误差
    pidState.I = 0;                 % 积分项
    pidState.d_filt = 0;            % 滤波微分项
    pidState.ybar = 0;              % 平滑后的改进率
    pidState.iter_count = 0;        % 迭代计数
    pidState.params = params;       % 保存参数
    
    % 初始值（无解时使用）
    gamma = params.GammaMax;
    p_informed = 0.0;  % 无解时不用知情采样
    return;
end

% 更新参数（允许运行时修改）
pidState.params = params;

%% ========== 更新代价历史 ==========
pidState.iter_count = pidState.iter_count + 1;
pidState.c_hist = [pidState.c_hist, c_best];

W = params.WindowSize;
eps_val = params.Epsilon;

%% ========== 计算改进效率（测量量y_k） ==========
if isinf(c_best) || length(pidState.c_hist) < W + 1
    % 尚无解或数据不足：使用最大探索策略
    gamma = params.GammaMax;
    p_informed = 0.0;
    return;
end

% 窗口起点和当前的代价
c_old = pidState.c_hist(end - W);
c_current = c_best;

% 计算改进率（有界、平滑）
y_k = max(0, min(1, (c_old - c_current) / (c_old + eps_val)));

% EMA平滑
rho_y = params.RhoY;
pidState.ybar = rho_y * pidState.ybar + (1 - rho_y) * y_k;

%% ========== PID误差计算 ==========
y_star = params.TargetEfficiency;
e_k = y_star - pidState.ybar;

%% ========== 滤波微分 ==========
rho_d = params.RhoD;
pidState.d_filt = rho_d * pidState.d_filt + (1 - rho_d) * (e_k - pidState.e_prev);

%% ========== 积分候选（带饱和限制） ==========
I_cand = pidState.I + e_k;
I_cand = max(params.IMin, min(params.IMax, I_cand));

%% ========== PID输出（使用候选积分） ==========
Kp = params.Kp;
Ki = params.Ki;
Kd = params.Kd;

u_cand = Kp * e_k + Ki * I_cand + Kd * pidState.d_filt;

%% ========== 映射到gamma和p ==========
% gamma：膨胀系数（探索越多越大）
gamma_cand = params.Gamma0 * exp(params.AlphaGamma * u_cand);
gamma_cand = max(params.GammaMin, min(params.GammaMax, gamma_cand));

% p_informed：知情采样概率（探索越多越小）
p_cand = params.P0 - params.AlphaP * tanh(u_cand);
p_cand = max(params.PMin, min(params.PMax, p_cand));

%% ========== 抗积分饱和（Anti-windup） ==========
% 判断是否饱和：如果gamma或p达到边界，且误差继续推向边界，则冻结积分
saturated = false;

% gamma饱和检测
if (gamma_cand >= params.GammaMax - 1e-6 && e_k > 0) || ...
   (gamma_cand <= params.GammaMin + 1e-6 && e_k < 0)
    saturated = true;
end

% p饱和检测
if (p_cand <= params.PMin + 1e-6 && e_k > 0) || ...
   (p_cand >= params.PMax - 1e-6 && e_k < 0)
    saturated = true;
end

if saturated
    % 冻结积分，重新计算u
    u_k = Kp * e_k + Ki * pidState.I + Kd * pidState.d_filt;
    
    gamma = params.Gamma0 * exp(params.AlphaGamma * u_k);
    gamma = max(params.GammaMin, min(params.GammaMax, gamma));
    
    p_informed = params.P0 - params.AlphaP * tanh(u_k);
    p_informed = max(params.PMin, min(params.PMax, p_informed));
else
    % 更新积分
    pidState.I = I_cand;
    gamma = gamma_cand;
    p_informed = p_cand;
end

%% ========== 更新状态 ==========
pidState.e_prev = e_k;
pidState.current_gamma = gamma;
pidState.current_p = p_informed;
pidState.current_u = u_k;
pidState.current_y = pidState.ybar;
pidState.current_error = e_k;

end
