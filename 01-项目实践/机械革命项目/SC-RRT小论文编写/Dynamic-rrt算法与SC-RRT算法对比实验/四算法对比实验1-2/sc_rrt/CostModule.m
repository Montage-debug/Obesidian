function [costHat, errorInfo] = CostModule(tree, idx, goalPoint, m, varargin)
% CostModule - 统一的代价计算模块 (SC-RRT完全优化版)
%
% 功能：
%    综合所有代价计算特性（包括A*、PID调节、自适应PID、模糊推理等）
%    支持三种代价模式：basic（基础A*）、pid（固定PID）、adaptive（自适应PID+模糊推理）
% 用法：
%    [costHat, ~] = CostModule(tree, idx, goalPoint, m, 'Mode', 'basic')
%    [costHat, info] = CostModule(tree, idx, goalPoint, m, 'Mode', 'pid', ...)
%    [costHat, info] = CostModule(tree, idx, goalPoint, m, 'Mode', 'adaptive', ...)
%
% 输入参数：
%    tree       - 树矩阵，Nx(m+4)矩阵
%    idx        - 当前节点索引
%    goalPoint  - 目标点坐标，1xm
%    m          - 空间维度
%    varargin   - 可选，Name-Value对参数
%       'Mode'              - 代价模式 ('basic'|'pid'|'adaptive')，默认'basic'
%       'PrevError'         - 上次误差 (用于PID)
%       'IntegralError'     - 累积误差 (用于PID)
%       'BestPathLength'    - 当前最优路径长度 (用于PID)
%       'IterCount'         - 当前迭代次数 (用于自适应PID)
%       'MaxIterations'     - 最大迭代次数 (用于自适应PID)
%       'SearchEfficiency'  - 搜索效率 [0,1] (用于自适应PID)
%       'ErrorHistory'      - 历史误差序列 (用于自适应PID)
%
% 输出：
%    costHat   - 调整后总代价
%    errorInfo - 误差信息结构体
%
% 参考文献:
%    Åström, K. J., & Hägglund, T. (2006). Advanced PID Control.
%
% 作者: SC-RRT完全优化版
% 日期: 2025-12-12

% ========== 参数解析 ==========
p = inputParser;
addParameter(p, 'Mode', 'basic', @(x) ismember(x, {'basic', 'pid', 'adaptive'}));
addParameter(p, 'PrevError', 0, @isnumeric);
addParameter(p, 'IntegralError', 0, @isnumeric);
addParameter(p, 'BestPathLength', inf, @isnumeric);
addParameter(p, 'IterCount', 0, @isnumeric);
addParameter(p, 'MaxIterations', 10000, @isnumeric);
addParameter(p, 'SearchEfficiency', 0.5, @isnumeric);
addParameter(p, 'ErrorHistory', [], @isnumeric);
parse(p, varargin{:});

mode = p.Results.Mode;
prevError = p.Results.PrevError;
integralError = p.Results.IntegralError;
bestPathLength = p.Results.BestPathLength;
iterCount = p.Results.IterCount;
maxIterations = p.Results.MaxIterations;
searchEfficiency = p.Results.SearchEfficiency;
errorHistory = p.Results.ErrorHistory;

% ========== 初始化误差结构体 ==========
errorInfo = struct();
errorInfo.mode = mode;
errorInfo.currentError = 0;
errorInfo.integralError = integralError;
errorInfo.derivativeError = 0;
errorInfo.adaptiveGains = [0, 0, 0];
errorInfo.adjustmentFactor = 1.0;
errorInfo.searchStage = 'unknown';
errorInfo.efficiency = searchEfficiency;

% ========== 计算基础A*代价 ==========
costHat = computeBasicCost(tree, idx, goalPoint, m);

% 如果是基础模式或没有最优路径，直接返回
if strcmp(mode, 'basic') || isinf(bestPathLength)
    return;
end

% ========== 根据模式选择计算方法 ==========
switch mode
    case 'pid'
        [costHat, errorInfo] = computePIDCost(costHat, tree, idx, goalPoint, m, ...
            prevError, integralError, bestPathLength, errorInfo);

    case 'adaptive'
        [costHat, errorInfo] = computeAdaptivePIDCost(costHat, tree, idx, goalPoint, m, ...
            prevError, integralError, bestPathLength, iterCount, maxIterations, ...
            searchEfficiency, errorHistory, errorInfo);
end

end

%% ========== 基础A*模块 ==========
function costHat = computeBasicCost(tree, idx, goalPoint, m)
% 标准A*公式：F = G + H

if isempty(tree) || idx <= 0 || idx > size(tree, 1)
    costHat = realmax;
    return;
end

if size(tree, 2) < m + 4
    costHat = realmax;
    return;
end

try
    % G(n): 起点到当前节点的实际代价
    costG = tree(idx, m+2);

    % H(n): 当前到目标点的启发式代价（欧氏距离）
    currentPos = tree(idx, 1:m);
    costH = norm(currentPos - goalPoint);

    costHat = costG + costH;

    % 防止无效值
    if ~isfinite(costHat)
        costHat = realmax;
    end
catch
    costHat = realmax;
end
end

%% ========== 固定PID调节模块 ==========
function [costHatPID, errorInfo] = computePIDCost(costHat, tree, idx, goalPoint, m, ...
    prevError, integralError, bestPathLength, errorInfo)
% 使用固定PID增益的代价调节

% PID增益
Kp = 0.1;
Ki = 0.01;
Kd = 0.05;

% 计算当前位置和误差信息
actualCostToCurrent = tree(idx, m+2);
currentPosition = tree(idx, 1:m);
distanceToGoal = norm(currentPosition - goalPoint);
currentEstimate = actualCostToCurrent + distanceToGoal;

% 当前误差
currentError = abs(bestPathLength - currentEstimate);

% 更新积分误差
newIntegralError = integralError + currentError;

% 计算微分误差
errorDerivative = currentError - prevError;

% PID调节因子
pidFactor = 1 + Kp * currentError + Ki * newIntegralError + Kd * errorDerivative;

% 限幅
pidFactor = max(0.5, min(2.0, pidFactor));

costHatPID = costHat * pidFactor;

% 更新误差信息
errorInfo.currentError = currentError;
errorInfo.integralError = newIntegralError;
errorInfo.derivativeError = errorDerivative;
errorInfo.adaptiveGains = [Kp, Ki, Kd];
errorInfo.adjustmentFactor = pidFactor;

end

%% ========== 自适应PID调节模块 ==========
function [costHatPID, errorInfo] = computeAdaptivePIDCost(costHat, tree, idx, goalPoint, m, ...
    prevError, integralError, bestPathLength, iterCount, maxIterations, ...
    searchEfficiency, errorHistory, errorInfo)
% 基于多阶自适应机制的PID调节

% 1. 计算自适应PID增益
[Kp_base, Ki_base, Kd_base, searchStage] = computeAdaptiveGains(iterCount, maxIterations);
errorInfo.searchStage = searchStage;

% 2. 计算当前误差（绝对和相对性）
actualCostToCurrent = tree(idx, m+2);
currentPosition = tree(idx, 1:m);
distanceToGoal = norm(currentPosition - goalPoint);
currentEstimate = actualCostToCurrent + distanceToGoal;

if bestPathLength > eps
    e_rel = (currentEstimate - bestPathLength) / bestPathLength;
else
    e_rel = 0;
end

e_norm = e_rel / (1 + abs(e_rel));
w_abs = 0.3;
w_rel = 0.7;
currentError = w_abs * abs(e_rel) + w_rel * e_norm;
errorInfo.currentError = currentError;

% 3. 死区积分，防止饱和和泄漏系数
e_threshold = 0.5;
integral_min = -5.0;
integral_max = 5.0;
lambda = 0.95;  % 泄漏因子

if abs(currentError) < e_threshold
    newIntegralError = lambda * integralError + currentError;
else
    newIntegralError = lambda * integralError;
end
newIntegralError = max(integral_min, min(integral_max, newIntegralError));
errorInfo.integralError = newIntegralError;

% 4. 计算微分项
errorDerivative = currentError - prevError;
errorInfo.derivativeError = errorDerivative;

% 5. 模糊推理PID增益
[deltaKp, deltaKi, deltaKd] = computeFuzzyTuning(currentError, errorDerivative, errorHistory);

Kp = Kp_base + deltaKp;
Ki = Ki_base + deltaKi;
Kd = Kd_base + deltaKd;
errorInfo.adaptiveGains = [Kp, Ki, Kd];

% 6. PID控制输出
u_PID = Kp * currentError + Ki * newIntegralError + Kd * errorDerivative;

% 7. 根据搜索效率变化进度影响权重
eta_threshold_high = 0.3;
eta_threshold_low = 0.1;
if searchEfficiency > eta_threshold_high
    weight_factor = 1.0;
elseif searchEfficiency < eta_threshold_low
    weight_factor = 0.7;
else
    weight_factor = 0.85;
end
errorInfo.efficiency = searchEfficiency;

% 8. 最终调整因子
adjustment_factor = 1 + weight_factor * tanh(u_PID);
errorInfo.adjustmentFactor = adjustment_factor;

% 9. 调整后代价
costHatPID = costHat * adjustment_factor;

% 10. 安全限幅
costHatPID = max(0.3 * costHat, min(3.0 * costHat, costHatPID));

end

%% ========== 辅助函数1：自适应增益分段函数 ==========
function [Kp, Ki, Kd, stage] = computeAdaptiveGains(iterCount, maxIterations)
% 根据当前迭代进度计算自适应增益

% 基础增益参数
Kp_base = 0.15;
Ki_base = 0.02;
Kd_base = 0.08;

beta_p = 0.5;
beta_i = 0.6;
beta_d = 0.4;

alpha = iterCount / maxIterations;

Kp = Kp_base * (1 + beta_p * cos(pi * alpha));
Ki = Ki_base * (1 - beta_i * alpha^2);
Kd = Kd_base * (1 + beta_d * sin(pi * alpha));

if alpha < 0.3
    stage = 'explore';
elseif alpha < 0.7
    stage = 'balance';
else
    stage = 'converge';
end
end

%% ========== 辅助函数2：模糊推理调参 ==========
function [deltaKp, deltaKi, deltaKd] = computeFuzzyTuning(error, errorDerivative, errorHistory)
% 使用模糊逻辑微调增益

sigma_e = 0.2;
sigma_d = 0.1;

mu_small_e = exp(-(error / sigma_e)^2);
mu_large_e = 1 - mu_small_e;

mu_stable_ed = exp(-(errorDerivative / sigma_d)^2);
mu_changing_ed = 1 - mu_stable_ed;

deltaKp = mu_large_e * mu_changing_ed * 0.05 - mu_small_e * mu_stable_ed * 0.02;
deltaKi = mu_small_e * mu_stable_ed * 0.005 - mu_large_e * mu_changing_ed * 0.002;

% Kd部分：判断误差序列震荡时增大微分增益
deltaKd = 0;
if ~isempty(errorHistory) && length(errorHistory) >= 3
    signs = sign(errorHistory(end-2:end));
    sign_changes = sum(abs(diff(signs)));
    if sign_changes >= 2
        deltaKd = 0.02;
    end
end

% 限幅
deltaKp = max(-0.05, min(0.05, deltaKp));
deltaKi = max(-0.01, min(0.01, deltaKi));
deltaKd = max(0, min(0.03, deltaKd));
end
