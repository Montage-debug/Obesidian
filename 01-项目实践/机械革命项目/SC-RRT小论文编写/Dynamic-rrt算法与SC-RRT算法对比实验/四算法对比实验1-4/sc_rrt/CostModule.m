function [costHat, errorInfo] = CostModule(tree, idx, goalPoint, m, varargin)
% CostModule - 高性能代价计算模块 (SC-RRT速度优化版)
%
% 功能：综合代价计算（基础A*、PID调节、自适应PID+模糊推理）
%       移除inputParser，使用手动解析以提升热路径性能
%
% 输入参数：
%    tree       - 树矩阵，Nx(m+4)
%    idx        - 当前节点索引
%    goalPoint  - 目标点坐标，1xm
%    m          - 空间维度
%    varargin   - Name-Value对参数（手动解析）

% ========== 快速参数解析（替代inputParser） ==========
mode = 'basic';
prevError = 0;
integralError = 0;
bestPathLength = inf;
iterCount = 0;
maxIterations = 10000;
searchEfficiency = 0.5;
errorHistory = [];

nargs = length(varargin);
i = 1;
while i <= nargs
    if ischar(varargin{i})
        switch varargin{i}
            case 'Mode',             mode = varargin{i+1}; i = i + 2;
            case 'PrevError',        prevError = varargin{i+1}; i = i + 2;
            case 'IntegralError',    integralError = varargin{i+1}; i = i + 2;
            case 'BestPathLength',   bestPathLength = varargin{i+1}; i = i + 2;
            case 'IterCount',        iterCount = varargin{i+1}; i = i + 2;
            case 'MaxIterations',    maxIterations = varargin{i+1}; i = i + 2;
            case 'SearchEfficiency', searchEfficiency = varargin{i+1}; i = i + 2;
            case 'ErrorHistory',     errorHistory = varargin{i+1}; i = i + 2;
            otherwise, i = i + 2;
        end
    else
        i = i + 1;
    end
end

% ========== 初始化误差结构体 ==========
errorInfo.mode = mode;
errorInfo.currentError = 0;
errorInfo.integralError = integralError;
errorInfo.derivativeError = 0;
errorInfo.adaptiveGains = [0, 0, 0];
errorInfo.adjustmentFactor = 1.0;
errorInfo.searchStage = 'unknown';
errorInfo.efficiency = searchEfficiency;

% ========== 计算基础A*代价（内联） ==========
if idx <= 0 || idx > size(tree, 1) || size(tree, 2) < m + 4
    costHat = realmax;
    return;
end

costG = tree(idx, m+2);
currentPos = tree(idx, 1:m);
costH = norm(currentPos - goalPoint);
costHat = costG + costH;

if ~isfinite(costHat)
    costHat = realmax;
    return;
end

% 基础模式或无最优路径，直接返回
if mode(1) == 'b' || isinf(bestPathLength)
    return;
end

% ========== PID/Adaptive 代价调节 ==========
currentEstimate = costHat;

if mode(1) == 'p'
    % 固定PID
    currentError = abs(bestPathLength - currentEstimate);
    newIntegralError = integralError + currentError;
    errorDerivative = currentError - prevError;
    
    pidFactor = 1 + 0.1 * currentError + 0.01 * newIntegralError + 0.05 * errorDerivative;
    pidFactor = max(0.5, min(2.0, pidFactor));
    
    costHat = costHat * pidFactor;
    
    errorInfo.currentError = currentError;
    errorInfo.integralError = newIntegralError;
    errorInfo.derivativeError = errorDerivative;
    errorInfo.adaptiveGains = [0.1, 0.01, 0.05];
    errorInfo.adjustmentFactor = pidFactor;
else
    % 自适应PID（简化版，保留核心逻辑）
    alpha = iterCount / maxIterations;
    
    % 自适应增益（内联计算）
    Kp = 0.15 * (1 + 0.5 * cos(pi * alpha));
    Ki = 0.02 * (1 - 0.6 * alpha * alpha);
    Kd = 0.08 * (1 + 0.4 * sin(pi * alpha));
    
    % 误差计算
    if bestPathLength > eps
        e_rel = (currentEstimate - bestPathLength) / bestPathLength;
    else
        e_rel = 0;
    end
    e_norm = e_rel / (1 + abs(e_rel));
    currentError = 0.3 * abs(e_rel) + 0.7 * e_norm;
    
    % 积分（带泄漏和死区）
    if abs(currentError) < 0.5
        newIntegralError = 0.95 * integralError + currentError;
    else
        newIntegralError = 0.95 * integralError;
    end
    newIntegralError = max(-5.0, min(5.0, newIntegralError));
    
    % 微分
    errorDerivative = currentError - prevError;
    
    % 简化模糊推理（内联）
    mu_small = exp(-(currentError / 0.2)^2);
    mu_large = 1 - mu_small;
    mu_stable = exp(-(errorDerivative / 0.1)^2);
    mu_changing = 1 - mu_stable;
    
    Kp = Kp + mu_large * mu_changing * 0.05 - mu_small * mu_stable * 0.02;
    Ki = Ki + mu_small * mu_stable * 0.005 - mu_large * mu_changing * 0.002;
    
    % PID输出
    u_PID = Kp * currentError + Ki * newIntegralError + Kd * errorDerivative;
    
    % 搜索效率权重
    if searchEfficiency > 0.3
        wf = 1.0;
    elseif searchEfficiency < 0.1
        wf = 0.7;
    else
        wf = 0.85;
    end
    
    adjustment = 1 + wf * tanh(u_PID);
    costHat = costHat * adjustment;
    costHat = max(0.3 * (costG + costH), min(3.0 * (costG + costH), costHat));
    
    errorInfo.currentError = currentError;
    errorInfo.integralError = newIntegralError;
    errorInfo.derivativeError = errorDerivative;
    errorInfo.adaptiveGains = [Kp, Ki, Kd];
    errorInfo.adjustmentFactor = adjustment;
    
    if alpha < 0.3
        errorInfo.searchStage = 'explore';
    elseif alpha < 0.7
        errorInfo.searchStage = 'balance';
    else
        errorInfo.searchStage = 'converge';
    end
end

end
