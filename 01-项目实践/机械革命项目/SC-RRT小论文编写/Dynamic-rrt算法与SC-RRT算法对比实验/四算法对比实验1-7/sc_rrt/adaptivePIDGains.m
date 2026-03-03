function [Kp, Ki, Kd, stage] = adaptivePIDGains(iterCount, maxIterations)
% adaptivePIDGains - 自适应PID增益调度函数
% 按迭代进度动态调整PID增益
%
% 参考文献：
%   Åström, K. J., & Hägglund, T. (2006). Advanced PID Control.
%   章节3.4: "Gain Scheduling and Adaptive Control"
%
% 输入:
%   iterCount      - 当前迭代次数
%   maxIterations  - 最大迭代次数
% 输出:
%   Kp    - 自适应比例增益
%   Ki    - 自适应积分增益
%   Kd    - 自适应微分增益
%   stage - 当前搜索阶段 ('explore', 'balance', 'converge')

% 基础增益（基于Ziegler-Nichols整定）
Kp_base = 0.15;
Ki_base = 0.02;
Kd_base = 0.08;

% 调制系数
beta_p = 0.5;  % 比例项振幅系数
beta_i = 0.6;  % 积分项衰减系数
beta_d = 0.4;  % 微分项振幅系数

% 归一化进度 α ∈ [0, 1]
alpha = iterCount / maxIterations;

% 自适应调度公式
% Kp(α) = Kp_base × (1 + β_p × cos(π × α))
% 余弦函数使得初始和后期较大，中期较小
Kp = Kp_base * (1 + beta_p * cos(pi * alpha));

% Ki(α) = Ki_base × (1 - β_i × α²)
% 二次衰减函数，后期积分项逐渐减小，避免过度累积
Ki = Ki_base * (1 - beta_i * alpha^2);

% Kd(α) = Kd_base × (1 + β_d × sin(π × α))
% 正弦函数使平滑阶段微分项增大，增强稳定性
Kd = Kd_base * (1 + beta_d * sin(pi * alpha));

% 确定搜索阶段
if alpha < 0.3
    stage = 'explore';   % 探索阶段：强探索，弱收敛
elseif alpha < 0.7
    stage = 'balance';   % 平衡阶段：探索与收敛并重
else
    stage = 'converge';  % 收敛阶段：弱探索，强收敛
end

end
