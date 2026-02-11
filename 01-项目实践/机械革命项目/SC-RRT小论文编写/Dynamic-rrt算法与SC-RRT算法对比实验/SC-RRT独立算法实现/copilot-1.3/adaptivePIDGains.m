function [Kp, Ki, Kd, stage] = adaptivePIDGains(iterCount, maxIterations)
% adaptivePIDGains - 自适应PID增益调节函数
% 根据搜索进度动态调整PID参数
%
% 理论依据：
%   ??str??m, K. J., & H??gglund, T. (2006). Advanced PID Control.
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

% 基础增益参数（基于Ziegler-Nichols整定法）
Kp_base = 0.15;
Ki_base = 0.02;
Kd_base = 0.08;

% 调节因子
beta_p = 0.5;  % 比例增益调节因子
beta_i = 0.6;  % 积分增益调节因子
beta_d = 0.4;  % 微分增益调节因子

% 计算搜索进度 α ∈ [0, 1]
alpha = iterCount / maxIterations;

% 自适应增益公式
% Kp(α) = Kp_base × (1 + β_p × cos(π × α))
% 余弦函数使初期和后期增大比例响应，中期减小
Kp = Kp_base * (1 + beta_p * cos(pi * alpha));

% Ki(α) = Ki_base × (1 - β_i × α?)
% 抛物线规律递减，避免后期积分饱和，早期积累误差信息
Ki = Ki_base * (1 - beta_i * alpha^2);

% Kd(α) = Kd_base × (1 + β_d × sin(π × α))
% 正弦函数使平衡阶段最大化微分作用，抑制振荡
Kd = Kd_base * (1 + beta_d * sin(pi * alpha));

% 确定搜索阶段
if alpha < 0.3
    stage = 'explore';   % 探索阶段：强探索，弱利用
elseif alpha < 0.7
    stage = 'balance';   % 平衡阶段：探索与利用并重
else
    stage = 'converge';  % 收敛阶段：弱探索，强利用
end

end