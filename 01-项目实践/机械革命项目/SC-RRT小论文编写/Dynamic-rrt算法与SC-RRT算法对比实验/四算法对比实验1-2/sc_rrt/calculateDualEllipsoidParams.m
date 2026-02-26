function [c_best_A, c_best_B, c_min_A, c_min_B] = calculateDualEllipsoidParams(treeA, treeB, startPoint, goalPoint, meetPoint, m, buffer)
% calculateDualEllipsoidParams - 计算非对称双椭球体参数（PID优化版）
%
% 核心改进：使用PID优化后的F_hat代价预估作为椭球体约束
%
% 输入:
%   treeA      - 起点树 [N×(m+4)]，列格式 [pos | parent | G | F_hat | children]
%   treeB      - 终点树 [M×(m+4)]
%   startPoint - 起始点 [1×m]
%   goalPoint  - 目标点 [1×m]
%   meetPoint  - 交汇点 [1×m]
%   m          - 空间维度
%   buffer     - 缓冲系数 (默认1.2)
%
% 输出:
%   c_best_A - 椭球体A的半长轴（基于PID优化F_hat）
%   c_best_B - 椭球体B的半长轴（基于PID优化F_hat）
%   c_min_A  - 椭球体A的焦距（几何下界）
%   c_min_B  - 椭球体B的焦距（几何下界）

if nargin < 7
    buffer = 1.2;
end

% 输入验证
if isempty(treeA) || isempty(treeB)
    c_best_A = inf;
    c_best_B = inf;
    c_min_A = norm(meetPoint - startPoint);
    c_min_B = norm(goalPoint - meetPoint);
    return;
end

% ========== 计算几何最小距离（下界） ==========
% 树A：从起点到交汇点的直线距离
c_min_A = norm(meetPoint - startPoint);

% 树B：从交汇点到终点的直线距离
c_min_B = norm(goalPoint - meetPoint);

% 边界检查：焦距不能为0
if c_min_A < eps
    c_min_A = eps;
end
if c_min_B < eps
    c_min_B = eps;
end

% ========== 计算树A的最优椭球约束 ==========
sizeA = size(treeA, 1);
if sizeA > 1
    % 策略：使用PID优化后的F_hat代价预估（列m+3）
    % F_hat = F_basic * (1 + w_η * tanh(u_PID))
    % 这比简单的G+H更准确，因为考虑了搜索历史和趋势
    
    % 计算每个节点到交汇点的总代价预估
    costs_to_meet = zeros(sizeA, 1);
    for i = 1:sizeA
        F_hat_i = treeA(i, m+3);  % PID优化后的代价预估
        H_to_meet = norm(treeA(i, 1:m) - meetPoint);  % 到交汇点的启发式
        costs_to_meet(i) = F_hat_i + H_to_meet;
    end
    
    % 选择最小代价作为c_best_A
    c_best_A = min(costs_to_meet);
    
    % 应用缓冲系数（增加采样空间）
    c_best_A = c_best_A * buffer;
    
    % 确保c_best_A >= c_min_A（椭球定义要求）
    c_best_A = max(c_best_A, c_min_A * 1.1);
else
    c_best_A = inf;  % 树为空时无限制
end

% ========== 计算树B的最优椭球约束 ==========
sizeB = size(treeB, 1);
if sizeB > 1
    % 策略：同树A，使用PID优化F_hat代价
    costs_to_meet = zeros(sizeB, 1);
    for i = 1:sizeB
        F_hat_i = treeB(i, m+3);  % PID优化后的代价预估
        H_to_meet = norm(treeB(i, 1:m) - meetPoint);  % 到交汇点的启发式
        costs_to_meet(i) = F_hat_i + H_to_meet;
    end
    
    % 选择最小代价
    c_best_B = min(costs_to_meet);
    
    % 应用缓冲系数
    c_best_B = c_best_B * buffer;
    
    % 确保c_best_B >= c_min_B
    c_best_B = max(c_best_B, c_min_B * 1.1);
else
    c_best_B = inf;  % 树为空时无限制
end

% ========== 确保双椭球体交集可达 ==========
% 交集条件: c_best_A + c_best_B >= ||start - goal||
c_total_min = norm(goalPoint - startPoint);
if isfinite(c_best_A) && isfinite(c_best_B)
    if c_best_A + c_best_B < c_total_min * 0.99  % 添加容差避免数值误差
        % 按比例放大以确保交集可达
        scale_factor = (c_total_min * 1.05) / (c_best_A + c_best_B);
        c_best_A = c_best_A * scale_factor;
        c_best_B = c_best_B * scale_factor;
    end
end

% 最终有限性检查
if ~isfinite(c_best_A)
    c_best_A = c_min_A * buffer * 2;
end
if ~isfinite(c_best_B)
    c_best_B = c_min_B * buffer * 2;
end

end
