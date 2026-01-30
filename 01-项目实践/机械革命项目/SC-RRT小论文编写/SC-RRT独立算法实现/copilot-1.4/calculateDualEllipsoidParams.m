function [c_best_A, c_best_B, c_min_A, c_min_B] = calculateDualEllipsoidParams(treeA, treeB, startPoint, goalPoint, meetPoint, m, buffer)
% calculateDualEllipsoidParams - 计算非对称双椭球体参数
%
% 功能：
%   为双向RRT的两棵树分别计算超椭球约束参数
%
% 输入:
%   treeA      - 起点树 [N×(m+4)]
%   treeB      - 终点树 [M×(m+4)]
%   startPoint - 起始点 [1×m]
%   goalPoint  - 目标点 [1×m]
%   meetPoint  - 交汇点 [1×m]
%   m          - 空间维度
%   buffer     - 缓冲系数 (默认1.2)
%
% 输出:
%   c_best_A - 椭球体A的半长轴 (start→meet)
%   c_best_B - 椭球体B的半长轴 (meet→goal)
%   c_min_A  - 椭球体A的焦距
%   c_min_B  - 椭球体B的焦距  

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

% ========== 计算焦距（椭球的理论最小值） ==========
c_min_A = norm(meetPoint - startPoint);
c_min_B = norm(goalPoint - meetPoint);

% 边界检查：焦距不能为0
if c_min_A < eps
    c_min_A = eps;
    warning('交汇点与起点重合，设置最小焦距');
end
if c_min_B < eps
    c_min_B = eps;
    warning('交汇点与终点重合，设置最小焦距');
end

% ========== 计算超椭球A的半长轴大小 ==========
% 椭球体A: 焦点(start, meetPoint)
% 遍历树A，找到最小节点→交汇点的代价
c_best_A = inf;
nA = size(treeA, 1);

for i = 1:nA
    % G(i): 起点到节点i的实际代价
    G_i = treeA(i, m+2);
    
    % H(i, meet): 节点i到交汇点的启发式代价（欧氏距离）
    H_i_meet = norm(treeA(i, 1:m) - meetPoint);
    
    % F(i) = G(i) + H(i, meet)
    F_i = G_i + H_i_meet;
    
    if F_i < c_best_A
        c_best_A = F_i;
    end
end

% ========== 计算超椭球B的半长轴大小 ==========
% 椭球体B: 焦点(meetPoint, goal)
% 遍历树B，找到最小节点→交汇点的代价
c_best_B = inf;
nB = size(treeB, 1);

for j = 1:nB
    % G(j): 终点到节点j的实际代价
    G_j = treeB(j, m+2);
    
    % H(j, meet): 节点j到交汇点的启发式代价
    H_j_meet = norm(treeB(j, 1:m) - meetPoint);
    
    % F(j) = G(j) + H(j, meet)
    F_j = G_j + H_j_meet;
    
    if F_j < c_best_B
        c_best_B = F_j;
    end
end

% ========== 应用缓冲区增量，防止过小椭球 ==========
% 确保半长轴不会小于理论最小值
% 注意：c_best 必须 >= c_min，因为椭球定义要求 2a >= 2c
% buffer必须确保 c_best >= c_min，即 buffer >= 1.0
c_best_A = max(c_best_A, c_min_A * buffer);
c_best_B = max(c_best_B, c_min_B * buffer);

% ========== 确保双椭球体交集可达 ==========
% 交集条件: c_best_A + c_best_B >= ||start - goal||
c_total_min = norm(goalPoint - startPoint);
if c_best_A + c_best_B < c_total_min * 0.99  % 添加容差避免数值误差
    % 按比例放大以确保交集可达
    scale_factor = (c_total_min * 1.05) / (c_best_A + c_best_B);
    c_best_A = c_best_A * scale_factor;
    c_best_B = c_best_B * scale_factor;
end

% 最终有限性检查
if ~isfinite(c_best_A)
    c_best_A = c_min_A * buffer * 2;
end
if ~isfinite(c_best_B)
    c_best_B = c_min_B * buffer * 2;
end

end
