function F_hat = CalCostHat(x_start, x_goal, x_c, cost_c, tree, c_idx)
% CalCostHat - 估计经过节点c的路径总成本F̂(c)
%
% 论文核心思想:
%   F(c) = G(c) + H(c)
%   其中 G(c) = cost from start to c (已知)
%        H(c) = estimated cost from c to goal (需要估计)
%
% 估计策略 (基于论文Section 3.1):
%   1. 提取从start到c的路径片段sc
%   2. 计算sc的最小包围矩形(MBR)体积
%   3. 使用距离比例和MBR特征估计H(c)
%   4. F̂(c) = G(c) + Ĥ(c)
%
% 输入:
%   x_start - 当前起点位置
%   x_goal  - 目标点位置
%   x_c     - 当前节点c的位置
%   cost_c  - 从start到c的累计cost (G(c))
%   tree    - RRT树结构
%   c_idx   - 节点c在树中的索引
%
% 输出:
%   F_hat   - 估计的总路径成本

m = length(x_c);  % 维度

% 特殊情况：c就是start，返回无穷大（论文约定）
if c_idx == 1 || norm(x_c - x_start) < 1e-6
    F_hat = inf;
    return;
end

% 1. 提取从start到c的路径
path_sc = ExtractPathToNode(tree, c_idx, x_start);

% 如果路径太短（只有start和c），使用简化估计
if size(path_sc, 1) <= 2
    % 简化估计: F̂(c) = G(c) + α * ||c - goal||
    % α取决于障碍物密度，这里使用经验值
    alpha = 1.2;  % 略大于1以考虑绕障
    F_hat = cost_c + alpha * norm(x_c - x_goal);
    return;
end

% 2. 计算路径sc的最小包围矩形(MBR)
MBR_min = min(path_sc, [], 1);  % 每个维度的最小值
MBR_max = max(path_sc, [], 1);  % 每个维度的最大值
MBR_lengths = MBR_max - MBR_min;  % 每个维度的边长

% 避免零边长（路径在某维度上是直线）
MBR_lengths(MBR_lengths < 1e-3) = 1e-3;

% MBR体积
S_sc = prod(MBR_lengths);

% 3. 估计H(c) - 从c到goal的成本
% 论文方法：使用距离比例和MBR缩放
dist_start_c = norm(x_c - x_start);
dist_c_goal = norm(x_goal - x_c);

% 避免除零
if dist_start_c < 1e-6
    dist_start_c = 1e-6;
end

% 距离比例
r = dist_c_goal / dist_start_c;

% 估计从c到goal的MBR体积 (论文: 重复与缩放方法)
% 这里使用简化实现：按距离比例缩放主要维度
% 找到MBR中最长的边（主方向）
[max_length, max_dim] = max(MBR_lengths);

% 估计到goal的MBR长度（在主维度上按距离比例缩放）
estimated_lengths = MBR_lengths;
estimated_lengths(max_dim) = max_length * r;

% 估计到goal的MBR体积
S_cg_hat = prod(estimated_lengths);

% 4. 将体积转换为路径长度估计
% 使用启发式: cost ≈ volume^(1/m) * factor
% factor考虑路径不是直线以及障碍物影响
factor = 1.5;  % 经验参数

% G(c)的估计（已知实际值）
G_c = cost_c;

% H(c)的估计
H_c_hat = (S_cg_hat^(1/m)) * factor;

% 另一种估计方式：基于实际G(c)和距离比例
% 这种方法更稳定
H_c_alternative = (cost_c / dist_start_c) * dist_c_goal * 1.1;

% 综合两种估计（取较大值以保证informed subset不会过小）
H_c_hat = max(H_c_hat, H_c_alternative);

% 但不应小于直线距离
H_c_hat = max(H_c_hat, dist_c_goal);

% 5. 最终估计
F_hat = G_c + H_c_hat;

% 论文约束：F̂(c)不应小于start到goal的直线距离
dist_start_goal = norm(x_goal - x_start);
F_hat = max(F_hat, dist_start_goal);

end

function path = ExtractPathToNode(tree, nodeIdx, startPoint)
    % 从nodeIdx回溯到start提取路径
    path = [];
    currentIdx = nodeIdx;
    
    while currentIdx ~= 0
        path = [tree.nodes(currentIdx, :); path];
        currentIdx = tree.parents(currentIdx);
    end
end
