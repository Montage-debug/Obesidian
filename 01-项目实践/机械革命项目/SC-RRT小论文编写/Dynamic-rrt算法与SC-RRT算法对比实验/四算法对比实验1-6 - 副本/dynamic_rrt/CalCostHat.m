function F_hat = CalCostHat(x_start, x_goal, x_c, cost_c, tree, c_idx)
% CalCostHat - 估计经过节点c的路径总成本F_hat(c)
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
%   4. F_hat(c) = G(c) + H_hat(c)

m = length(x_c);

% 特殊情况: c就是start
if c_idx == 1 || norm(x_c - x_start) < 1e-6
    F_hat = inf;
    return;
end

% 1. 提取从start到c的路径
path_sc = ExtractPathToNode(tree, c_idx, x_start);

% 路径太短则使用简化估计
if size(path_sc, 1) <= 2
    alpha = 1.2;
    F_hat = cost_c + alpha * norm(x_c - x_goal);
    return;
end

% 2. 计算路径sc的最小包围矩形(MBR)
MBR_min = min(path_sc, [], 1);
MBR_max = max(path_sc, [], 1);
MBR_lengths = MBR_max - MBR_min;
MBR_lengths(MBR_lengths < 1e-3) = 1e-3;
S_sc = prod(MBR_lengths);

% 3. 估计H(c)
dist_start_c = norm(x_c - x_start);
dist_c_goal = norm(x_goal - x_c);
if dist_start_c < 1e-6
    dist_start_c = 1e-6;
end
r = dist_c_goal / dist_start_c;

[max_length, max_dim] = max(MBR_lengths);
estimated_lengths = MBR_lengths;
estimated_lengths(max_dim) = max_length * r;
S_cg_hat = prod(estimated_lengths);

% 4. 将体积转换为路径长度估计
factor = 1.5;
G_c = cost_c;
H_c_hat = (S_cg_hat^(1/m)) * factor;
H_c_alternative = (cost_c / dist_start_c) * dist_c_goal * 1.1;
H_c_hat = max(H_c_hat, H_c_alternative);
H_c_hat = max(H_c_hat, dist_c_goal);

% 5. 最终估计
F_hat = G_c + H_c_hat;
dist_start_goal = norm(x_goal - x_start);
F_hat = max(F_hat, dist_start_goal);

end

function path = ExtractPathToNode(tree, nodeIdx, startPoint)
    path = [];
    currentIdx = nodeIdx;
    while currentIdx ~= 0
        path = [tree.nodes(currentIdx, :); path];
        currentIdx = tree.parents(currentIdx);
    end
end
