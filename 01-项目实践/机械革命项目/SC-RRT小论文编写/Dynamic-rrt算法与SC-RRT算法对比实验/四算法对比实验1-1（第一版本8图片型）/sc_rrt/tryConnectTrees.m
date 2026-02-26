function [success, connect_idx] = tryConnectTrees(point, tree, obstacles, dim, threshold)
% tryConnectTrees - 尝试将一个点连接到树
%
% 输入:
%   point      - 待连接的点 [1×dim]
%   tree       - 目标树 [N×(dim+4)]
%   obstacles  - 障碍物结构体
%   dim        - 空间维度
%   threshold  - 连接阈值距离
%
% 输出:
%   success     - 是否连接成功
%   connect_idx - 连接到的节点索引

success = false;
connect_idx = 0;
n = size(tree, 1);

if n == 0
    return;
end

% 找到树中最近的节点
min_dist = inf;
nearest_idx = 1;

for i = 1:n
    dist = norm(tree(i, 1:dim) - point);
    if dist < min_dist
        min_dist = dist;
        nearest_idx = i;
    end
end

% 如果距离足够近且无碰撞,则连接成功
if min_dist < threshold
    nearest_point = tree(nearest_idx, 1:dim);
    if isCollisionFree(nearest_point, point, obstacles, dim)
        success = true;
        connect_idx = nearest_idx;
    end
end

end
