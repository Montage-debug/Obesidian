function idx = findNodeIndex(tree, target_point, dim)
% findNodeIndex - 在树中查找指定坐标的节点索引
%
% 输入:
%   tree         - 树矩阵 [N×(dim+4)]
%   target_point - 目标点坐标 [1×dim]
%   dim          - 空间维度
%
% 输出:
%   idx - 节点索引（如果未找到返回0）

n = size(tree, 1);
tolerance = 1e-6;

for i = 1:n
    if norm(tree(i, 1:dim) - target_point) < tolerance
        idx = i;
        return;
    end
end

% 如果未找到精确匹配，返回最近的节点
idx = 1;
min_dist = inf;
for i = 1:n
    dist = norm(tree(i, 1:dim) - target_point);
    if dist < min_dist
        min_dist = dist;
        idx = i;
    end
end

end
