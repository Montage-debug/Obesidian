function [nearest_idx, nearest_point] = findNearestInTree(tree, sample, m)
% findNearestInTree - 在树中查找距离采样点最近的节点
%
% 输入:
%   tree   - 树矩阵 [N×(m+4)]
%   sample - 采样点 [1×m]
%   m      - 空间维度
%
% 输出:
%   nearest_idx   - 最近节点索引
%   nearest_point - 最近节点坐标 [1×m]
%
% 作者: SC-RRT完全优化版
% 日期: 2025-12-12

if isempty(tree)
    nearest_idx = 0;
    nearest_point = [];
    return;
end

% 计算所有节点到采样点的距离
positions = tree(:, 1:m);
distances = vecnorm(positions - sample, 2, 2);

% 找到最小距离的节点
[~, nearest_idx] = min(distances);
nearest_point = tree(nearest_idx, 1:m);

end
