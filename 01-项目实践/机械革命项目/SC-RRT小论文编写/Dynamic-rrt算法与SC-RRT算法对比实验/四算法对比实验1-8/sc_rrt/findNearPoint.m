function [nearestIdx, nearestPoint] = findNearPoint(tree, point)
% findNearPoint - 在树中查找最近节点
%
% 输入:
%   tree  - 树矩阵 [N×(m+4)]
%   point - 查询点 [1×m]
%
% 输出:
%   nearestIdx   - 最近节点索引
%   nearestPoint - 最近节点坐标 [1×m]

if isempty(tree)
    nearestIdx = 0;
    nearestPoint = [];
    return;
end

% 获取维度
[n, cols] = size(tree);
if cols < 4
    error('findNearPoint: 树结构不正确');
end

m = cols - 4;

% 确保point是行向量
if size(point, 1) > size(point, 2)
    point = point';
end

% 计算所有节点到查询点的距离
distances = sqrt(sum((tree(:, 1:m) - repmat(point, n, 1)).^2, 2));

% 找到最小距离的索引
[~, nearestIdx] = min(distances);

% 返回最近节点坐标
nearestPoint = tree(nearestIdx, 1:m);

end
