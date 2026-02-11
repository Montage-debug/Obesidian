function nearestIdx = Nearest(tree, x_rand)
% Nearest - 找到树中离x_rand最近的节点
%
% 输入:
%   tree   - RRT树结构
%   x_rand - 采样点 [1×m]
%
% 输出:
%   nearestIdx - 最近节点的索引

% 计算所有节点到x_rand的距离
distances = vecnorm(tree.nodes - repmat(x_rand, tree.count, 1), 2, 2);

% 找到最小距离的索引
[~, nearestIdx] = min(distances);

end
