function nearestIdx = Nearest(tree, x_rand)
% Nearest - 找到树中离x_rand最近的节点
%
% 输入:
%   tree   - RRT树结构 (tree.nodes, tree.count)
%   x_rand - 采样点 [1 x m]
%
% 输出:
%   nearestIdx - 最近节点的索引

distances = vecnorm(tree.nodes(1:tree.count,:) - repmat(x_rand, tree.count, 1), 2, 2);
[~, nearestIdx] = min(distances);

end
