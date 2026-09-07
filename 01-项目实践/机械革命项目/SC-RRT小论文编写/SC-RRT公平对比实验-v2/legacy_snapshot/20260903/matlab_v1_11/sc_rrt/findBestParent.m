function [best_parent_idx, best_cost] = findBestParent(tree, new_point, nearest_idx, search_radius, obstacles, dim)
% findBestParent - 高性能最优父节点搜索（向量化优化版）

n = size(tree, 1);
best_parent_idx = nearest_idx;
best_cost = tree(nearest_idx, dim+2) + norm(new_point - tree(nearest_idx, 1:dim));

% 向量化距离计算
positions = tree(1:n, 1:dim);
diffs = bsxfun(@minus, positions, new_point);
dists = sqrt(sum(diffs .* diffs, 2));

% 筛选搜索半径内的候选节点
candidate_mask = dists < search_radius;
candidate_indices = find(candidate_mask);

if isempty(candidate_indices)
    return;
end

% 向量化代价计算
candidate_costs = tree(candidate_indices, dim+2) + dists(candidate_indices);

% 按代价排序，只对代价更优的候选做碰撞检测
better_mask = candidate_costs < best_cost;
better_indices = candidate_indices(better_mask);
better_costs = candidate_costs(better_mask);

if isempty(better_indices)
    return;
end

% 按代价从小到大排序
[sorted_costs, sort_order] = sort(better_costs);
sorted_indices = better_indices(sort_order);

% 依次碰撞检测，找到第一个无碰撞的即为最优
for k = 1:length(sorted_indices)
    i = sorted_indices(k);
    if isCollisionFree(tree(i, 1:dim), new_point, obstacles, dim)
        best_cost = sorted_costs(k);
        best_parent_idx = i;
        return;
    end
end

end
