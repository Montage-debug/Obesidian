function [newStartIdx, newStart] = ChooseNodePareto(tree, x_goal, p_non_optimal, currentStartIdx)
% ChooseNodePareto - 基于Pareto dominance选择新的起点 (优化版本)
%
% 优化: 
%   1. 限制候选集大小 (只选最靠近goal的节点)
%   2. 使用简化F_hat估计 (避免O(depth)路径提取)
%   3. 向量化Pareto支配判断

n_nodes = tree.count;

% 限制候选节点数量以避免O(n²)
max_candidates = min(n_nodes, 150);

% 预计算到目标的距离
dists_to_goal = vecnorm(tree.nodes(1:n_nodes,:) - x_goal, 2, 2);

if n_nodes > max_candidates
    % 选取距离目标最近的候选节点
    [~, sorted_idx] = sort(dists_to_goal);
    candidate_indices = sorted_idx(1:max_candidates);
else
    candidate_indices = (1:n_nodes)';
end

n_candidates = length(candidate_indices);
x_start = tree.nodes(1, :);
dist_start_goal = norm(x_goal - x_start);

% 计算Pareto向量: [-O(x), F_hat(x)]
% 使用简化F_hat估计 (不需要路径提取)
pareto_vectors = zeros(n_candidates, 2);

for i = 1:n_candidates
    idx = candidate_indices(i);
    O_x = length(tree.children{idx});
    
    % 简化的F_hat估计: F_hat = G(x) + alpha * H(x)
    cost_x = tree.costs(idx);
    d_goal = dists_to_goal(idx);
    d_start = norm(tree.nodes(idx,:) - x_start);
    
    if d_start > 1e-6
        % alpha = 路径弯曲系数 (cost/直线距离)
        alpha = cost_x / d_start;
    else
        alpha = 1.2;
    end
    
    % F_hat = 已知代价 + 估计剩余代价
    F_hat_x = cost_x + max(d_goal, alpha * d_goal);
    F_hat_x = max(F_hat_x, dist_start_goal);
    
    pareto_vectors(i, 1) = -O_x;
    pareto_vectors(i, 2) = F_hat_x;
end

% 向量化Pareto支配检查
is_pareto_optimal = true(n_candidates, 1);
for i = 1:n_candidates
    if ~is_pareto_optimal(i), continue; end
    % 检查是否存在j支配i
    dominated_by = (pareto_vectors(:,1) <= pareto_vectors(i,1)) & ...
                   (pareto_vectors(:,2) <= pareto_vectors(i,2));
    strictly_better = (pareto_vectors(:,1) < pareto_vectors(i,1)) | ...
                      (pareto_vectors(:,2) < pareto_vectors(i,2));
    dominated_by(i) = false;
    if any(dominated_by & strictly_better)
        is_pareto_optimal(i) = false;
    end
end

pareto_local_indices = find(is_pareto_optimal);
pareto_indices = candidate_indices(pareto_local_indices);

if isempty(pareto_indices)
    [~, best_local] = min(pareto_vectors(:, 2));
    newStartIdx = candidate_indices(best_local);
    newStart = tree.nodes(newStartIdx, :);
    return;
end

if rand < p_non_optimal
    non_pareto_local = find(~is_pareto_optimal);
    if ~isempty(non_pareto_local)
        idx = randi(length(non_pareto_local));
        newStartIdx = candidate_indices(non_pareto_local(idx));
    else
        idx = randi(length(pareto_indices));
        newStartIdx = pareto_indices(idx);
    end
else
    if length(pareto_indices) == 1
        newStartIdx = pareto_indices(1);
    else
        scores = zeros(length(pareto_local_indices), 1);
        max_O = max(-pareto_vectors(:, 1)) + 1e-6;
        max_F = max(pareto_vectors(:, 2)) + 1e-6;
        for i = 1:length(pareto_local_indices)
            li = pareto_local_indices(i);
            O_normalized = -pareto_vectors(li, 1) / max_O;
            F_normalized = 1 - (pareto_vectors(li, 2) / max_F);
            scores(i) = O_normalized * 0.3 + F_normalized * 0.7;
        end
        scores = max(scores, 1e-10);
        probs = scores / sum(scores);
        cumprobs = cumsum(probs);
        r = rand;
        selected = find(cumprobs >= r, 1, 'first');
        newStartIdx = pareto_indices(selected);
    end
end

if newStartIdx == currentStartIdx && n_nodes > 1
    candidates = setdiff(pareto_indices, currentStartIdx);
    if ~isempty(candidates)
        idx = randi(length(candidates));
        newStartIdx = candidates(idx);
    end
end

newStart = tree.nodes(newStartIdx, :);

end
