function [newStartIdx, newStart] = ChooseNodePareto(tree, x_goal, p_non_optimal, currentStartIdx)
% ChooseNodePareto - 基于Pareto dominance选择新的起点
%
% Pareto向量定义:
%   v(x) = [-O(x), F_hat(x)]
%   O(x) = out-degree, F_hat(x) = 估计总成本

m = size(tree.nodes, 2);
n_nodes = tree.count;
pareto_vectors = zeros(n_nodes, 2);

for i = 1:n_nodes
    O_x = length(tree.children{i});
    x_node = tree.nodes(i, :);
    cost_node = tree.costs(i);
    F_hat_x = CalCostHat(tree.nodes(1, :), x_goal, x_node, cost_node, tree, i);
    if isinf(F_hat_x)
        F_hat_x = 1e10;
    end
    pareto_vectors(i, 1) = -O_x;
    pareto_vectors(i, 2) = F_hat_x;
end

is_pareto_optimal = true(n_nodes, 1);
for i = 1:n_nodes
    for j = 1:n_nodes
        if i == j, continue; end
        if pareto_vectors(j, 1) <= pareto_vectors(i, 1) && ...
           pareto_vectors(j, 2) <= pareto_vectors(i, 2) && ...
           (pareto_vectors(j, 1) < pareto_vectors(i, 1) || ...
            pareto_vectors(j, 2) < pareto_vectors(i, 2))
            is_pareto_optimal(i) = false;
            break;
        end
    end
end

pareto_indices = find(is_pareto_optimal);

if isempty(pareto_indices)
    [~, newStartIdx] = min(pareto_vectors(:, 2));
    newStart = tree.nodes(newStartIdx, :);
    return;
end

if rand < p_non_optimal
    non_pareto_indices = find(~is_pareto_optimal);
    if ~isempty(non_pareto_indices)
        idx = randi(length(non_pareto_indices));
        newStartIdx = non_pareto_indices(idx);
    else
        idx = randi(length(pareto_indices));
        newStartIdx = pareto_indices(idx);
    end
else
    if length(pareto_indices) == 1
        newStartIdx = pareto_indices(1);
    else
        scores = zeros(length(pareto_indices), 1);
        for i = 1:length(pareto_indices)
            idx = pareto_indices(i);
            O_normalized = -pareto_vectors(idx, 1) / max(-pareto_vectors(:, 1) + 1e-6);
            F_normalized = 1 - (pareto_vectors(idx, 2) / (max(pareto_vectors(:, 2)) + 1e-6));
            scores(i) = O_normalized * 0.3 + F_normalized * 0.7;
        end
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
