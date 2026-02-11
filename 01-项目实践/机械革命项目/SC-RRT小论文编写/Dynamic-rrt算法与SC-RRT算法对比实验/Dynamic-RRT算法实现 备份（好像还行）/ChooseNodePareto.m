function [newStartIdx, newStart] = ChooseNodePareto(tree, x_goal, p_non_optimal, currentStartIdx)
% ChooseNodePareto - 基于Pareto dominance选择新的起点
%
% 论文依据: Section 3.2 - Dynamic Programming
%   使用Pareto dominance对树节点排序，选择"更值得作为中间起点"的节点
%
% Pareto向量定义:
%   v(x) = [-O(x), F̂(x)]
%   其中:
%     O(x) = out-degree (子节点数，越大越好，所以取负)
%     F̂(x) = 估计总成本 (越小越好)
%
% 选择策略:
%   1. 计算所有节点的Pareto支配关系
%   2. 找到Pareto-optimal节点集
%   3. 以概率p随机选择非最优节点（避免局部最优）
%   4. 否则从Pareto-optimal集中随机选一个
%
% 输入:
%   tree            - RRT树结构
%   x_goal          - 目标点
%   p_non_optimal   - 选择非Pareto节点的概率
%   currentStartIdx - 当前起点索引
%
% 输出:
%   newStartIdx - 新起点在树中的索引
%   newStart    - 新起点位置

m = size(tree.nodes, 2);

% 1. 为每个节点计算Pareto向量
n_nodes = tree.count;
pareto_vectors = zeros(n_nodes, 2);

for i = 1:n_nodes
    % O(x): out-degree（子节点数）
    O_x = length(tree.children{i});
    
    % F̂(x): 估计总成本
    x_node = tree.nodes(i, :);
    cost_node = tree.costs(i);
    
    % 使用CalCostHat估计（从当前节点到goal）
    % 注意：这里的"start"是原始起点，不是currentStart
    F_hat_x = CalCostHat(tree.nodes(1, :), x_goal, x_node, cost_node, tree, i);
    
    % 如果F̂(x)是无穷大（节点是起点），用一个大值代替
    if isinf(F_hat_x)
        F_hat_x = 1e10;
    end
    
    % Pareto向量: [-O(x), F̂(x)]
    pareto_vectors(i, 1) = -O_x;  % 取负，因为越大越好
    pareto_vectors(i, 2) = F_hat_x;
end

% 2. 计算Pareto dominance关系
% 节点i支配节点j，如果：
%   pareto_vectors(i, 1) <= pareto_vectors(j, 1) AND
%   pareto_vectors(i, 2) <= pareto_vectors(j, 2) AND
%   至少有一个是严格小于

is_pareto_optimal = true(n_nodes, 1);

for i = 1:n_nodes
    for j = 1:n_nodes
        if i == j
            continue;
        end
        
        % 检查j是否支配i
        if pareto_vectors(j, 1) <= pareto_vectors(i, 1) && ...
           pareto_vectors(j, 2) <= pareto_vectors(i, 2) && ...
           (pareto_vectors(j, 1) < pareto_vectors(i, 1) || ...
            pareto_vectors(j, 2) < pareto_vectors(i, 2))
            % j支配i，所以i不是Pareto-optimal
            is_pareto_optimal(i) = false;
            break;
        end
    end
end

% 找到Pareto-optimal节点集
pareto_indices = find(is_pareto_optimal);

% 如果没有Pareto-optimal节点（理论上不应该），选择F̂最小的
if isempty(pareto_indices)
    [~, newStartIdx] = min(pareto_vectors(:, 2));
    newStart = tree.nodes(newStartIdx, :);
    return;
end

% 3. 以概率p选择非Pareto节点（避免局部最优）
if rand < p_non_optimal
    % 从非Pareto-optimal节点中选择
    non_pareto_indices = find(~is_pareto_optimal);
    
    if ~isempty(non_pareto_indices)
        % 随机选一个非Pareto节点
        idx = randi(length(non_pareto_indices));
        newStartIdx = non_pareto_indices(idx);
    else
        % 如果所有节点都是Pareto-optimal，从中随机选
        idx = randi(length(pareto_indices));
        newStartIdx = pareto_indices(idx);
    end
else
    % 4. 从Pareto-optimal集中选择
    % 论文：如果有多个，随机选一个
    
    % 为了更好的效果，可以按某种启发式加权选择
    % 这里使用简单的随机选择
    
    if length(pareto_indices) == 1
        newStartIdx = pareto_indices(1);
    else
        % 计算每个Pareto节点的"质量"（F̂越小越好，O越大越好）
        scores = zeros(length(pareto_indices), 1);
        for i = 1:length(pareto_indices)
            idx = pareto_indices(i);
            % 综合分数：归一化的O和F̂
            O_normalized = -pareto_vectors(idx, 1) / max(-pareto_vectors(:, 1) + 1e-6);
            F_normalized = 1 - (pareto_vectors(idx, 2) / (max(pareto_vectors(:, 2)) + 1e-6));
            scores(i) = O_normalized * 0.3 + F_normalized * 0.7;  % F̂权重更高
        end
        
        % 按分数加权随机选择
        probs = scores / sum(scores);
        cumprobs = cumsum(probs);
        r = rand;
        selected = find(cumprobs >= r, 1, 'first');
        newStartIdx = pareto_indices(selected);
    end
end

% 确保不选当前起点（除非没有其他选择）
if newStartIdx == currentStartIdx && n_nodes > 1
    % 尝试选择另一个
    candidates = setdiff(pareto_indices, currentStartIdx);
    if ~isempty(candidates)
        idx = randi(length(candidates));
        newStartIdx = candidates(idx);
    end
end

newStart = tree.nodes(newStartIdx, :);

end
