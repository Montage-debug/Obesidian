function [xPareto, c_best, c_min, paretoIndices] = ParetoModule(tree, goalPoint, p_nonPareto, m)
% ParetoModule - 高性能Pareto前沿选择模块（优化版）

[n, cols] = size(tree);
if cols < m + 4
    error('ParetoModule: 树结构错误');
end

if nargin < 3, p_nonPareto = 0.1; end

% 构建多维目标向量（向量化）
V = zeros(n, 3);
V(:, 1) = -tree(:, m+4);   % 维度1: 生长度（负值，越多越好）
V(:, 2) = tree(:, m+3);     % 维度2: F_hat总代价

% 维度3: 路径曲折度（向量化计算）
G = tree(:, m+2);
positions = tree(:, 1:m);
startPos = tree(1, 1:m);
diffs = bsxfun(@minus, positions, startPos);
H_direct = sqrt(sum(diffs .* diffs, 2));
H_direct(H_direct < 1e-6) = 1e-6;
lambda_raw = G ./ H_direct;
lambda_raw = max(lambda_raw, 1.0);
V(:, 3) = 1.0 - 1.0 ./ lambda_raw;

% 快速近似Pareto前沿（排序法，O(nlogn)代替O(n²)）
% 按第一目标排序后，逐步筛选非支配解
[~, sortIdx] = sort(V(:, 1));
isDominated = false(n, 1);

% 使用简化策略：按V1排序后，只与之前的最优V2/V3比较
best_V2 = inf;
best_V3 = inf;
for k = 1:n
    i = sortIdx(k);
    if V(i, 2) >= best_V2 && V(i, 3) >= best_V3
        isDominated(i) = true;
    else
        if V(i, 2) < best_V2, best_V2 = V(i, 2); end
        if V(i, 3) < best_V3, best_V3 = V(i, 3); end
    end
end

paretoIndices = find(~isDominated);
if isempty(paretoIndices)
    paretoIndices = (1:n)';  % 如果全被支配，返回所有
end

nonIdx = find(isDominated);

% 选择策略
if rand < p_nonPareto && ~isempty(nonIdx)
    pick = nonIdx(randi(numel(nonIdx)));
else
    pick = paretoIndices(randi(numel(paretoIndices)));
end

xPareto = tree(pick, 1:m);

% 最优F_hat
if ~isempty(paretoIndices)
    [~, minIdx] = min(V(paretoIndices, 2));
    bestNodeIdx = paretoIndices(minIdx);
    c_best = tree(bestNodeIdx, m+3);
else
    c_best = inf;
end

c_min = norm(tree(1, 1:m) - goalPoint(1:m));

end
