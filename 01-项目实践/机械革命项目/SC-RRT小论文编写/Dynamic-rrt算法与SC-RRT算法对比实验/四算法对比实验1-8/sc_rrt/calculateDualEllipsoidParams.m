function [c_best_A, c_best_B, c_min_A, c_min_B] = calculateDualEllipsoidParams(treeA, treeB, startPoint, goalPoint, meetPoint, m, buffer)
% calculateDualEllipsoidParams - 高性能双椭球体参数计算（向量化优化版）

if nargin < 7, buffer = 1.2; end

if isempty(treeA) || isempty(treeB)
    c_best_A = inf; c_best_B = inf;
    c_min_A = norm(meetPoint - startPoint);
    c_min_B = norm(goalPoint - meetPoint);
    return;
end

c_min_A = max(norm(meetPoint - startPoint), eps);
c_min_B = max(norm(goalPoint - meetPoint), eps);

% 向量化计算树A
sizeA = size(treeA, 1);
if sizeA > 1
    F_hat = treeA(1:sizeA, m+3);
    diffs = bsxfun(@minus, treeA(1:sizeA, 1:m), meetPoint);
    H_to_meet = sqrt(sum(diffs .* diffs, 2));
    c_best_A = min(F_hat + H_to_meet) * buffer;
    c_best_A = max(c_best_A, c_min_A * 1.1);
else
    c_best_A = inf;
end

% 向量化计算树B
sizeB = size(treeB, 1);
if sizeB > 1
    F_hat = treeB(1:sizeB, m+3);
    diffs = bsxfun(@minus, treeB(1:sizeB, 1:m), meetPoint);
    H_to_meet = sqrt(sum(diffs .* diffs, 2));
    c_best_B = min(F_hat + H_to_meet) * buffer;
    c_best_B = max(c_best_B, c_min_B * 1.1);
else
    c_best_B = inf;
end

% 确保双椭球体交集可达
c_total_min = norm(goalPoint - startPoint);
if isfinite(c_best_A) && isfinite(c_best_B)
    if c_best_A + c_best_B < c_total_min * 0.99
        scale_factor = (c_total_min * 1.05) / (c_best_A + c_best_B);
        c_best_A = c_best_A * scale_factor;
        c_best_B = c_best_B * scale_factor;
    end
end

if ~isfinite(c_best_A), c_best_A = c_min_A * buffer * 2; end
if ~isfinite(c_best_B), c_best_B = c_min_B * buffer * 2; end

end
