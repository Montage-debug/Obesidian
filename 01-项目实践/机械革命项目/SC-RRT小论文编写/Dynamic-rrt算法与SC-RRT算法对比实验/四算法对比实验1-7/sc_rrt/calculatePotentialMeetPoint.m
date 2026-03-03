function [meetPoint, centroidA, centroidB] = calculatePotentialMeetPoint(treeA, treeB, startPoint, goalPoint, m)
% calculatePotentialMeetPoint - 高性能动态交汇点计算（向量化优化版）

sizeA = size(treeA, 1);
sizeB = size(treeB, 1);

if sizeA == 0 || sizeB == 0
    meetPoint = (startPoint + goalPoint) / 2;
    centroidA = startPoint;
    centroidB = goalPoint;
    return;
end

% 向量化计算树A的有效质心（前30%最优节点）
centroidA = fastEffectiveCentroid(treeA, goalPoint, m, sizeA);
centroidB = fastEffectiveCentroid(treeB, startPoint, m, sizeB);

% 权重基于树的大小
weightA = sizeA / (sizeA + sizeB);
weightB = 1 - weightA;

% 势场法计算交汇点
meetPoint = weightA * centroidB + weightB * centroidA;

% 边界约束
minBound = min([startPoint; goalPoint]);
maxBound = max([startPoint; goalPoint]);
meetPoint = max(meetPoint, minBound);
meetPoint = min(meetPoint, maxBound);

end

function centroid = fastEffectiveCentroid(tree, targetPoint, m, n)
% 向量化计算有效质心

if n <= 1
    centroid = tree(1, 1:m);
    return;
end

% 向量化F = G + H计算
G = tree(1:n, m+2);
diffs = bsxfun(@minus, tree(1:n, 1:m), targetPoint);
H = sqrt(sum(diffs .* diffs, 2));
costs = G + H;

% 取前30%最优节点
topK = max(1, floor(n * 0.3));
if topK >= n
    centroid = mean(tree(1:n, 1:m), 1);
    return;
end

% 使用 mink 避免完整排序（O(n)代替O(nlogn)）
[~, topIdx] = mink(costs, topK);
centroid = mean(tree(topIdx, 1:m), 1);

end
