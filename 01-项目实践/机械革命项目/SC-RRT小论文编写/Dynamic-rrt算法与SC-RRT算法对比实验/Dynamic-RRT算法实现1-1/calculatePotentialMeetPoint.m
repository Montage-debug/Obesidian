function [meetPoint, centroidA, centroidB] = calculatePotentialMeetPoint(treeA, treeB, startPoint, goalPoint, m)
% calculatePotentialMeetPoint - 使用势场法计算动态交汇点
%
% 功能: 计算双向树的有效中心，引导两树交汇
% 将交汇点作为动态目标点，引导双向收敛
%
% 输入:
%   treeA      - 起点树 [N×(m+4)]
%   treeB      - 终点树 [M×(m+4)]
%   startPoint - 起始点 [1×m]
%   goalPoint  - 目标点 [1×m]
%   m          - 空间维度
%
% 输出:
%   meetPoint  - 动态交汇点 [1×m]
%   centroidA  - 树A有效质心
%   centroidB  - 树B有效质心

% ========== 计算树A有效质心 ==========
% 只考虑离目标点最近的前30%优质节点
[centroidA, ~] = calculateEffectiveCentroid(treeA, goalPoint, m);

% ========== 计算树B有效质心 ==========
% 只考虑离起点最近的前30%优质节点
[centroidB, ~] = calculateEffectiveCentroid(treeB, startPoint, m);

% ========== 根据树大小 权重 ==========
% 权重：树越大越有权重，这样小树被吸引向大树靠近
sizeA = size(treeA, 1);
sizeB = size(treeB, 1);
weightA = sizeA / (sizeA + sizeB);
weightB = 1 - weightA;

% ========== 势场方程计算交汇点 ==========
% 树A的吸引力指向树B的质心
% 树B的吸引力指向树A的质心
% 交汇点位置为加权平均
meetPoint = weightA * centroidB + weightB * centroidA;

% ========== 边界约束 ==========
% 确保交汇点不超出目标点的范围
minBound = min([startPoint; goalPoint]);
maxBound = max([startPoint; goalPoint]);
meetPoint = max(meetPoint, minBound);
meetPoint = min(meetPoint, maxBound);

end

%% ========== 辅助函数：计算有效质心 ==========
function [centroid, bestCost] = calculateEffectiveCentroid(tree, targetPoint, m)
% 计算树的有效质心（筛选优质节点）
%
% 说明：只选择前30%代价最小的节点，这些被视为优质节点

n = size(tree, 1);
if n == 0
    centroid = tree(1, 1:m);
    bestCost = inf;
    return;
end

% 计算每个节点到目标点的累计代价
costs = zeros(n, 1);
for i = 1:n
    % F(i) = G(i) + H(i, target)
    G_i = tree(i, m+2);  % 实际代价
    H_i = norm(tree(i, 1:m) - targetPoint);  % 启发式代价
    costs(i) = G_i + H_i;
end

% 选择前30%代价最小的优质节点
[sortedCosts, sortedIdx] = sort(costs);
topPercent = max(1, floor(n * 0.3));
topNodes = sortedIdx(1:topPercent);

% 计算这些优质节点的均值
centroid = mean(tree(topNodes, 1:m), 1);
bestCost = sortedCosts(1);

end
