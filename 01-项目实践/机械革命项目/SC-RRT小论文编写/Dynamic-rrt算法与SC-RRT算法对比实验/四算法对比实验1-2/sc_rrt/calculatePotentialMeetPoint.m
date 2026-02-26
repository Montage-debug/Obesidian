function [meetPoint, centroidA, centroidB] = calculatePotentialMeetPoint(treeA, treeB, startPoint, goalPoint, m)
% calculatePotentialMeetPoint - 使用势场方法计算动态交汇点（增强版）
%
% 核心改进：
%   1. 只考虑朝向目标方向的有效节点（前30%最优）
%   2. 基于代价预估（G+H）筛选节点，避免被远离目标的节点干扰
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
%   centroidA  - 树A的有效质心
%   centroidB  - 树B的有效质心

% 获取树的大小
sizeA = size(treeA, 1);
sizeB = size(treeB, 1);

if sizeA == 0 || sizeB == 0
    meetPoint = (startPoint + goalPoint) / 2;
    centroidA = startPoint;
    centroidB = goalPoint;
    return;
end

% ========== 计算树A的有效质心 ==========
% 只考虑朝向目标方向且代价较小的前30%节点
[centroidA, ~] = calculateEffectiveCentroid(treeA, goalPoint, m);

% ========== 计算树B的有效质心 ==========
% 只考虑朝向起点方向且代价较小的前30%节点
[centroidB, ~] = calculateEffectiveCentroid(treeB, startPoint, m);

% ========== 计算动态权重 ==========
% 权重基于树的大小：树越大，对交汇点影响越大
weightA = sizeA / (sizeA + sizeB);
weightB = 1 - weightA;

% ========== 势场法计算交汇点 ==========
% 树A的质心向树B的质心移动
% 树B的质心向树A的质心移动
% 最终交汇点是两者的加权平均
meetPoint = weightA * centroidB + weightB * centroidA;

% ========== 边界约束 ==========
% 确保交汇点在起点和终点的包围盒内
minBound = min([startPoint; goalPoint]);
maxBound = max([startPoint; goalPoint]);
meetPoint = max(meetPoint, minBound);
meetPoint = min(meetPoint, maxBound);

end

%% ========== 辅助函数：计算有效质心 ==========
function [centroid, bestCost] = calculateEffectiveCentroid(tree, targetPoint, m)
% 计算有效质心（排除远离目标的节点）
%
% 策略：只选择前30%代价最小的节点，避免被远离目标的节点干扰
%
% 策略：只选取前30%代价最小的节点，避免被远离目标的节点拖累

n = size(tree, 1);
if n == 0
    centroid = tree(1, 1:m);
    bestCost = inf;
    return;
end

% 计算每个节点到目标的估计总代价
costs = zeros(n, 1);
for i = 1:n
    % F(i) = G(i) + H(i, target)
    G_i = tree(i, m+2);  % 实际代价
    H_i = norm(tree(i, 1:m) - targetPoint);  % 启发式代价
    costs(i) = G_i + H_i;
end

% 排序选取前30%最优节点
[sortedCosts, sortedIdx] = sort(costs);
topPercent = max(1, floor(n * 0.3));
topNodes = sortedIdx(1:topPercent);

% 计算这些最优节点的质心
centroid = mean(tree(topNodes, 1:m), 1);
bestCost = sortedCosts(1);

end
