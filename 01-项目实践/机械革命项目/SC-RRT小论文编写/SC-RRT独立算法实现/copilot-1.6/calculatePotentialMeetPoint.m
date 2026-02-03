function [meetPoint, centroidA, centroidB] = calculatePotentialMeetPoint(treeA, treeB, startPoint, goalPoint, m)
% calculatePotentialMeetPoint - 使用势场理论计算动态会合点
%
% ★★★ 核心创新函数 ★★★
% 利用人工势场理论，基于两棵树的有效质心计算最优会合点
%
% 输入:
%   treeA      - 起点树 [N×(m+4)]
%   treeB      - 终点树 [M×(m+4)]
%   startPoint - 起始点 [1×m]
%   goalPoint  - 目标点 [1×m]
%   m          - 空间维度
%
% 输出:
%   meetPoint  - 动态会合点 [1×m]
%   centroidA  - 树A的有效质心
%   centroidB  - 树B的有效质心

% ========== 计算树A的有效质心 ==========
% 只考虑朝向目标方向的前30%优质节点
[centroidA, ~] = calculateEffectiveCentroid(treeA, goalPoint, m);

% ========== 计算树B的有效质心 ==========
% 只考虑朝向起点方向的前30%优质节点
[centroidB, ~] = calculateEffectiveCentroid(treeB, startPoint, m);

% ========== 计算动态权重 ==========
% 权重基于树的大小：树越大，对会合点的影响越大
sizeA = size(treeA, 1);
sizeB = size(treeB, 1);
weightA = sizeA / (sizeA + sizeB);
weightB = 1 - weightA;

% ========== 势场引导的会合点计算 ==========
% 树A的质心吸引会合点朝向树B的质心
% 树B的质心吸引会合点朝向树A的质心
% 最终会合点是两个吸引力的平衡点
meetPoint = weightA * centroidB + weightB * centroidA;

% ========== 边界约束 ==========
% 确保会合点在起点和终点的包围盒内
minBound = min([startPoint; goalPoint]);
maxBound = max([startPoint; goalPoint]);
meetPoint = max(meetPoint, minBound);
meetPoint = min(meetPoint, maxBound);

end

%% ========== 辅助函数：计算有效质心 ==========
function [centroid, bestCost] = calculateEffectiveCentroid(tree, targetPoint, m)
% 计算树的有效质心（排除低质量节点）
%
% 策略：只选择前30%代价最小的节点，避免被远离目标的节点干扰

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

% 排序并选择前30%的优质节点
[sortedCosts, sortedIdx] = sort(costs);
topPercent = max(1, floor(n * 0.3));
topNodes = sortedIdx(1:topPercent);

% 计算这些优质节点的质心
centroid = mean(tree(topNodes, 1:m), 1);
bestCost = sortedCosts(1);

end
