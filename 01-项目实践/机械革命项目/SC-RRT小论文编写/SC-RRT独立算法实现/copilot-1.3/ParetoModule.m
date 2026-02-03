function [xPareto, c_best, c_min, paretoIndices] = ParetoModule(tree, goalPoint, p_nonPareto, m)
% ParetoModule - 统一的Pareto前沿选择模块
%
% 功能：基于三维Pareto支配关系选择最优节??
%
% 输入??
%   tree         - 搜索?? [N×(m+4)]
%   goalPoint    - 目标?? [1×m]
%   p_nonPareto  - 非Pareto节点选择概率（默??0.1??
%   m            - 空间维度
%
% 输出??
%   xPareto       - 选中的Pareto最优节点坐?? [1×m]
%   c_best        - 当前Pareto最优节点的路径代价
%   c_min         - 起点到目标点的理论最短距??
%   paretoIndices - Pareto前沿节点索引列表

% ========== 参数验证 ==========
[n, cols] = size(tree);
if cols < m + 4
    error('ParetoModule: 树结构不完整');
end

if nargin < 3
    p_nonPareto = 0.1;
end

% ========== 构造三维Pareto向量 ==========
V = zeros(n, 3);

for i = 1:n
    % 维度1: 负出度（出度越大越好，取负后越小越好??
    V(i, 1) = -tree(i, m+4);
    
    % 维度2: F_hat代价估计
    V(i, 2) = tree(i, m+3);
    
    % 维度3: 归一化路径曲折度
    V(i, 3) = computePathTortuosity(tree, i, m);
end

% ========== 计算Pareto前沿 ==========
paretoIndices = computeParetoFront(V);
nonIdx = setdiff(1:n, paretoIndices);

% ========== 选择策略 ==========
if rand < p_nonPareto && ~isempty(nonIdx)
    % 以概率p选择非Pareto节点（增加多样性）
    pick = nonIdx(randi(numel(nonIdx)));
else
    % 以概??(1-p)选择Pareto前沿节点
    pick = paretoIndices(randi(numel(paretoIndices)));
end

% ========== 返回选中节点 ==========
xPareto = tree(pick, 1:m);

% ========== 计算超椭球参?? ==========
if ~isempty(paretoIndices)
    % 从Pareto前沿中找到F_hat最小的节点
    [~, minIdx] = min(V(paretoIndices, 2));
    bestNodeIdx = paretoIndices(minIdx);
    c_best = tree(bestNodeIdx, m+3);
else
    c_best = inf;
end

% 计算起点到目标点的理论最短距??
c_min = norm(tree(1, 1:m) - goalPoint(1:m));

end

%% ========== 子函??1: 计算Pareto前沿 ==========
function paretoIndices = computeParetoFront(V)
% 计算非支配解（Pareto前沿??
%
% 输入??
%   V - Pareto向量矩阵 [n×d]
% 输出??
%   paretoIndices - 非支配解索引

n = size(V, 1);
isDominated = false(n, 1);

for i = 1:n
    for j = 1:n
        if i ~= j
            % 检查j是否支配i
            if all(V(j, :) <= V(i, :)) && any(V(j, :) < V(i, :))
                isDominated(i) = true;
                break;
            end
        end
    end
end

paretoIndices = find(~isDominated);
end

%% ========== 子函??2: 计算路径曲折?? ==========
function lambda_norm = computePathTortuosity(tree, idx, m)
% 计算节点的归一化路径曲折度
%
% 公式??
%   λ_raw = G(x) / ||x - x_start||
%   λ_norm = 1 - 1/λ_raw ?? [0, 1)

% 参数检??
if isempty(tree) || idx <= 0 || idx > size(tree, 1)
    lambda_norm = 0;
    return;
end

if size(tree, 2) < m + 4
    lambda_norm = 0;
    return;
end

% 获取节点信息
G_x = tree(idx, m+2);           % 实际累积代价
x_pos = tree(idx, 1:m);         % 节点坐标
x_start = tree(1, 1:m);         % 起点坐标

% 计算直线距离
H_direct = norm(x_pos - x_start);

% 防止除零
epsilon = 1e-6;

if H_direct < epsilon
    lambda_norm = 0;  % 起点或极近起??
    return;
end

% 计算原始曲折??
lambda_raw = G_x / (H_direct + epsilon);

% 确保 λ_raw >= 1
if lambda_raw < 1.0
    lambda_raw = 1.0;
end

% 归一化到 [0, 1)
lambda_norm = 1.0 - 1.0 / lambda_raw;

% 安全限幅
lambda_norm = max(0.0, min(lambda_norm, 0.9999));

end
