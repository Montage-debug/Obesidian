function [xPareto, c_best, c_min, paretoIndices, bestNodeIdx] = ParetoModule(tree, goalPoint, p_nonPareto, m)
% ParetoModule - 统一的Pareto前沿选择模块（改进版）
%
% 功能：基于三维Pareto支配关系选择最优节点
%
% 输入：
%   tree         - 树结构 [N×(m+4)]
%   goalPoint    - 目标点 [1×m]
%   p_nonPareto  - 非Pareto节点选择概率（默认0.1）
%   m            - 空间维度
%
% 输出：
%   xPareto       - 选中的Pareto最优节点坐标 [1×m]
%   c_best        - 当前Pareto最优节点的路径代价
%   c_min         - 根节点到目标点的直线距离
%   paretoIndices - Pareto前沿节点索引列表
%   bestNodeIdx   - 综合评分最优的Pareto节点索引（用于可视化）

% ========== 输入验证 ==========
[n, cols] = size(tree);
if cols < m + 4
    error('ParetoModule: 树结构维度错误');
end

if nargin < 3
    p_nonPareto = 0.1;
end

% ========== 构建三维Pareto目标 ==========
V = zeros(n, 3);

for i = 1:n
    % 维度1: 节点深度（深度越大越好，取负数使越小越好）
    V(i, 1) = -tree(i, m+4);
    
    % 维度2: F_hat总代价
    V(i, 2) = tree(i, m+3);
    
    % 维度3: 归一化路径曲折度
    V(i, 3) = computePathTortuosity(tree, i, m);
end

% ========== 计算Pareto前沿 ==========
paretoIndices = computeParetoFront(V);
nonIdx = setdiff(1:n, paretoIndices);

% ========== 多目标综合评分（改进版：归一化+权重） ==========
% 对Pareto前沿节点进行归一化和加权评分
if ~isempty(paretoIndices)
    V_pareto = V(paretoIndices, :);
    
    % 归一化每个维度到[0, 1]
    V_norm = zeros(size(V_pareto));
    for d = 1:3
        min_val = min(V_pareto(:, d));
        max_val = max(V_pareto(:, d));
        
        if max_val - min_val > 1e-6
            V_norm(:, d) = (V_pareto(:, d) - min_val) / (max_val - min_val);
        else
            V_norm(:, d) = 0;  % 所有值相同时设为0
        end
    end
    
    % 多目标权重（可调整）
    % w1: 深度权重（鼓励探索更深的节点）
    % w2: 代价权重（主要优化目标）
    % w3: 曲折度权重（辅助平滑性优化）
    weights = [0.2, 0.6, 0.2];  
    
    % 加权综合评分（越小越好）
    scores = V_norm * weights';
    
    % 找到综合评分最优的节点
    [~, bestIdx] = min(scores);
    bestNodeIdx = paretoIndices(bestIdx);
else
    % 没有Pareto前沿时，选择F_hat最小的节点
    [~, bestNodeIdx] = min(V(:, 2));
end

% ========== 选择策略 ==========
if rand < p_nonPareto && ~isempty(nonIdx)
    % 以概率p选择非Pareto节点（增加多样性）
    pick = nonIdx(randi(numel(nonIdx)));
else
    % 以概率(1-p)选择Pareto前沿节点
    if ~isempty(paretoIndices)
        % 优先选择综合评分最优的节点
        if rand < 0.7
            pick = bestNodeIdx;
        else
            % 30%概率随机选择其他Pareto节点（探索）
            pick = paretoIndices(randi(numel(paretoIndices)));
        end
    else
        pick = 1;  % 默认选择根节点
    end
end

% ========== 返回选中节点 ==========
xPareto = tree(pick, 1:m);

% ========== 计算超椭球参数 ==========
if ~isempty(paretoIndices)
    % 使用综合评分最优节点的F_hat
    c_best = tree(bestNodeIdx, m+3);
else
    c_best = inf;
end

% 计算根节点到目标点的直线距离
c_min = norm(tree(1, 1:m) - goalPoint(1:m));

end

%% ========== �Ӻ�??1: ����Paretoǰ�� ==========
function paretoIndices = computeParetoFront(V)
% �����֧��⣨Paretoǰ��??
%
% ����??
%   V - Pareto�������� [n��d]
% ���??
%   paretoIndices - ��֧�������

n = size(V, 1);
isDominated = false(n, 1);

for i = 1:n
    for j = 1:n
        if i ~= j
            % ���j�Ƿ�֧��i
            if all(V(j, :) <= V(i, :)) && any(V(j, :) < V(i, :))
                isDominated(i) = true;
                break;
            end
        end
    end
end

paretoIndices = find(~isDominated);
end

%% ========== �Ӻ�??2: ����·������?? ==========
function lambda_norm = computePathTortuosity(tree, idx, m)
% ����ڵ�Ĺ�һ��·�����۶�
%
% ��ʽ??
%   ��_raw = G(x) / ||x - x_start||
%   ��_norm = 1 - 1/��_raw ?? [0, 1)

% ������??
if isempty(tree) || idx <= 0 || idx > size(tree, 1)
    lambda_norm = 0;
    return;
end

if size(tree, 2) < m + 4
    lambda_norm = 0;
    return;
end

% ��ȡ�ڵ���Ϣ
G_x = tree(idx, m+2);           % ʵ���ۻ�����
x_pos = tree(idx, 1:m);         % �ڵ�����
x_start = tree(1, 1:m);         % �������

% ����ֱ�߾���
H_direct = norm(x_pos - x_start);

% ��ֹ����
epsilon = 1e-6;

if H_direct < epsilon
    lambda_norm = 0;  % ���򼫽���??
    return;
end

% ����ԭʼ����??
lambda_raw = G_x / (H_direct + epsilon);

% ȷ�� ��_raw >= 1
if lambda_raw < 1.0
    lambda_raw = 1.0;
end

% ��һ���� [0, 1)
lambda_norm = 1.0 - 1.0 / lambda_raw;

% ��ȫ�޷�
lambda_norm = max(0.0, min(lambda_norm, 0.9999));

end
