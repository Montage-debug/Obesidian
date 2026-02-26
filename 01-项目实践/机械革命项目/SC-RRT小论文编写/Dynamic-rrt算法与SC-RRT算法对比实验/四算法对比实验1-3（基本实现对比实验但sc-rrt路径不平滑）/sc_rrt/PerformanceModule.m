function efficiency = PerformanceModule(metricType, varargin)
% PerformanceModule - 统一性能评估模块 (完全优化版)
%
% 功能：
%   综合计算搜索效率、路径曲折度等多种指标
%
% 用法：
%   eff = PerformanceModule('search_efficiency', validNodes, totalNodes, pathHistory, windowSize)
%   tortuosity = PerformanceModule('path_tortuosity', actualLength, directLength)
%   length = PerformanceModule('path_length', path)
%   smoothness = PerformanceModule('path_smoothness', path)

switch metricType
    case 'search_efficiency'
        % 综合搜索效率
        if length(varargin) < 3
            error('PerformanceModule(search_efficiency): 需要 validNodes, totalNodes, pathHistory 参数');
        end
        validNodeCount = varargin{1};
        totalNodeCount = varargin{2};
        pathImprovementHistory = varargin{3};
        
        if length(varargin) >= 4
            windowSize = varargin{4};
        else
            windowSize = 100;
        end
        
        efficiency = computeSearchEfficiency(validNodeCount, totalNodeCount, ...
                                            pathImprovementHistory, windowSize);
        
    case 'path_tortuosity'
        % 计算路径曲折度
        if length(varargin) < 2
            error('PerformanceModule(path_tortuosity): 需要 actualLength, directLength 参数');
        end
        actualLength = varargin{1};
        directLength = varargin{2};
        efficiency = computeTortuosity(actualLength, directLength);
        
    case 'path_length'
        % 路径长度
        if nargin < 2
            error('path_length需要参数：path');
        end
        path = varargin{1};
        efficiency = calculatePathLength(path);
        
    case 'path_smoothness'
        % 路径平滑度
        if nargin < 2
            error('path_smoothness需要参数：path');
        end
        path = varargin{1};
        efficiency = calculatePathSmoothness(path);
        
    otherwise
        error('未知指标类型: %s', metricType);
end

end

%% ========== 子函数: 综合搜索效率 ==========
function efficiency = computeSearchEfficiency(validNodeCount, totalNodeCount, ...
                                             pathImprovementHistory, windowSize)
% 计算综合搜索效率指标
%
% 公式:
%   η(t) = w_node × η_node(t) + w_path × η_path(t)

% 效率权重
w_node = 0.4;  % 节点效率权重
w_path = 0.6;  % 路径改进效率权重

% 1. 计算节点效率（有效节点占比）
if totalNodeCount == 0
    eta_node = 0;
else
    eta_node = validNodeCount / totalNodeCount;
end

% 2. 计算路径改进效率
eta_path = 0;
if ~isempty(pathImprovementHistory) && length(pathImprovementHistory) >= 2
    % 使用滑动窗口计算路径改进率
    startIdx = max(1, length(pathImprovementHistory) - windowSize + 1);
    recentHistory = pathImprovementHistory(startIdx:end);
    
    if length(recentHistory) >= 2
        initialLength = recentHistory(1);
        finalLength = recentHistory(end);
        
        if initialLength > 0 && finalLength < initialLength
            eta_path = (initialLength - finalLength) / initialLength;
        end
    end
end

% 3. 综合效率计算
efficiency = w_node * eta_node + w_path * eta_path;

% 保证效率在[0, 1]范围内
efficiency = max(0, min(1, efficiency));

end

%% ========== 子函数: 路径曲折度 ==========
function tortuosity = computeTortuosity(actualLength, directLength)
% 计算路径曲折度
%
% 公式:
%   λ_raw = G / H ∈ [1, ∞)
%   λ_norm = 1 - 1/λ_raw ∈ [0, 1)

epsilon = 1e-6;

if directLength < epsilon
    tortuosity = 0;  % 起点终点重合
    return;
end

lambda_raw = actualLength / (directLength + epsilon);

% 保证 λ_raw >= 1
if lambda_raw < 1.0
    lambda_raw = 1.0;
end

% 归一化
tortuosity = 1.0 - 1.0 / lambda_raw;

% 安全限幅
tortuosity = max(0.0, min(tortuosity, 0.9999));

end

%% ========== 子函数: 路径长度 ==========
function length = calculatePathLength(path)
% 计算路径总长度

if isempty(path) || size(path, 1) < 2
    length = 0;
    return;
end

length = 0;
for i = 1:size(path, 1)-1
    length = length + norm(path(i+1, :) - path(i, :));
end

end

%% ========== 子函数: 路径平滑度 ==========
function smoothness = calculatePathSmoothness(path)
% 计算路径平滑度（角度变化标准差）

if isempty(path) || size(path, 1) < 3
    smoothness = 0;
    return;
end

angles = zeros(size(path, 1) - 2, 1);

for i = 2:size(path, 1)-1
    v1 = path(i, :) - path(i-1, :);
    v2 = path(i+1, :) - path(i, :);
    
    % 计算夹角
    cos_angle = dot(v1, v2) / (norm(v1) * norm(v2) + 1e-10);
    cos_angle = max(-1, min(1, cos_angle));
    angles(i-1) = acos(cos_angle);
end

smoothness = std(angles);

end
