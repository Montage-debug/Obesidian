function efficiency = PerformanceModule(metricType, varargin)
% PerformanceModule - 统一的性能评估模块
%
% 功能：
%   综合检索效率、路径冗长等性能指标计算
%
% 用法：
%   eff = PerformanceModule('search_efficiency', validNodes, totalNodes, pathHistory, windowSize)
%   tortuosity = PerformanceModule('path_tortuosity', actualLength, directLength)

switch metricType
    case 'search_efficiency'
        % 计算检索效率
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
        % 计算路径弯曲度
        if length(varargin) < 2
            error('PerformanceModule(path_tortuosity): 需要 actualLength, directLength 参数');
        end
        actualLength = varargin{1};
        directLength = varargin{2};
        efficiency = computeTortuosity(actualLength, directLength);
        
    otherwise
        error('未知指标类型: %s', metricType);
end

end

%% ========== 子函数: 计算检索效率 ==========
function efficiency = computeSearchEfficiency(validNodeCount, totalNodeCount, ...
                                             pathImprovementHistory, windowSize)
% 计算综合检索效率指标
%
% 公式：
%   η(t) = w_node × η_node(t) + w_path × η_path(t)

% 效率权重
w_node = 0.4;  % 节点效率权重
w_path = 0.6;  % 路径改进效率权重

% 1. 计算节点效率（有效节点比率）
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

%% ========== 子函数: 计算路径弯曲度 ==========
function tortuosity = computeTortuosity(actualLength, directLength)
% 计算路径弯曲度
%
% 公式：
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