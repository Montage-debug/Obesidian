function metrics = evaluatePathQuality(path, obstacles, m)
% evaluatePathQuality - 全面评估路径质量
%
% 输入:
%   path      - 路径点 [N×m]
%   obstacles - 障碍物结构体
%   m         - 维度 (2或3)
%
% 输出:
%   metrics - 结构体，包含:
%     length         - 路径总长度
%     smoothness     - 平滑度（平均转角，越小越平滑）
%     curvature      - 平均曲率
%     maxCurvature   - 最大曲率
%     safetyMargin   - 安全裕度（到最近障碍物的最小距离）
%     turnAngleStd   - 转角标准差（越小越一致）
%     clearanceScore - 综合安全评分 (0-100)
%     qualityScore   - 综合质量评分 (0-100)

metrics = struct();

% 路径点数量
N = size(path, 1);

if N < 2
    % 路径太短，返回默认值
    metrics.length = 0;
    metrics.smoothness = inf;
    metrics.curvature = inf;
    metrics.maxCurvature = inf;
    metrics.safetyMargin = inf;
    metrics.turnAngleStd = inf;
    metrics.clearanceScore = 0;
    metrics.qualityScore = 0;
    return;
end

%% 1. 路径长度
segmentLengths = zeros(N-1, 1);
for i = 1:N-1
    segmentLengths(i) = norm(path(i+1, :) - path(i, :));
end
metrics.length = sum(segmentLengths);

%% 2. 平滑度（基于转角）
if N < 3
    metrics.smoothness = 0;
    metrics.turnAngleStd = 0;
    turnAngles = [];
else
    turnAngles = zeros(N-2, 1);
    for i = 2:N-1
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        
        norm_v1 = norm(v1);
        norm_v2 = norm(v2);
        
        if norm_v1 > eps && norm_v2 > eps
            cosAngle = dot(v1, v2) / (norm_v1 * norm_v2);
            cosAngle = max(-1, min(1, cosAngle));  % 钳位到[-1,1]
            turnAngles(i-1) = acos(cosAngle);  % 弧度
        else
            turnAngles(i-1) = 0;
        end
    end
    
    metrics.smoothness = mean(turnAngles);  % 平均转角（弧度）
    metrics.turnAngleStd = std(turnAngles);  % 转角标准差
end

%% 3. 曲率
if N < 3
    metrics.curvature = 0;
    metrics.maxCurvature = 0;
else
    curvatures = zeros(N-2, 1);
    for i = 2:N-1
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        
        norm_v1 = norm(v1);
        norm_v2 = norm(v2);
        
        if norm_v1 > eps && norm_v2 > eps
            % 使用Menger曲率公式
            % K = 2 * sin(theta) / |chord|
            cosAngle = dot(v1, v2) / (norm_v1 * norm_v2);
            cosAngle = max(-1, min(1, cosAngle));
            angle = acos(cosAngle);
            
            chord = norm(path(i+1, :) - path(i-1, :));
            if chord > eps
                curvatures(i-1) = 2 * sin(angle) / chord;
            else
                curvatures(i-1) = 0;
            end
        else
            curvatures(i-1) = 0;
        end
    end
    
    metrics.curvature = mean(curvatures);
    metrics.maxCurvature = max(curvatures);
end

%% 4. 安全裕度（到障碍物的最小距离）
minDistances = inf(N, 1);

if m == 2 && isfield(obstacles, 'circles') && ~isempty(obstacles.circles)
    for i = 1:N
        point = path(i, :);
        for j = 1:size(obstacles.circles, 1)
            center = obstacles.circles(j, 1:2);
            radius = obstacles.circles(j, 3);
            dist = norm(point - center) - radius;
            minDistances(i) = min(minDistances(i), dist);
        end
    end
elseif m == 3 && isfield(obstacles, 'spheres') && ~isempty(obstacles.spheres)
    for i = 1:N
        point = path(i, :);
        for j = 1:size(obstacles.spheres, 1)
            center = obstacles.spheres(j, 1:3);
            radius = obstacles.spheres(j, 4);
            dist = norm(point - center) - radius;
            minDistances(i) = min(minDistances(i), dist);
        end
    end
end

metrics.safetyMargin = min(minDistances);

%% 5. 综合安全评分 (0-100)
% 基于安全裕度的评分，距离越大评分越高
if isinf(metrics.safetyMargin)
    metrics.clearanceScore = 100;
else
    % 使用sigmoid函数映射
    % 距离>50时得分接近100，距离<10时得分较低
    metrics.clearanceScore = 100 * (1 / (1 + exp(-0.1 * (metrics.safetyMargin - 20))));
end

%% 6. 综合质量评分 (0-100)
% 综合考虑：长度、平滑度、曲率、安全性
% 归一化各项指标

% 平滑度评分（转角越小越好，0度=100分，180度=0分）
smoothnessScore = 100 * (1 - metrics.smoothness / pi);

% 曲率评分（曲率越小越好）
curvatureScore = 100 * (1 / (1 + metrics.curvature * 100));

% 一致性评分（转角标准差越小越好）
if N >= 3
    consistencyScore = 100 * (1 / (1 + metrics.turnAngleStd * 10));
else
    consistencyScore = 100;
end

% 综合评分（加权平均）
weights = [0.2, 0.3, 0.2, 0.3];  % [长度、平滑、曲率、安全]
normalizedLength = 100 * (1 / (1 + metrics.length / 1000));  % 假设1000为基准

metrics.qualityScore = weights(1) * normalizedLength + ...
                       weights(2) * smoothnessScore + ...
                       weights(3) * curvatureScore + ...
                       weights(4) * metrics.clearanceScore;

metrics.qualityScore = max(0, min(100, metrics.qualityScore));  % 钳位到[0,100]

end
