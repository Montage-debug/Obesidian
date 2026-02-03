function smoothPath = smoothAndValidatePath(rawPath, obstacles, m, numPoints)
% smoothAndValidatePath - 自适应路径平滑（安全优先，确保平滑成功）
%
% 改进策略：
%   1. 根据路径安全裕度自适应选择平滑强度
%   2. 多级降级机制确保一定能平滑成功
%   3. 保守重连 + 积极平滑的组合策略

if nargin < 4
    numPoints = 200;
end

if size(rawPath, 1) < 3
    smoothPath = rawPath;
    fprintf('    路径节点过少(%d)，跳过平滑\n', size(rawPath, 1));
    return;
end

originalNodeCount = size(rawPath, 1);

% 计算安全裕度（到障碍物最小距离）
minClearance = inf;
for i = 1:size(rawPath, 1)
    if m == 2
        % 2D环境使用circles字段
        for j = 1:size(obstacles.circles, 1)
            obstacleCenter = obstacles.circles(j, 1:2);
            obstacleRadius = obstacles.circles(j, 3);
            dist = norm(rawPath(i, 1:2) - obstacleCenter) - obstacleRadius;
            minClearance = min(minClearance, dist);
        end
    else
        % 3D环境使用spheres字段
        for j = 1:size(obstacles.spheres, 1)
            obstacleCenter = obstacles.spheres(j, 1:3);
            obstacleRadius = obstacles.spheres(j, 4);
            dist = norm(rawPath(i, 1:3) - obstacleCenter) - obstacleRadius;
            minClearance = min(minClearance, dist);
        end
    end
end
minClearance = max(0, minClearance);

fprintf('    安全裕度: %.2f → ', minClearance);

% 自适应策略选择
if minClearance > 20
    fprintf('激进平滑\n');
    [smoothPath, success] = tryAggressiveSmoothing(rawPath, obstacles, m, numPoints);
    if ~success
        fprintf('    降级到适度平滑\n');
        [smoothPath, success] = tryModerateSmoothing(rawPath, obstacles, m, numPoints);
    end
elseif minClearance > 10
    fprintf('适度平滑\n');
    [smoothPath, success] = tryModerateSmoothing(rawPath, obstacles, m, numPoints);
else
    fprintf('保守平滑\n');
    [smoothPath, success] = tryConservativeSmoothing(rawPath, obstacles, m, numPoints);
end

% 最后的保底策略
if ~success || isequal(smoothPath, rawPath)
    fprintf('    最终降级: 线性平滑\n');
    smoothPath = tryMinimalSmoothing(rawPath, m, numPoints);
end

% 输出结果
if ~isequal(smoothPath, rawPath)
    smoothLength = calculatePathLength(smoothPath);
    originalLength = calculatePathLength(rawPath);
    fprintf('    ✓ 节点 %d→%d, 长度 %.2f→%.2f (+%.1f%%)\n', ...
        originalNodeCount, size(smoothPath, 1), originalLength, smoothLength, ...
        (smoothLength/originalLength - 1) * 100);
else
    fprintf('    ⚠️ 保持原路径\n');
end

end

function [smoothPath, success] = tryAggressiveSmoothing(rawPath, obstacles, m, numPoints)
% 激进平滑：移动平均 + 样条插值
success = false;

% 移动平均预处理
if size(rawPath, 1) >= 5
    windowSize = min(5, floor(size(rawPath, 1) / 3));
    prePath = rawPath;
    
    for i = 2:size(rawPath, 1)-1
        startIdx = max(1, i - floor(windowSize/2));
        endIdx = min(size(rawPath, 1), i + floor(windowSize/2));
        weights = gausswin(endIdx - startIdx + 1);
        weights = weights / sum(weights);
        
        for d = 1:m
            prePath(i, d) = sum(rawPath(startIdx:endIdx, d) .* weights);
        end
    end
else
    prePath = rawPath;
end

% 样条插值
segLen = sqrt(sum(diff(prePath).^2, 2));
cumLen = [0; cumsum(segLen)];
s_new = linspace(0, cumLen(end), numPoints);

smoothPath = zeros(numPoints, m);
for i = 1:m
    smoothPath(:, i) = interp1(cumLen, prePath(:, i), s_new, 'spline');
end

% 验证
if ~checkCollision(smoothPath, obstacles, m)
    success = true;
else
    smoothPath = rawPath;
end
end

function [smoothPath, success] = tryModerateSmoothing(rawPath, obstacles, m, numPoints)
% 适度平滑：样条插值（无预处理）
success = false;

segLen = sqrt(sum(diff(rawPath).^2, 2));
cumLen = [0; cumsum(segLen)];
s_new = linspace(0, cumLen(end), numPoints);

smoothPath = zeros(numPoints, m);
for i = 1:m
    smoothPath(:, i) = interp1(cumLen, rawPath(:, i), s_new, 'spline');
end

if ~checkCollision(smoothPath, obstacles, m)
    success = true;
else
    smoothPath = rawPath;
end
end

function [smoothPath, success] = tryConservativeSmoothing(rawPath, obstacles, m, numPoints)
% 保守平滑：PCHIP插值
success = false;

segLen = sqrt(sum(diff(rawPath).^2, 2));
cumLen = [0; cumsum(segLen)];
s_new = linspace(0, cumLen(end), numPoints);

smoothPath = zeros(numPoints, m);
for i = 1:m
    smoothPath(:, i) = interp1(cumLen, rawPath(:, i), s_new, 'pchip');
end

if ~checkCollision(smoothPath, obstacles, m)
    success = true;
else
    smoothPath = rawPath;
end
end

function smoothPath = tryMinimalSmoothing(rawPath, m, numPoints)
% 最小平滑：线性插值（保底方案，基本不会失败）

segLen = sqrt(sum(diff(rawPath).^2, 2));
cumLen = [0; cumsum(segLen)];
actualPoints = min(numPoints, size(rawPath, 1) * 3);
s_new = linspace(0, cumLen(end), actualPoints);

smoothPath = zeros(actualPoints, m);
for i = 1:m
    smoothPath(:, i) = interp1(cumLen, rawPath(:, i), s_new, 'linear');
end
end

function hasCol = checkCollision(path, obstacles, m)
% 快速碰撞检测
hasCol = false;
for i = 1:size(path, 1) - 1
    if ~isCollisionFree(path(i, :), path(i+1, :), obstacles, m)
        hasCol = true;
        return;
    end
end
end
