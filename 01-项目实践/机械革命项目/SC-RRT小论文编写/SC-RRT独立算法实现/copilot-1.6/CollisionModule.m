function result = CollisionModule(operation, varargin)
% CollisionModule - 统一的碰撞检测模块
%
% 功能：整合碰撞检测、最近邻搜索、节点扩展等功能
%
% 用法：
%   isFree = CollisionModule('check', point1, point2, obstacles, m)
%   [nearIdx, nearPt] = CollisionModule('nearest', tree, targetPoint, m)
%   newPoint = CollisionModule('expand', nearPoint, targetPoint, stepSize)

switch operation
    case 'check'
        % 碰撞检测
        if length(varargin) < 4
            error('CollisionModule(check): 需要 point1, point2, obstacles, m 参数');
        end
        point1 = varargin{1};
        point2 = varargin{2};
        obstacles = varargin{3};
        m = varargin{4};
        result = checkCollision(point1, point2, obstacles, m);
        
    case 'nearest'
        % 最近邻搜索
        if length(varargin) < 3
            error('CollisionModule(nearest): 需要 tree, targetPoint, m 参数');
        end
        tree = varargin{1};
        targetPoint = varargin{2};
        m = varargin{3};
        [result.index, result.point] = findNearest(tree, targetPoint, m);
        
    case 'expand'
        % 节点扩展
        if length(varargin) < 3
            error('CollisionModule(expand): 需要 nearPoint, targetPoint, stepSize 参数');
        end
        nearPoint = varargin{1};
        targetPoint = varargin{2};
        stepSize = varargin{3};
        result = expandNode(nearPoint, targetPoint, stepSize);
        
    otherwise
        error('未知操作: %s', operation);
end

end

%% ========== 子函数1: 碰撞检测 ==========
function isFree = checkCollision(point1, point2, obstacles, m)
% 检测线段路径是否与障碍物碰撞

% 参数验证
if length(point1) ~= m || length(point2) ~= m
    error('point1 和 point2 的维度必须为 %d', m);
end

% 确保输入为行向量
point1 = reshape(point1, 1, m);
point2 = reshape(point2, 1, m);

% 初始化返回值
isFree = true;

% 计算线段方向和长度
direction = point2 - point1;
segmentLength = norm(direction);

if segmentLength <= eps
    return;  % 点重合，无碰撞
end

unitDirection = direction / segmentLength;

% ========== 方法1: 离散采样检测 ==========
numSamples = 10;
for k = 0:numSamples
    t = k / numSamples * segmentLength;
    samplePoint = point1 + t * unitDirection;
    
    if m == 2
        if isfield(obstacles, 'circles') && ~isempty(obstacles.circles)
            for j = 1:size(obstacles.circles, 1)
                center = obstacles.circles(j, 1:2);
                radius = obstacles.circles(j, 3);
                if norm(samplePoint - center) < radius - eps
                    isFree = false;
                    return;
                end
            end
        end
    else % m == 3
        if isfield(obstacles, 'spheres') && ~isempty(obstacles.spheres)
            for j = 1:size(obstacles.spheres, 1)
                center = obstacles.spheres(j, 1:3);
                radius = obstacles.spheres(j, 4);
                if norm(samplePoint - center) < radius - eps
                    isFree = false;
                    return;
                end
            end
        end
    end
end

% ========== 方法2: 精确解析检测 ==========
if m == 2
    if isfield(obstacles, 'circles') && ~isempty(obstacles.circles)
        for j = 1:size(obstacles.circles, 1)
            center = obstacles.circles(j, 1:2);
            radius = obstacles.circles(j, 3);
            
            % 线段与圆的交点检测
            d = point1 - center;
            a = dot(unitDirection, unitDirection);
            b = 2 * dot(d, unitDirection);
            c = dot(d, d) - radius^2;
            discriminant = b^2 - 4 * a * c;
            
            if discriminant > -eps
                t1 = (-b - sqrt(max(discriminant, 0))) / (2 * a);
                t2 = (-b + sqrt(max(discriminant, 0))) / (2 * a);
                if (t1 >= -eps && t1 <= segmentLength + eps) || ...
                   (t2 >= -eps && t2 <= segmentLength + eps)
                    isFree = false;
                    return;
                end
            end
        end
    end
else % m == 3
    if isfield(obstacles, 'spheres') && ~isempty(obstacles.spheres)
        for j = 1:size(obstacles.spheres, 1)
            center = obstacles.spheres(j, 1:3);
            radius = obstacles.spheres(j, 4);
            
            % 线段与球的交点检测
            d = point1 - center;
            a = dot(unitDirection, unitDirection);
            b = 2 * dot(d, unitDirection);
            c = dot(d, d) - radius^2;
            discriminant = b^2 - 4 * a * c;
            
            if discriminant > -eps
                t1 = (-b - sqrt(max(discriminant, 0))) / (2 * a);
                t2 = (-b + sqrt(max(discriminant, 0))) / (2 * a);
                if (t1 >= -eps && t1 <= segmentLength + eps) || ...
                   (t2 >= -eps && t2 <= segmentLength + eps)
                    isFree = false;
                    return;
                end
            end
        end
    end
end

end

%% ========== 子函数2: 最近邻搜索 ==========
function [nearestIndex, nearestPoint] = findNearest(tree, targetPoint, m)
% 在树中寻找距离目标点最近的节点

if size(tree, 2) < m + 4
    error('树维度与目标点不匹配');
end

% 确保 targetPoint 为行向量
targetPoint = reshape(targetPoint, 1, m);

% 向量化计算距离
distances = sqrt(sum((tree(:, 1:m) - repmat(targetPoint, size(tree, 1), 1)).^2, 2));

% 找到最小距离索引
[~, nearestIndex] = min(distances);
nearestPoint = tree(nearestIndex, 1:m);

end

%% ========== 子函数3: 节点扩展 ==========
function newPoint = expandNode(nearPoint, targetPoint, stepSize)
% Steer 函数：从 nearPoint 向 targetPoint 扩展

direction = targetPoint - nearPoint;
distance = norm(direction);

if distance > stepSize
    % 按步长移动
    newPoint = nearPoint + (direction / distance) * stepSize;
else
    % 直接到达目标点
    newPoint = targetPoint;
end

end
