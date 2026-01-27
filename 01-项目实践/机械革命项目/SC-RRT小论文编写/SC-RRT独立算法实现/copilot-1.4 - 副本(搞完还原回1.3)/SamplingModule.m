function sample = SamplingModule(samplingType, varargin)
% SamplingModule - 单一点采样模块
%
% 功能：在指定区域内采样点（空间均匀、目标偏置、椭球体积采样）
%
% 调用方法：
%   sample = SamplingModule('uniform', bounds, goalPoint)
%   sample = SamplingModule('goal_biased', goalPoint, radius, bounds)
%   sample = SamplingModule('ellipsoid', startPoint, goalPoint, cBest, bounds, m)
%
% 输入：
%   samplingType - 采样类型: 'uniform', 'goal_biased', 'ellipsoid'
%   varargin     - 根据采样类型的不同参数
%
% 输出：
%   sample - 采样点 [1 x m]

switch samplingType
    case 'uniform'
        % 空间均匀采样
        if length(varargin) < 2
            error('SamplingModule(uniform): 需要 bounds, goalPoint 参数');
        end
        bounds = varargin{1};
        goalPoint = varargin{2};
        sample = uniformSampling(bounds, goalPoint);
        
    case 'goal_biased'
        % 目标偏置采样
        if length(varargin) < 3
            error('SamplingModule(goal_biased): 需要 goalPoint, radius, bounds 参数');
        end
        goalPoint = varargin{1};
        radius = varargin{2};
        bounds = varargin{3};
        sample = goalBiasedSampling(goalPoint, radius, bounds);
        
    case 'ellipsoid'
        % 椭球体积采样
        if length(varargin) < 5
            error('SamplingModule(ellipsoid): 需要 startPoint, goalPoint, cBest, bounds, m 参数');
        end
        startPoint = varargin{1};
        goalPoint = varargin{2};
        cBest = varargin{3};
        bounds = varargin{4};
        m = varargin{5};
        sample = ellipsoidSampling(startPoint, goalPoint, cBest, bounds, m);
        
    otherwise
        error('不支持采样类型: %s', samplingType);
end

end

%% ========== 子函数: 空间均匀采样 ==========
function randomPoint = uniformSampling(bounds, goalPoint)
% 在空间内均匀采样，附20%目标偏置

m = length(goalPoint);
if length(bounds) ~= 2*m
    error('bounds 尺寸应为 %d 列, 2*m', 2*m);
end

goalBias = 0.2; % 目标偏置概率
if rand < goalBias
    % 目标附近高斯采样
    range = bounds(2:2:end) - bounds(1:2:end);
    offset = range .* (rand(1, m) - 0.5) * 0.1;
    randomPoint = goalPoint + offset;
else
    % 空间均匀采样
    randomPoint = zeros(1, m);
    for i = 1:m
        minBound = bounds(2*i-1);
        maxBound = bounds(2*i);
        randomPoint(i) = minBound + rand * (maxBound - minBound);
    end
end

% 边界裁剪
for i = 1:m
    randomPoint(i) = max(bounds(2*i-1), min(bounds(2*i), randomPoint(i)));
end
end

%% ========== 子函数: 目标偏置采样 ==========
function randomPoint = goalBiasedSampling(goalPoint, goalBiasRadius, bounds)
% 在目标点附近高斯采样

m = length(goalPoint);
if length(bounds) ~= 2*m
    error('bounds 尺寸应为 %d 列, 2*m', 2*m);
end

% 确保 goalPoint 为行向量
goalPoint = reshape(goalPoint, 1, m);

% 高斯采样
randomPoint = goalPoint + goalBiasRadius * randn(1, m);

% 边界裁剪
for i = 1:m
    minBound = bounds(2*i-1);
    maxBound = bounds(2*i);
    randomPoint(i) = max(minBound, min(maxBound, randomPoint(i)));
end
end

%% ========== 子函数: 椭球体积采样 ==========
function sample = ellipsoidSampling(startPoint, goalPoint, cBest, bounds, m)
% 椭球体积采样

% 如果未找到路径，返回空间均匀采样
if isinf(cBest)
    sample = uniformSampling(bounds, goalPoint);
    return;
end

% 计算路径长度
cMin = norm(goalPoint(1:m) - startPoint(1:m));
if cBest < cMin
    cBest = cMin;
end

% 椭球参数
a = cBest / 2; % 长半轴
b = sqrt(cBest^2 - cMin^2) / 2; % 短半轴

% 椭球中心
center = (startPoint(1:m) + goalPoint(1:m)) / 2;

% 主轴方向向量
eVec = (goalPoint(1:m) - startPoint(1:m)) / cMin;

% 计算旋转矩阵
R = computeRotationMatrix(eVec, m);

% 在椭球内采样并验证
max_attempts = 100;
for attempt = 1:max_attempts
    % 单位球采样
    xBall = randomInUnitBall(m);
    
    % 变换
    L = diag([a, repmat(b, 1, m-1)]);
    sample = R * (L * xBall) + center';
    
    % 验证: 1)边界检查 2)椭球内验证
    dist_sum = norm(sample - startPoint(1:m)') + norm(sample - goalPoint(1:m)');
    
    if all(sample >= bounds(1:2:end)') && all(sample <= bounds(2:2:end)') && ...
       dist_sum <= cBest * 1.001
        sample = sample'; % 转为行向量
        return;
    end
end

% 超过次数失败，返回中心附近随机点
sample = center' + randn(1, m) * min(a, b) * 0.1;
end

%% ========== 辅助函数1: 旋转矩阵计算 ==========
function R = computeRotationMatrix(eVec, m)
% 根据方向向量计算旋转矩阵

if m == 2
    theta = atan2(eVec(2), eVec(1));
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
elseif m == 3
    % 3D Rodrigues旋转公式
    v = [1; 0; 0];
    axis = cross(v, eVec(:));
    if norm(axis) < 1e-8
        R = eye(3);
    else
        axis = axis / norm(axis);
        angle = acos(dot(v, eVec(:)));
        K = [0 -axis(3) axis(2);
             axis(3) 0 -axis(1);
             -axis(2) axis(1) 0];
        R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
    end
else
    R = eye(m);
end
end

%% ========== 辅助函数2: 单位球内均匀采样 ==========
function x = randomInUnitBall(m)
% 单位球内均匀采样

% 随机方向，服从高斯分布后归一化
v = randn(m, 1);
v = v / norm(v);

% 随机半径，服从球体积分布
r = rand()^(1/m);

% 组合方向和半径
x = v * r;
end