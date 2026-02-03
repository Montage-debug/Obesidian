function sample = SamplingModule(samplingType, varargin)
% SamplingModule - ��һ�����ģ��
%
% ���ܣ���ָ�������ڲ����㣨�ռ���ȡ�Ŀ��ƫ�á��������������
%
% ���÷�����
%   sample = SamplingModule('uniform', bounds, goalPoint)
%   sample = SamplingModule('goal_biased', goalPoint, radius, bounds)
%   sample = SamplingModule('ellipsoid', startPoint, goalPoint, cBest, bounds, m)
%
% ���룺
%   samplingType - ��������: 'uniform', 'goal_biased', 'ellipsoid'
%   varargin     - ���ݲ������͵Ĳ�ͬ����
%
% �����
%   sample - ������ [1 x m]

switch samplingType
    case 'uniform'
        % �ռ���Ȳ���
        if length(varargin) < 2
            error('SamplingModule(uniform): ��Ҫ bounds, goalPoint ����');
        end
        bounds = varargin{1};
        goalPoint = varargin{2};
        sample = uniformSampling(bounds, goalPoint);
        
    case 'goal_biased'
        % Ŀ��ƫ�ò���
        if length(varargin) < 3
            error('SamplingModule(goal_biased): ��Ҫ goalPoint, radius, bounds ����');
        end
        goalPoint = varargin{1};
        radius = varargin{2};
        bounds = varargin{3};
        sample = goalBiasedSampling(goalPoint, radius, bounds);
        
    case 'ellipsoid'
        % 椭球体内采样
        if length(varargin) < 5
            error('SamplingModule(ellipsoid): 需要 startPoint, goalPoint, cBest, bounds, m 参数');
        end
        startPoint = varargin{1};
        goalPoint = varargin{2};
        cBest = varargin{3};
        bounds = varargin{4};
        m = varargin{5};
        % 可选参数：gamma（膨胀系数）
        if length(varargin) >= 6
            gamma = varargin{6};
        else
            gamma = 1.0;
        end
        sample = ellipsoidSampling(startPoint, goalPoint, cBest, bounds, m, gamma);
        
    otherwise
        error('��֧�ֲ�������: %s', samplingType);
end

end

%% ========== �Ӻ���: �ռ���Ȳ��� ==========
function randomPoint = uniformSampling(bounds, goalPoint)
% �ڿռ��ھ��Ȳ�������20%Ŀ��ƫ��

m = length(goalPoint);
if length(bounds) ~= 2*m
    error('bounds �ߴ�ӦΪ %d ��, 2*m', 2*m);
end

goalBias = 0.2; % Ŀ��ƫ�ø���
if rand < goalBias
    % Ŀ�긽����˹����
    range = bounds(2:2:end) - bounds(1:2:end);
    offset = range .* (rand(1, m) - 0.5) * 0.1;
    randomPoint = goalPoint + offset;
else
    % �ռ���Ȳ���
    randomPoint = zeros(1, m);
    for i = 1:m
        minBound = bounds(2*i-1);
        maxBound = bounds(2*i);
        randomPoint(i) = minBound + rand * (maxBound - minBound);
    end
end

% �߽�ü�
for i = 1:m
    randomPoint(i) = max(bounds(2*i-1), min(bounds(2*i), randomPoint(i)));
end
end

%% ========== �Ӻ���: Ŀ��ƫ�ò��� ==========
function randomPoint = goalBiasedSampling(goalPoint, goalBiasRadius, bounds)
% ��Ŀ��㸽����˹����

m = length(goalPoint);
if length(bounds) ~= 2*m
    error('bounds �ߴ�ӦΪ %d ��, 2*m', 2*m);
end

% ȷ�� goalPoint Ϊ������
goalPoint = reshape(goalPoint, 1, m);

% ��˹����
randomPoint = goalPoint + goalBiasRadius * randn(1, m);

% �߽�ü�
for i = 1:m
    minBound = bounds(2*i-1);
    maxBound = bounds(2*i);
    randomPoint(i) = max(minBound, min(maxBound, randomPoint(i)));
end
end

%% ========== 子函数: 椭球体内采样 ==========
function sample = ellipsoidSampling(startPoint, goalPoint, cBest, bounds, m, gamma)
% 椭球体内采样（支持动态膨胀系数gamma）
%
% 输入：
%   startPoint - 起点（椭球焦点1）
%   goalPoint  - 终点（椭球焦点2）
%   cBest      - 当前最优路径代价
%   bounds     - 空间边界
%   m          - 维度
%   gamma      - 膨胀系数（默认1.0，>1时椭球变大）

if nargin < 6
    gamma = 1.0;
end

% 若未找到路径，返回空间均匀采样
if isinf(cBest)
    sample = uniformSampling(bounds, goalPoint);
    return;
end

% 焦点距离
cMin = norm(goalPoint(1:m) - startPoint(1:m));
if cBest < cMin
    cBest = cMin;
end

% 应用膨胀系数
cBest_expanded = gamma * cBest;

% 椭球参数
a = cBest_expanded / 2; % 半长轴
b = sqrt(cBest_expanded^2 - cMin^2) / 2; % 半短轴

% 椭球中心
center = (startPoint(1:m) + goalPoint(1:m)) / 2;

% 主轴方向向量
eVec = (goalPoint(1:m) - startPoint(1:m)) / cMin;

% 计算旋转矩阵
R = computeRotationMatrix(eVec, m);

% 椭球内采样并验证
max_attempts = 100;
for attempt = 1:max_attempts
    % 单位球采样
    xBall = randomInUnitBall(m);
    
    % 变换
    L = diag([a, repmat(b, 1, m-1)]);
    sample = R * (L * xBall) + center';
    
    % 验证: 1)边界内 2)椭球内验证（使用膨胀后的cBest）
    dist_sum = norm(sample - startPoint(1:m)') + norm(sample - goalPoint(1:m)');
    
    if all(sample >= bounds(1:2:end)') && all(sample <= bounds(2:2:end)') && ...
       dist_sum <= cBest_expanded * 1.001
        sample = sample'; % 转为行向量
        return;
    end
end

% 采样失败则返回中心附近的点
sample = center' + randn(1, m) * min(a, b) * 0.1;
end

%% ========== ��������1: ��ת������� ==========
function R = computeRotationMatrix(eVec, m)
% ���ݷ�������������ת����

if m == 2
    theta = atan2(eVec(2), eVec(1));
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
elseif m == 3
    % 3D Rodrigues��ת��ʽ
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

%% ========== ��������2: ��λ���ھ��Ȳ��� ==========
function x = randomInUnitBall(m)
% ��λ���ھ��Ȳ���

% ������򣬷��Ӹ�˹�ֲ����һ��
v = randn(m, 1);
v = v / norm(v);

% ����뾶������������ֲ�
r = rand()^(1/m);

% ��Ϸ���Ͱ뾶
x = v * r;
end