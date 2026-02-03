function result = PathModule(operation, varargin)
% PathModule - ͳһ��·������ģ�飨��ǿ�棩
%
% ���ܣ�����·��������·�����ȼ��㡢·��ƽ���ȹ���
%       ֧�ֶ���ƽ���㷨��pchip, bspline, arc, hybrid
%
% �÷���
%   path = PathModule('build', tree, goalIndex, goalPoint, m)
%   path = PathModule('build', tree, goalIndex, goalPoint, m, 'SmoothMethod', 'hybrid')
%   length = PathModule('length', path)
%   smoothPath = PathModule('smooth', rawPath, 'Method', 'hybrid', 'NumPoints', 150)
%
% ֧�ֵĲ�����
%   'build'  - ��������ȡ·����ƽ��
%   'length' - ����·���ܳ���
%   'smooth' - ·��ƽ������
%
% ��������: 2025-12-04
% ��������: B����ƽ����Բ������ƽ�������ƽ��

switch operation
    case 'build'
        % ����·��
        if length(varargin) < 4
            error('PathModule(build): ��Ҫ tree, goalIndex, goalPoint, m ����');
        end
        tree = varargin{1};
        goalIndex = varargin{2};
        goalPoint = varargin{3};
        m = varargin{4};
        
        % ������ѡ����
        p = inputParser;
        addParameter(p, 'SmoothMethod', 'hybrid', @ischar); % ƽ������
        addParameter(p, 'NumPoints', 150, @isnumeric);       % �������
        parse(p, varargin{5:end});
        
        result = buildPath(tree, goalIndex, goalPoint, m, ...
                          p.Results.SmoothMethod, p.Results.NumPoints);
        
    case 'length'
        % ����·������
        if length(varargin) < 1
            error('PathModule(length): ��Ҫ path ����');
        end
        path = varargin{1};
        result = computePathLength(path);
        
    case 'smooth'
        % ƽ��·����֧�ֶ��ַ�����
        if length(varargin) < 1
            error('PathModule(smooth): ��Ҫ rawPath ����');
        end
        rawPath = varargin{1};
        
        % ������ѡ����
        p = inputParser;
        addParameter(p, 'Method', 'hybrid', @ischar);        % ƽ������
        addParameter(p, 'NumPoints', 150, @isnumeric);       % �������
        addParameter(p, 'AngleThreshold', 20, @isnumeric);   % ת����ֵ���ȣ�
        addParameter(p, 'MaxCurvature', Inf, @isnumeric);    % �����������
        addParameter(p, 'Smoothness', 0.7, @isnumeric);      % B����ƽ����
        parse(p, varargin{2:end});
        
        result = smoothPath(rawPath, p.Results.Method, p.Results.NumPoints, ...
                           p.Results.AngleThreshold, p.Results.MaxCurvature, ...
                           p.Results.Smoothness);
        
    otherwise
        error('δ֪����: %s', operation);
end

end

%% ========== �Ӻ���: ����·�� ==========
function path = buildPath(tree, goalIndex, goalPoint, m, smoothMethod, numPoints)
% ��Ŀ��ڵ���ݵ����ڵ㣬����·����ƽ��

if size(tree, 2) < m + 4
    error('��ά����Ŀ��㲻ƥ��');
end

% ����ԭʼ·��
rawPath = goalPoint;
currentIndex = goalIndex;

while currentIndex > 0
    currentNode = tree(currentIndex, 1:m);
    rawPath = [currentNode; rawPath];
    currentIndex = tree(currentIndex, m+1);  % ���ڵ�����
end

% ʹ��ָ������ƽ��·��
path = smoothPath(rawPath, smoothMethod, numPoints, 20, Inf, 0.7);

end

%% ========== �Ӻ���: ����·������ ==========
function pathLength = computePathLength(path)
% ����·���ܳ��ȣ�֧������ά�ȣ�

if size(path, 1) < 2
    pathLength = 0;
    return;
end

% ʹ����������������·������
diffPath = diff(path);              % ���ڵ�֮��Ĳ��
squaredDiff = sum(diffPath.^2, 2);  % ���ƽ����
pathLength = sum(sqrt(squaredDiff)); % 累计欧几里得距离

end

%% ========== 子函数: 路径平滑（仅支持PCHIP）==========
function path = smoothPath(rawPath, ~, numPoints, ~, ~, ~)
% 使用PCHIP方法平滑路径

if size(rawPath, 1) < 2
    path = rawPath;
    return;
end

path = smoothPathPCHIPInternal(rawPath, numPoints);

end

%% ========== 子函数: PCHIP平滑（内部实现）==========
function path = smoothPathPCHIPInternal(rawPath, numPoints)
% 使用分段三次Hermite插值平滑路径

if size(rawPath, 1) < 2
    path = rawPath;
    return;
end

m = size(rawPath, 2);  % 空间维度

% 参数化原始路径
t = 1:size(rawPath, 1);
ts = linspace(1, size(rawPath, 1), numPoints);

% 对每个维度分别插值
path = zeros(numPoints, m);
for i = 1:m
    path(:, i) = interp1(t, rawPath(:, i), ts, 'pchip');
end

end