function env = generate3DEnvironment(bounds, num_obstacles, varargin)
% generate3DEnvironment - 快速生成3D环境配置
%
% 语法:
%   env = generate3DEnvironment(bounds, num_obstacles)
%   env = generate3DEnvironment(bounds, num_obstacles, Name, Value)
%
% 输入:
%   bounds: [xmin xmax ymin ymax zmin zmax] - 环境边界
%   num_obstacles: 障碍物数量
%
% 可选参数 (Name-Value对):
%   'StartPoint': [x y z] - 起点坐标
%   'GoalPoint': [x y z] - 终点坐标
%   'RadiusRange': [min max] - 障碍物半径范围 (默认: [1.0 3.0])
%   'MinSpacing': 标量 - 障碍物之间最小间距 (默认: 2.0)
%   'Clearance': 标量 - 起点终点与障碍物的最小间距 (默认: 3.0)
%   'Visualize': true/false - 是否立即可视化 (默认: false)
%   'SaveFile': 字符串 - 保存文件名 (可选)
%
% 输出:
%   env: 环境结构体,包含障碍物信息和边界
%
% 示例:
%   % 简单生成 (1500x1500x1500环境)
%   env = generate3DEnvironment([0 1500 0 1500 0 1500], 50);
%
%   % 完整参数 (1500x1500x1500环境)
%   env = generate3DEnvironment([0 1500 0 1500 0 1500], 50, ...
%       'StartPoint', [150 150 150], ...
%       'GoalPoint', [1350 1350 1350], ...
%       'RadiusRange', [20 50], ...
%       'MinSpacing', 35, ...
%       'Visualize', true, ...
%       'SaveFile', 'maps/my_env_3d.mat');
%
% 作者: AI Assistant
% 日期: 2025-12-11

% 解析输入参数
p = inputParser;
addRequired(p, 'bounds', @(x) isnumeric(x) && length(x) == 6);
addRequired(p, 'num_obstacles', @(x) isnumeric(x) && x > 0);
addParameter(p, 'StartPoint', [bounds(1)+75, bounds(3)+75, bounds(5)+75], @(x) isnumeric(x) && length(x) == 3);
addParameter(p, 'GoalPoint', [bounds(2)-75, bounds(4)-75, bounds(6)-75], @(x) isnumeric(x) && length(x) == 3);
addParameter(p, 'RadiusRange', [15.0, 45.0], @(x) isnumeric(x) && length(x) == 2);
addParameter(p, 'MinSpacing', 30.0, @isnumeric);
addParameter(p, 'Clearance', 45.0, @isnumeric);
addParameter(p, 'Visualize', false, @islogical);
addParameter(p, 'SaveFile', '', @ischar);

parse(p, bounds, num_obstacles, varargin{:});

% 构建选项结构体
options = struct();
options.start_point = p.Results.StartPoint;
options.goal_point = p.Results.GoalPoint;
options.radius_range = p.Results.RadiusRange;
options.min_spacing = p.Results.MinSpacing;
options.clearance = p.Results.Clearance;

% 调用主生成函数
env = EnvironmentConfig.generate3DEnvironment(bounds, num_obstacles, options);

% 可视化
if p.Results.Visualize
    EnvironmentConfig.visualize3D(env);
end

% 保存文件
if ~isempty(p.Results.SaveFile)
    EnvironmentConfig.saveEnvironment(env, p.Results.SaveFile);
end

end
