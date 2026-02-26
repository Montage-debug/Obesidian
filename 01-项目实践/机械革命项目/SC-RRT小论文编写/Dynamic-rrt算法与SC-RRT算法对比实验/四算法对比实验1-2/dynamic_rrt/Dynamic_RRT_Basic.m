function [path, tree_out, success, metrics] = Dynamic_RRT_Basic(env, max_iterations)
% Dynamic_RRT_Basic - Dynamic-RRT算法封装接口
%
% 适配四算法对比框架的统一接口
% 论文: Dynamic RRT: Fast Feasible Path Planning in Randomly Distributed
%       Obstacle Environments (2023)
%
% 输入:
%   env            - 环境结构体 (来自generate2DEnvironment或generate3DEnvironment)
%                    必须包含: bounds, start_point, goal_point, obstacles, dimension
%   max_iterations - 最大迭代次数
%
% 输出:
%   path     - 规划路径 [N x m]
%   tree_out - 树结构 (统一格式: .vertices, .parent, .cost, .final_cost)
%   success  - 是否规划成功
%   metrics  - 性能指标结构体

    %% ========== 提取环境参数 ==========
    startPoint = env.start_point;
    goalPoint  = env.goal_point;
    bounds     = env.bounds;

    % 判断维度
    if ischar(env.dimension) || isstring(env.dimension)
        if contains(env.dimension, '2D') || contains(env.dimension, '2d')
            m = 2;
        else
            m = 3;
        end
    else
        m = env.dimension;
    end

    %% ========== 转换障碍物格式 ==========
    % 对比框架使用平坦矩阵 [N x 3] (2D) 或 [N x 4] (3D)
    % DynamicRRT 使用结构体 obstacles.circles / obstacles.spheres
    obstacles_struct = struct();
    if m == 2
        obstacles_struct.circles = env.obstacles;  % [N x 3]: [cx, cy, r]
    else
        obstacles_struct.spheres = env.obstacles;  % [N x 4]: [cx, cy, cz, r]
    end

    %% ========== 调用核心算法 ==========
    tic;
    [tree, raw_path, success, raw_metrics] = DynamicRRT(...
        startPoint, goalPoint, bounds, obstacles_struct, ...
        'MaxIterations', max_iterations, ...
        'Interval', 4, ...
        'ParetoProb', 0.1, ...
        'EnableVisualization', false);
    planning_time = toc;

    %% ========== 转换输出格式 ==========
    % 转换树结构为统一格式
    tree_out = struct();
    tree_out.vertices = tree.nodes(1:tree.count, :);
    tree_out.parent   = tree.parents(1:tree.count)';
    tree_out.cost     = tree.costs(1:tree.count)';

    if success && ~isempty(raw_path)
        path = raw_path;

        % 计算路径长度
        pathLength = 0;
        for i = 1:size(path, 1) - 1
            pathLength = pathLength + norm(path(i+1, :) - path(i, :));
        end
        tree_out.final_cost = pathLength;
    else
        path = [];
        tree_out.final_cost = inf;
    end

    %% ========== 构建统一metrics ==========
    metrics = struct();
    metrics.planning_time = planning_time;
    metrics.tree_nodes    = tree.count;
    metrics.iterations    = max_iterations;

    if success && ~isempty(path)
        metrics.path_length = tree_out.final_cost;
    else
        metrics.path_length = inf;
    end

    % 计算平滑度
    metrics.smoothness = calculateSmoothness(path);
    metrics.clearance  = NaN;

    % 保留原始metrics中的额外信息
    if isfield(raw_metrics, 'convergenceTime')
        metrics.convergence_time = raw_metrics.convergenceTime;
    end
    if isfield(raw_metrics, 'interval')
        metrics.interval = raw_metrics.interval;
    end
end

%% ========== 辅助函数 ==========
function smoothness = calculateSmoothness(path)
    if isempty(path) || size(path, 1) < 3
        smoothness = NaN;
        return;
    end
    angles = [];
    for i = 2:size(path, 1) - 1
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        cos_angle = dot(v1, v2) / (norm(v1) * norm(v2) + eps);
        cos_angle = max(-1, min(1, cos_angle));
        angles = [angles; acos(cos_angle)]; %#ok<AGROW>
    end
    smoothness = std(angles);
end
