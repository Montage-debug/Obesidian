function [path, tree, success, metrics] = Informed_RRT_Star_Basic(env, max_iterations)
% Informed_RRT_Star_Basic - Informed RRT* 算法实现
%
% 基于 Gammell et al. (2014) "Informed RRT*: Optimal Sampling-based Path
% Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic"
%
% 核心思想：
%   在找到初始可行解后，将采样空间限制在以起终点为焦点的超椭球体内，
%   仅对能够改善当前最优路径代价的区域进行采样。
%
% 输入:
%   env             - 环境结构体 (来自EnvironmentConfig)
%   max_iterations  - 最大迭代次数
%
% 输出:
%   path     - 规划路径 [N×dim]
%   tree     - 树结构体
%   success  - 是否成功
%   metrics  - 性能指标

if nargin < 2, max_iterations = 5000; end

%% ========== 环境解析 ==========
if isfield(env, 'start_point')
    start_point = env.start_point;
elseif isfield(env, 'start')
    start_point = env.start;
else
    error('缺少起点字段');
end

if isfield(env, 'goal_point')
    goal_point = env.goal_point;
elseif isfield(env, 'goal')
    goal_point = env.goal;
else
    error('缺少终点字段');
end

obstacles = env.obstacles;
dim_str = env.dimension;
bounds = env.bounds;

if ischar(dim_str) || isstring(dim_str)
    if contains(dim_str, '2')
        dim = 2;
    else
        dim = 3;
    end
else
    dim = dim_str;
end

% 自动参数
if dim == 2
    env_size = max(bounds(2)-bounds(1), bounds(4)-bounds(3));
else
    env_size = max([bounds(2)-bounds(1), bounds(4)-bounds(3), bounds(6)-bounds(5)]);
end

step_size = max(12, min(80, env_size * 0.010));
goal_threshold = step_size * 1.5;
goal_bias = 0.05;  % 5%目标偏向

% RRT*参数
gamma_rrt_star = 2.0 * (1 + 1/dim)^(1/dim) * (env_size / 1.0)^(1/dim);

fprintf('Informed-RRT*开始 (%dD, step=%.1f)\n', dim, step_size);

%% ========== 初始化树 ==========
capacity = min(max_iterations, 50000);
tree_nodes = zeros(capacity, dim);
tree_parent = zeros(capacity, 1);
tree_cost = zeros(capacity, 1);

tree_nodes(1, :) = start_point;
tree_parent(1) = 0;
tree_cost(1) = 0;
n_nodes = 1;

%% ========== 算法状态 ==========
success = false;
path = [];
c_best = inf;        % 当前最优路径代价
goal_idx = -1;       % 目标节点索引
first_solution_iter = inf;

% Informed采样参数
c_min = norm(goal_point - start_point);  % 焦距（最小可能代价）
x_center = (start_point + goal_point) / 2;

% 旋转矩阵：将标准椭球旋转到起终点连线方向
a1 = (goal_point - start_point) / c_min;
if dim == 2
    M = [a1; [-a1(2), a1(1)]];
else
    % 3D: 使用Gram-Schmidt或Rodrigues旋转
    if abs(a1(1)) < 0.9
        e1 = [1, 0, 0];
    else
        e1 = [0, 1, 0];
    end
    a2 = e1 - dot(e1, a1) * a1; a2 = a2 / norm(a2);
    a3 = cross(a1, a2);
    M = [a1; a2; a3];
end
C = M';  % 旋转矩阵

bounds_lo = bounds(1:2:end);
bounds_hi = bounds(2:2:end);

tic;

%% ========== 主循环 ==========
for iter = 1:max_iterations
    
    %% ===== 采样 =====
    if ~isinf(c_best)
        % Informed采样：在超椭球体内采样
        sample = sampleInformedEllipsoid(c_best, c_min, x_center, C, dim, bounds_lo, bounds_hi);
    else
        % 标准RRT*采样 + 目标偏向
        if rand < goal_bias
            sample = goal_point;
        else
            sample = bounds_lo + rand(1, dim) .* (bounds_hi - bounds_lo);
        end
    end
    
    %% ===== 最近节点 =====
    diffs = bsxfun(@minus, tree_nodes(1:n_nodes, :), sample);
    dists = sum(diffs .* diffs, 2);
    [~, nearest_idx] = min(dists);
    nearest = tree_nodes(nearest_idx, :);
    
    %% ===== Steer =====
    dir_vec = sample - nearest;
    dist = sqrt(dir_vec * dir_vec');
    if dist > step_size && dist > 1e-6
        new_point = nearest + (dir_vec / dist) * step_size;
    elseif dist > 1e-6
        new_point = sample;
    else
        continue;
    end
    
    %% ===== 碰撞检测 =====
    if ~isCollisionFreeLocal(nearest, new_point, obstacles, dim)
        continue;
    end
    
    %% ===== 邻域搜索 + 最优父节点选择 =====
    r_n = min(gamma_rrt_star * (log(n_nodes) / n_nodes)^(1/dim), step_size * 3);
    diffs_nb = bsxfun(@minus, tree_nodes(1:n_nodes, :), new_point);
    dists_nb = sqrt(sum(diffs_nb .* diffs_nb, 2));
    near_indices = find(dists_nb <= r_n);
    
    best_cost = tree_cost(nearest_idx) + norm(new_point - nearest);
    best_parent = nearest_idx;
    
    for ni = near_indices(:)'
        if ni == nearest_idx, continue; end
        tentative = tree_cost(ni) + norm(new_point - tree_nodes(ni, :));
        if tentative < best_cost
            if isCollisionFreeLocal(tree_nodes(ni, :), new_point, obstacles, dim)
                best_cost = tentative;
                best_parent = ni;
            end
        end
    end
    
    %% ===== 添加节点 =====
    n_nodes = n_nodes + 1;
    if n_nodes > size(tree_nodes, 1)
        tree_nodes = [tree_nodes; zeros(size(tree_nodes, 1), dim)];
        tree_parent = [tree_parent; zeros(size(tree_parent, 1), 1)];
        tree_cost = [tree_cost; zeros(size(tree_cost, 1), 1)];
    end
    tree_nodes(n_nodes, :) = new_point;
    tree_parent(n_nodes) = best_parent;
    tree_cost(n_nodes) = best_cost;
    
    %% ===== 重布线 =====
    for ni = near_indices(:)'
        if ni == best_parent, continue; end
        new_cost_through = best_cost + norm(tree_nodes(ni, :) - new_point);
        if new_cost_through < tree_cost(ni)
            if isCollisionFreeLocal(new_point, tree_nodes(ni, :), obstacles, dim)
                tree_parent(ni) = n_nodes;
                old_cost = tree_cost(ni);
                tree_cost(ni) = new_cost_through;
                % 递归更新子节点代价
                updateChildCosts(ni);
            end
        end
    end
    
    %% ===== 检查是否到达目标 =====
    dist_to_goal = norm(new_point - goal_point);
    if dist_to_goal < goal_threshold
        if isCollisionFreeLocal(new_point, goal_point, obstacles, dim)
            % 计算经过这条路径到目标的代价
            path_cost_to_goal = best_cost + dist_to_goal;
            
            if path_cost_to_goal < c_best
                c_best = path_cost_to_goal;
                goal_idx = n_nodes;
                success = true;
                
                if isinf(first_solution_iter) || first_solution_iter == inf
                    first_solution_iter = iter;
                end
            end
        end
    end
end

planning_time = toc;

%% ========== 路径提取 ==========
if success && goal_idx > 0
    % 回溯最优路径
    path = goal_point;
    current = goal_idx;
    while current > 0
        path = [tree_nodes(current, :); path];
        current = tree_parent(current);
    end
end

%% ========== 构建输出 ==========
tree = struct();
tree.vertices = tree_nodes(1:n_nodes, :);
tree.nodes = tree.vertices;
tree.parent = tree_parent(1:n_nodes);
tree.parents = tree.parent;
tree.cost = tree_cost(1:n_nodes);
if success
    tree.final_cost = c_best;
else
    tree.final_cost = inf;
end
tree.convergence_time = planning_time * (first_solution_iter / max(iter, 1));

%% ========== 性能指标 ==========
metrics = struct();
metrics.algorithm_name = 'Informed_RRT_Star';
metrics.iterations = iter;
metrics.tree_nodes = n_nodes;
metrics.planning_time = planning_time;
metrics.success_rate = double(success);

if success
    metrics.convergence_time = tree.convergence_time;
    metrics.path_length = sum(vecnorm(diff(path), 2, 2));
    metrics.path_cost = c_best;
    if size(path, 1) >= 3
        angles = zeros(size(path,1)-2, 1);
        for i = 2:size(path,1)-1
            v1 = path(i,:) - path(i-1,:);
            v2 = path(i+1,:) - path(i,:);
            ca = dot(v1,v2) / (norm(v1)*norm(v2) + 1e-10);
            angles(i-1) = acos(max(-1, min(1, ca)));
        end
        metrics.smoothness = std(angles);
    else
        metrics.smoothness = 0;
    end
else
    metrics.convergence_time = inf;
    metrics.path_length = inf;
    metrics.path_cost = inf;
    metrics.smoothness = inf;
end

fprintf('Informed-RRT*完成: %s, iter=%d, nodes=%d, cost=%.1f, t=%.3fs\n', ...
    string(success), iter, n_nodes, c_best, planning_time);

%% ========== 嵌套函数 ==========
    function updateChildCosts(parent_idx)
        % 递归更新所有子节点的代价
        children = find(tree_parent(1:n_nodes) == parent_idx);
        for ci = children(:)'
            tree_cost(ci) = tree_cost(parent_idx) + ...
                norm(tree_nodes(ci, :) - tree_nodes(parent_idx, :));
            updateChildCosts(ci);
        end
    end

end

%% ========== 辅助函数 ==========

function sample = sampleInformedEllipsoid(c_best, c_min, x_center, C, dim, bounds_lo, bounds_hi)
    % 在超椭球体内均匀采样
    % 椭球长半轴 = c_best/2, 其余半轴 = sqrt(c_best^2 - c_min^2)/2

    r1 = c_best / 2;
    r_rest = sqrt(max(0, c_best^2 - c_min^2)) / 2;
    
    % 生成单位球内均匀采样
    if dim == 2
        % 在2D单位圆内均匀采样
        theta = 2 * pi * rand;
        r = sqrt(rand);
        x_ball = [r * cos(theta), r * sin(theta)];
    else
        % 在nD单位球内均匀采样 (Muller方法)
        x_ball = randn(1, dim);
        x_ball = x_ball / norm(x_ball) * rand^(1/dim);
    end
    
    % 缩放为椭球
    L = diag([r1, repmat(r_rest, 1, dim-1)]);
    sample = (C * L * x_ball')' + x_center;
    
    % 边界裁剪
    sample = max(bounds_lo, min(bounds_hi, sample));
end

function collision_free = isCollisionFreeLocal(from_point, to_point, obstacles, dim)
    collision_free = true;
    
    direction = to_point - from_point;
    dist = norm(direction);
    num_checks = max(2, ceil(dist / 2));
    
    if dim == 2
        obs_centers = obstacles(:, 1:2);
        obs_radii = obstacles(:, 3);
    else
        obs_centers = obstacles(:, 1:3);
        obs_radii = obstacles(:, 4);
    end
    
    for i = 0:num_checks
        t = i / num_checks;
        check_point = from_point + t * direction;
        dists = vecnorm(obs_centers - check_point, 2, 2);
        if any(dists < obs_radii)
            collision_free = false;
            return;
        end
    end
end
