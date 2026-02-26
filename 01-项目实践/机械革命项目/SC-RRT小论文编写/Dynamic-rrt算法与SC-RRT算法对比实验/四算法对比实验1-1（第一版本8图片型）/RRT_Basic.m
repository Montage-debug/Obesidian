function [path, tree, success] = RRT_Basic(env, max_iter, step_size, goal_threshold)
% RRT_Basic - 原生RRT算法实现(基础版本)
% 适配新的统一环境配置系统
%
% 输入:
%   env: 环境结构体 (由EnvironmentConfig生成)
%   max_iter: 最大迭代次数 (默认: 5000)
%   step_size: 步长 (默认: 5)
%   goal_threshold: 目标阈值 (默认: 5)
%
% 输出:
%   path: 规划的路径 [N×2] (2D) 或 [N×3] (3D)
%   tree: 树结构体
%   success: 是否成功找到路径
%
% 特点:
%   - 原生RRT算法,不包含任何优化
%   - 支持2D和3D环境
%   - 目标偏向采样(20%概率)
%   - 简单的碰撞检测
%
% 作者: AI Assistant
% 日期: 2025-12-11

%% 参数设置
if nargin < 2, max_iter = 5000; end
if nargin < 3, step_size = 5; end
if nargin < 4, goal_threshold = 5; end

% 确定维度
is_2d = strcmp(env.dimension, '2D');
if is_2d
    dim = 2;
else
    dim = 3;
end

% 初始化树
tree = struct();
tree.vertices = env.start_point(:)';  % 节点位置 [N×dim]
tree.parent = 0;                       % 父节点索引
tree.cost = 0;                         % 从起点到节点的代价
node_count = 1;

% 距离计算函数
calcDist = @(p1, p2) norm(p1 - p2);

% 采样概率
goal_sample_rate = 0.2;

% 性能度量：收敛时间追踪
first_solution_found = false;
convergence_time = inf;

%% 主循环
tic;  % 开始计时
success = false;
goal_node_idx = -1;

fprintf('开始RRT路径规划...\n');
fprintf('  维度: %s\n', env.dimension);
fprintf('  起点: [%s]\n', num2str(env.start_point, '%.1f '));
fprintf('  终点: [%s]\n', num2str(env.goal_point, '%.1f '));
fprintf('  最大迭代: %d\n', max_iter);

for iter = 1:max_iter
    %% 1. 随机采样
    if rand() < goal_sample_rate
        % 目标偏向采样
        rand_point = env.goal_point;
    else
        % 随机采样
        if is_2d
            rand_point = [env.bounds(1) + rand()*(env.bounds(2)-env.bounds(1)), ...
                         env.bounds(3) + rand()*(env.bounds(4)-env.bounds(3))];
        else
            rand_point = [env.bounds(1) + rand()*(env.bounds(2)-env.bounds(1)), ...
                         env.bounds(3) + rand()*(env.bounds(4)-env.bounds(3)), ...
                         env.bounds(5) + rand()*(env.bounds(6)-env.bounds(5))];
        end
    end
    
    %% 2. 找最近节点
    [near_node, near_idx] = findNearestNode(tree, rand_point, node_count);
    
    %% 3. 扩展新节点
    new_node = steer(near_node, rand_point, step_size);
    
    %% 4. 碰撞检测
    if checkCollision(env, near_node, new_node)
        continue;  % 有碰撞,跳过
    end
    
    %% 5. 添加新节点
    node_count = node_count + 1;
    tree.vertices(node_count, :) = new_node;
    tree.parent(node_count) = near_idx;
    tree.cost(node_count) = tree.cost(near_idx) + calcDist(near_node, new_node);
    
    %% 6. 检查是否到达目标
    dist_to_goal = calcDist(new_node, env.goal_point);
    if dist_to_goal < goal_threshold
        % 最后连接到目标点
        if ~checkCollision(env, new_node, env.goal_point)
            node_count = node_count + 1;
            tree.vertices(node_count, :) = env.goal_point;
            tree.parent(node_count) = node_count - 1;
            tree.cost(node_count) = tree.cost(node_count-1) + dist_to_goal;
            
            goal_node_idx = node_count;
            success = true;
            
            % 记录首次可行解的收敛时间
            if ~first_solution_found
                convergence_time = toc;
                first_solution_found = true;
            end
            
            fprintf('✓ 成功找到路径! 迭代次数: %d, 收敛时间: %.3fs\n', iter, convergence_time);
            break;
        end
    end
    
    % 进度显示
    if mod(iter, 500) == 0
        fprintf('  迭代 %d/%d, 节点数: %d\n', iter, max_iter, node_count);
    end
end

%% 提取路径
planning_time = toc;  % 总计算时间

if success
    path = extractPath(tree, goal_node_idx);
    % 统一性能度量标准
    tree.path_cost = tree.cost(goal_node_idx);  % 路径代价（累计距离）
    tree.path_length = tree.path_cost;  % 路径长度（同义）
    tree.final_cost = tree.path_cost;  % 向后兼容
    tree.convergence_time = convergence_time;  % 首次可行解时间
    tree.planning_time = planning_time;  % 总计算时间
    tree.goal_node_idx = goal_node_idx;
    tree.success = true;
    fprintf('✓ 路径代价: %.2f, 节点数: %d, 收敛时间: %.3fs, 总时间: %.3fs\n', ...
            tree.path_cost, size(path, 1), convergence_time, planning_time);
else
    path = [];
    tree.path_cost = inf;
    tree.path_length = inf;
    tree.final_cost = inf;
    tree.convergence_time = inf;
    tree.planning_time = planning_time;
    tree.goal_node_idx = -1;
    tree.success = false;
    fprintf('✗ 未找到路径, 总时间: %.3fs\n', planning_time);
end

end

%% 辅助函数

function [nearest_node, nearest_idx] = findNearestNode(tree, point, node_count)
    % 找到树中距离给定点最近的节点
    min_dist = inf;
    nearest_idx = 1;
    
    for i = 1:node_count
        dist = norm(tree.vertices(i, :) - point);
        if dist < min_dist
            min_dist = dist;
            nearest_idx = i;
        end
    end
    
    nearest_node = tree.vertices(nearest_idx, :);
end

function new_node = steer(from_node, to_node, step_size)
    % 从from_node向to_node扩展step_size距离
    direction = to_node - from_node;
    dist = norm(direction);
    
    if dist <= step_size
        new_node = to_node;
    else
        new_node = from_node + (direction / dist) * step_size;
    end
end

function collision = checkCollision(env, from_point, to_point)
    % 检查从from_point到to_point的路径是否与障碍物碰撞
    collision = false;
    
    % 检查点数量(路径采样)
    num_checks = ceil(norm(to_point - from_point) / 0.5);  % 每0.5距离检查一次
    num_checks = max(num_checks, 2);
    
    if strcmp(env.dimension, '2D')
        % 2D碰撞检测
        for i = 0:num_checks
            t = i / num_checks;
            check_point = from_point * (1-t) + to_point * t;
            
            % 检查与每个障碍物的碰撞
            for j = 1:env.num
                obs_center = env.obstacles(j, 1:2);
                obs_radius = env.obstacles(j, 3);
                
                if norm(check_point - obs_center) < obs_radius
                    collision = true;
                    return;
                end
            end
        end
    else
        % 3D碰撞检测
        for i = 0:num_checks
            t = i / num_checks;
            check_point = from_point * (1-t) + to_point * t;
            
            % 检查与每个障碍物的碰撞
            for j = 1:env.num
                obs_center = env.obstacles(j, 1:3);
                obs_radius = env.obstacles(j, 4);
                
                if norm(check_point - obs_center) < obs_radius
                    collision = true;
                    return;
                end
            end
        end
    end
end

function path = extractPath(tree, goal_idx)
    % 从目标节点回溯到起点,提取路径
    path = [];
    current_idx = goal_idx;
    
    while current_idx ~= 0
        path = [tree.vertices(current_idx, :); path];
        current_idx = tree.parent(current_idx);
    end
end
