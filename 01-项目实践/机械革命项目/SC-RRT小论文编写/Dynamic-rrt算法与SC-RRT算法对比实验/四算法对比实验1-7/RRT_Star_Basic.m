function [path, tree, success] = RRT_Star_Basic(env, max_iter, step_size, goal_threshold)
% RRT_Star_Basic - RRT*算法实现(基础版本)
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
%   - RRT*算法,包含重布线优化
%   - 支持2D和3D环境
%   - 目标偏向采样(20%概率)
%   - 简单的碰撞检测
%   - 邻域搜索和父节点优化
%
% 作者: AI Assistant
% 日期: 2025-12-11

%% 参数设置
if nargin < 2, max_iter = 5000; end

% 确定维度
if ischar(env.dimension) || isstring(env.dimension)
    is_2d = strcmpi(env.dimension, '2D');
else
    is_2d = (env.dimension == 2);
end
if is_2d
    dim = 2;
else
    dim = 3;
end

% 确保起点和终点为行向量
if size(env.start_point, 1) > 1
    env.start_point = env.start_point';
end
if size(env.goal_point, 1) > 1
    env.goal_point = env.goal_point';
end

% 自适应步长和目标阈值 (根据环境大小)
env_diag = norm(env.goal_point - env.start_point);
if nargin < 3 || isempty(step_size)
    step_size = env_diag / 50;  % 自适应: 约为起终点距离的2%
end
if nargin < 4 || isempty(goal_threshold)
    goal_threshold = step_size * 1.5;
end

% 初始化树
tree = struct();
tree.vertices = env.start_point(:)';  % 节点位置 [N×dim]
tree.parent = 0;                       % 父节点索引
tree.cost = 0;                         % 从起点到节点的代价
node_count = 1;

% RRT*特有参数 - 自适应gamma
% 理论公式: gamma > (2*(1+1/d))^(1/d) * (mu(X_free)/zeta_d)^(1/d)
% 简化为环境对角线的比例
gamma_rrt_star = env_diag * 2;
rewire_count = 0;     % 重布线次数

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

fprintf('开始RRT*路径规划...\n');
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
    
    %% 5. RRT*特有: 寻找邻域节点
    % 计算邻域半径 (随着节点数增加而减小)
    % RRT*理论公式: r_n = gamma * (log(n)/n)^(1/d)
    radius = min(gamma_rrt_star * (log(node_count+1)/(node_count+1))^(1/dim), step_size * 3);
    neighbors = findNeighbors(tree, new_node, radius, node_count);
    
    %% 6. RRT*特有: 选择最优父节点
    min_cost = tree.cost(near_idx) + calcDist(near_node, new_node);
    min_parent_idx = near_idx;
    
    for i = 1:length(neighbors)
        nb_idx = neighbors(i);
        nb_node = tree.vertices(nb_idx, :);
        
        % 计算通过这个邻居节点的代价
        tentative_cost = tree.cost(nb_idx) + calcDist(nb_node, new_node);
        
        % 如果代价更小且路径无碰撞,选择这个邻居作为父节点
        if tentative_cost < min_cost && ~checkCollision(env, nb_node, new_node)
            min_cost = tentative_cost;
            min_parent_idx = nb_idx;
        end
    end
    
    %% 7. 添加新节点
    node_count = node_count + 1;
    tree.vertices(node_count, :) = new_node;
    tree.parent(node_count) = min_parent_idx;
    tree.cost(node_count) = min_cost;
    
    %% 8. RRT*特有: 重布线 (Rewire)
    % 检查新节点是否能作为邻居节点更好的父节点
    for i = 1:length(neighbors)
        nb_idx = neighbors(i);
        nb_node = tree.vertices(nb_idx, :);
        
        % 计算通过新节点到达邻居节点的代价
        new_cost = tree.cost(node_count) + calcDist(new_node, nb_node);
        
        % 如果代价更小且路径无碰撞,重布线
        if new_cost < tree.cost(nb_idx) && ~checkCollision(env, new_node, nb_node)
            tree.parent(nb_idx) = node_count;
            tree.cost(nb_idx) = new_cost;
            rewire_count = rewire_count + 1;
            
            % 递归更新该邻居节点的所有子节点的代价
            updateChildrenCost(tree, nb_idx, node_count);
        end
    end
    
    %% 9. 检查是否到达目标
    dist_to_goal = calcDist(new_node, env.goal_point);
    if dist_to_goal < goal_threshold
        % 最后连接到目标点
        if ~checkCollision(env, new_node, env.goal_point)
            % 检查是否已经有到达目标的路径
            if goal_node_idx > 0
                % 如果新路径代价更小,更新
                new_goal_cost = tree.cost(node_count) + dist_to_goal;
                if new_goal_cost < tree.cost(goal_node_idx)
                    tree.parent(goal_node_idx) = node_count;
                    tree.cost(goal_node_idx) = new_goal_cost;
                end
            else
                % 第一次到达目标
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
                
                fprintf('✓ 首次找到路径! 迭代次数: %d, 代价: %.2f, 收敛时间: %.3fs\n', ...
                        iter, tree.cost(goal_node_idx), convergence_time);
            end
        end
    end
    
    % 进度显示
    if mod(iter, 500) == 0
        if success
            fprintf('  迭代 %d/%d, 节点数: %d, 当前最优代价: %.2f, 重布线: %d次\n', ...
                    iter, max_iter, node_count, tree.cost(goal_node_idx), rewire_count);
        else
            fprintf('  迭代 %d/%d, 节点数: %d, 重布线: %d次\n', ...
                    iter, max_iter, node_count, rewire_count);
        end
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
    tree.rewire_count = rewire_count;
    tree.success = true;
    fprintf('✓ 最终路径代价: %.2f, 路径节点数: %d, 树节点数: %d\n', ...
            tree.path_cost, size(path, 1), node_count);
    fprintf('  收敛时间: %.3fs, 总时间: %.3fs, 重布线: %d次\n', ...
            convergence_time, planning_time, rewire_count);
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
    % 找到树中距离给定点最近的节点 (向量化)
    dists = vecnorm(tree.vertices(1:node_count, :) - point, 2, 2);
    [~, nearest_idx] = min(dists);
    nearest_node = tree.vertices(nearest_idx, :);
end

function neighbors = findNeighbors(tree, point, radius, node_count)
    % 找到给定点半径范围内的所有节点 (向量化)
    dists = vecnorm(tree.vertices(1:node_count, :) - point, 2, 2);
    neighbors = find(dists <= radius)';
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
    % 检查从from_point到to_point的路径是否与障碍物碰撞 (向量化)
    collision = false;
    
    dist = norm(to_point - from_point);
    num_checks = max(2, ceil(dist / 2));  % 自适应检查密度
    direction = to_point - from_point;
    
    if strcmp(env.dimension, '2D')
        obs_centers = env.obstacles(:, 1:2);
        obs_radii = env.obstacles(:, 3);
        
        for i = 0:num_checks
            t = i / num_checks;
            check_point = from_point + t * direction;
            dists = vecnorm(obs_centers - check_point, 2, 2);
            if any(dists < obs_radii)
                collision = true;
                return;
            end
        end
    else
        obs_centers = env.obstacles(:, 1:3);
        obs_radii = env.obstacles(:, 4);
        
        for i = 0:num_checks
            t = i / num_checks;
            check_point = from_point + t * direction;
            dists = vecnorm(obs_centers - check_point, 2, 2);
            if any(dists < obs_radii)
                collision = true;
                return;
            end
        end
    end
end

function updateChildrenCost(tree, node_idx, new_parent_idx)
    % 递归更新节点的所有子节点的代价
    % BFS遍历更新整个子树
    queue = [];
    % 找到node_idx的所有子节点
    for i = 1:length(tree.parent)
        if tree.parent(i) == node_idx
            queue = [queue, i];
        end
    end
    
    while ~isempty(queue)
        current = queue(1);
        queue(1) = [];
        
        parent_idx = tree.parent(current);
        if parent_idx > 0
            tree.cost(current) = tree.cost(parent_idx) + ...
                norm(tree.vertices(current,:) - tree.vertices(parent_idx,:));
        end
        
        % 找当前节点的子节点
        for i = 1:length(tree.parent)
            if tree.parent(i) == current
                queue = [queue, i];
            end
        end
    end
end

function path = extractPath(tree, goal_idx)
    % 从目标节点回溯到起点,提取路径
    path = [];
    current_idx = goal_idx;
    
    while current_idx > 0
        path = [tree.vertices(current_idx, :); path];
        current_idx = tree.parent(current_idx);
    end
end
