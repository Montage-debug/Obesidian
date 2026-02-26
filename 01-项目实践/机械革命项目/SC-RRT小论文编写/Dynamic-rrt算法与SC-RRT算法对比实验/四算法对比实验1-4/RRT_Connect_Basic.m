function [path, tree, success] = RRT_Connect_Basic(env, max_iter, step_size, goal_threshold)
% RRT_Connect_Basic - 标准RRT-Connect算法实现
% 
% RRT-Connect是一种双向快速探索随机树算法，具有以下特点：
%   1. 双向生长：从起点和终点同时生长两棵树
%   2. Connect操作：不仅扩展一步，而且尝试连接到采样点
%   3. 快速收敛：通过双向搜索和连接策略加快规划速度
%   4. 原生实现：标准RRT-Connect算法，无额外优化
%
% 输入:
%   env: 环境结构体 (由EnvironmentConfig生成)
%   max_iter: 最大迭代次数 (默认: 5000)
%   step_size: 步长 (默认: 5)
%   goal_threshold: 连接阈值 (默认: 5)
%
% 输出:
%   path: 规划的路径 [N×2] (2D) 或 [N×3] (3D)
%   tree: 树结构体
%   success: 是否成功找到路径
%
% 算法流程:
%   1. 初始化两棵树 (Ta从起点, Tb从终点)
%   2. 随机采样一个点
%   3. 将Ta向采样点扩展 (Extend操作)
%   4. 尝试将Tb连接到Ta的新节点 (Connect操作)
%   5. 如果连接成功，返回路径；否则交换两棵树
%   6. 重复直到找到路径或达到最大迭代次数
%
% 作者: GitHub Copilot
% 日期: 2025-12-11

%% 参数设置
if nargin < 2, max_iter = 5000; end

% 确定维度
is_2d = strcmp(env.dimension, '2D');
if is_2d
    dim = 2;
else
    dim = 3;
end

% 自适应步长和目标阈值
env_diag = norm(env.goal_point - env.start_point);
if nargin < 3 || isempty(step_size)
    step_size = env_diag / 50;
end
if nargin < 4 || isempty(goal_threshold)
    goal_threshold = step_size;
end

% 初始化树A (从起点生长)
treeA = struct();
treeA.vertices = env.start_point(:)';
treeA.parent = 0;
treeA.cost = 0;
node_count_A = 1;

% 初始化树B (从终点生长)
treeB = struct();
treeB.vertices = env.goal_point(:)';
treeB.parent = 0;
treeB.cost = 0;
node_count_B = 1;

% 距离计算函数
calcDist = @(p1, p2) norm(p1 - p2);

%% 主循环
success = false;
connect_node_A = -1;
connect_node_B = -1;

fprintf('开始RRT-Connect路径规划...\n');
fprintf('  维度: %s\n', env.dimension);
fprintf('  起点: [%s]\n', num2str(env.start_point, '%.1f '));
fprintf('  终点: [%s]\n', num2str(env.goal_point, '%.1f '));
fprintf('  最大迭代: %d\n', max_iter);

for iter = 1:max_iter
    %% 1. 随机采样
    if is_2d
        rand_point = [env.bounds(1) + rand()*(env.bounds(2)-env.bounds(1)), ...
                     env.bounds(3) + rand()*(env.bounds(4)-env.bounds(3))];
    else
        rand_point = [env.bounds(1) + rand()*(env.bounds(2)-env.bounds(1)), ...
                     env.bounds(3) + rand()*(env.bounds(4)-env.bounds(3)), ...
                     env.bounds(5) + rand()*(env.bounds(6)-env.bounds(5))];
    end
    
    %% 2. Extend操作: 将树A向采样点扩展
    [treeA, node_count_A, new_node, extended] = extendTree(treeA, node_count_A, rand_point, ...
                                                           env, step_size);
    
    if ~extended
        continue;  % 扩展失败，继续下一次迭代
    end
    
    %% 3. Connect操作: 尝试将树B连接到树A的新节点
    [treeB, node_count_B, reached] = connectTree(treeB, node_count_B, new_node, ...
                                                  env, step_size, goal_threshold);
    
    %% 4. 检查是否连接成功
    if reached
        % 找到连接点
        connect_node_A = node_count_A;  % 树A的新节点
        
        % 树B中最接近new_node的节点
        distances = zeros(node_count_B, 1);
        for i = 1:node_count_B
            distances(i) = calcDist(treeB.vertices(i, :), new_node);
        end
        [~, connect_node_B] = min(distances);
        
        success = true;
        fprintf('✓ 成功找到路径! 迭代次数: %d\n', iter);
        break;
    end
    
    %% 5. 交换两棵树 (让两棵树均衡生长)
    temp = treeA;
    treeA = treeB;
    treeB = temp;
    
    temp_count = node_count_A;
    node_count_A = node_count_B;
    node_count_B = temp_count;
    
    % 进度显示
    if mod(iter, 500) == 0
        fprintf('  迭代 %d/%d, 树A节点: %d, 树B节点: %d\n', ...
                iter, max_iter, node_count_A, node_count_B);
    end
end

%% 提取路径
if success
    % 提取从起点到连接点A的路径
    pathA = extractPath(treeA, connect_node_A);
    
    % 提取从连接点B到终点的路径
    pathB = extractPath(treeB, connect_node_B);
    
    % 反转pathB并合并
    pathB = flipud(pathB);
    path = [pathA; pathB];
    
    % 计算路径长度
    path_length = 0;
    for i = 1:size(path, 1)-1
        path_length = path_length + calcDist(path(i, :), path(i+1, :));
    end
    
    % 合并两棵树用于可视化
    tree = struct();
    tree.vertices = [treeA.vertices; treeB.vertices];
    tree.parent = [treeA.parent(:); treeB.parent(:) + node_count_A];
    tree.parent(node_count_A + 1) = 0;  % 树B的根节点
    tree.cost = zeros(node_count_A + node_count_B, 1);
    tree.final_cost = path_length;
    tree.goal_node_idx = size(path, 1);
    
    fprintf('✓ 路径长度: %.2f, 路径节点数: %d, 总树节点数: %d\n', ...
            path_length, size(path, 1), node_count_A + node_count_B);
else
    path = [];
    
    % 即使失败也合并树结构
    tree = struct();
    tree.vertices = [treeA.vertices; treeB.vertices];
    tree.parent = [treeA.parent(:); treeB.parent(:) + node_count_A];
    tree.parent(node_count_A + 1) = 0;
    tree.cost = zeros(node_count_A + node_count_B, 1);
    tree.final_cost = inf;
    tree.goal_node_idx = -1;
    
    fprintf('✗ 未找到路径\n');
end

end

%% ========== 辅助函数 ==========

function [tree, node_count, new_node, extended] = extendTree(tree, node_count, target_point, ...
                                                              env, step_size)
    % Extend操作: 向目标点扩展一步
    extended = false;
    new_node = [];
    
    % 找最近节点
    [nearest_node, nearest_idx] = findNearestNode(tree, target_point, node_count);
    
    % Steer: 从最近节点向目标点移动
    new_node = steer(nearest_node, target_point, step_size);
    
    % 碰撞检测
    if checkCollision(env, nearest_node, new_node)
        return;  % 有碰撞，扩展失败
    end
    
    % 添加新节点
    node_count = node_count + 1;
    tree.vertices(node_count, :) = new_node;
    tree.parent(node_count) = nearest_idx;
    tree.cost(node_count) = tree.cost(nearest_idx) + norm(new_node - nearest_node);
    
    extended = true;
end

function [tree, node_count, reached] = connectTree(tree, node_count, target_point, ...
                                                    env, step_size, threshold)
    % Connect操作: 尝试连接到目标点
    % 不断向目标点扩展，直到到达或发生碰撞
    reached = false;
    
    max_connect_steps = 100;  % 防止无限循环
    
    for step = 1:max_connect_steps
        % 找最近节点
        [nearest_node, nearest_idx] = findNearestNode(tree, target_point, node_count);
        
        % 检查是否已经足够接近
        dist = norm(nearest_node - target_point);
        if dist < threshold
            reached = true;
            return;
        end
        
        % Steer: 向目标点移动
        new_node = steer(nearest_node, target_point, step_size);
        
        % 碰撞检测
        if checkCollision(env, nearest_node, new_node)
            return;  % 碰撞，连接失败
        end
        
        % 添加新节点
        node_count = node_count + 1;
        tree.vertices(node_count, :) = new_node;
        tree.parent(node_count) = nearest_idx;
        tree.cost(node_count) = tree.cost(nearest_idx) + norm(new_node - nearest_node);
        
        % 检查是否已到达目标点
        if norm(new_node - target_point) < threshold
            reached = true;
            return;
        end
    end
end

function [nearest_node, nearest_idx] = findNearestNode(tree, point, node_count)
    % 找到树中距离给定点最近的节点 (向量化)
    dists = vecnorm(tree.vertices(1:node_count, :) - point, 2, 2);
    [~, nearest_idx] = min(dists);
    nearest_node = tree.vertices(nearest_idx, :);
end

function new_node = steer(from_node, to_node, step_size)
    % 从from_node向to_node扩展step_size距离
    direction = to_node - from_node;
    dist = norm(direction);
    
    if dist <= step_size
        new_node = to_node;
    else
        new_node = from_node + (direction ./ dist) .* step_size;
    end
end

function collision = checkCollision(env, from_point, to_point)
    % 检查从from_point到to_point的路径是否与障碍物碰撞 (向量化)
    collision = false;
    
    dist = norm(to_point - from_point);
    num_checks = max(2, ceil(dist / 2));
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

function path = extractPath(tree, goal_idx)
    % 从目标节点回溯到起点,提取路径
    % 先收集所有节点索引,然后一次性提取
    idx_list = [];
    current_idx = goal_idx;
    
    while current_idx > 0
        idx_list = [current_idx; idx_list];
        current_idx = tree.parent(current_idx);
    end
    
    % 提取路径点
    if isempty(idx_list)
        path = [];
    else
        path = tree.vertices(idx_list, :);
    end
end
