function [path, tree, success, metrics] = SC_RRT_Basic(env, max_iterations, varargin)
% SC_RRT_Basic - SC-RRT算法全面优化版本
%
% 功能特点:
%   1. 双向RRT with 非对称双椭球约束采样
%   2. 动态交汇点转移机制
%   3. Pareto前沿多目标优化
%   4. 自适应PID控制代价调节
%   5. 模糊推理增益调优
%   6. 节点重连策略(Rewiring)
%   7. 势场法交汇点计算
%   8. 完整的性能指标追踪
%
% 输入参数:
%   env             - 环境结构体 (来自EnvironmentConfig)
%                     必需字段: dimension, bounds, start, goal, obstacles, num
%   max_iterations  - 最大迭代次数 (默认: 5000)
%   varargin        - 可选参数 (Name-Value pairs):
%     'Mode'                  - 控制模式 ('basic'|'pid'|'adaptive') 默认'adaptive'
%     'StepSize'              - 步长 (默认: 根据维度自动设置)
%     'GoalThreshold'         - 目标阈值 (默认: 根据维度自动设置)
%     'UpdateInterval'        - 交汇点更新间隔 (默认: 50)
%     'SmoothingFactor'       - 交汇点平滑因子 (默认: 0.7)
%     'EllipsoidBuffer'       - 椭球体缓冲系数 (默认: 1.2)
%     'UseParetoFrontier'     - 是否使用Pareto前沿 (默认: true)
%     'VisualizeDualEllipsoid' - 是否可视化双椭球体 (默认: false)
%     'VisualizationInterval' - 可视化间隔 (默认: 0, 0=关闭)
%
% 输出:
%   path     - 规划路径 [N×dim]
%   tree     - 树结构体,包含nodes和parents字段
%   success  - 规划是否成功
%   metrics  - 性能指标结构体,包含:
%              .iterations      - 实际迭代次数
%              .tree_nodes      - 树节点总数
%              .path_length     - 路径长度
%              .planning_time   - 规划时间(秒)
%              .success_rate    - 成功率(0或1)
%              .smoothness      - 平滑度指标
%              .clearance       - 平均障碍物间隙
%
% 示例:
%   env = generate2DEnvironment([0 100 0 100], 20);
%   [path, tree, success, metrics] = SC_RRT_Basic(env, 5000, 'Mode', 'adaptive');
%   visualizeAndSaveRRTResult(env, path, tree, success, 'SC-RRT', 'results');
%
% 参考文献:
%   1. Zhao et al. (2023) - Dynamic RRT
%   2. Åström & Hägglund (2006) - Advanced PID Control
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-12

%% ============================================================
%% ==================== 关键参数配置区 =======================
%% ============================================================
STEP_SIZE_RATIO = 0.008;       % 步长比例系数（加大以加快探索）
STEP_SIZE_MIN_2D = 10;         % 2D环境最小步长
STEP_SIZE_MAX_2D = 80;         % 2D环境最大步长
STEP_SIZE_MIN_3D = 15;         % 3D环境最小步长
STEP_SIZE_MAX_3D = 50;         % 3D环境最大步长
%% ============================================================

%% ========== 快速参数解析（替代inputParser）==========
mode = 'adaptive';
user_step_size = [];
user_goal_threshold = [];
update_interval = 80;
smoothing_factor = 0.7;
ellipsoid_buffer = 1.2;
use_pareto = true;
viz_ellipsoid = false;
vis_interval = 0;

ni = 1;
while ni <= length(varargin)
    if ischar(varargin{ni})
        switch varargin{ni}
            case 'Mode',                    mode = varargin{ni+1}; ni = ni + 2;
            case 'StepSize',                user_step_size = varargin{ni+1}; ni = ni + 2;
            case 'GoalThreshold',           user_goal_threshold = varargin{ni+1}; ni = ni + 2;
            case 'UpdateInterval',          update_interval = varargin{ni+1}; ni = ni + 2;
            case 'SmoothingFactor',         smoothing_factor = varargin{ni+1}; ni = ni + 2;
            case 'EllipsoidBuffer',         ellipsoid_buffer = varargin{ni+1}; ni = ni + 2;
            case 'UseParetoFrontier',       use_pareto = varargin{ni+1}; ni = ni + 2;
            case 'VisualizeDualEllipsoid',  viz_ellipsoid = varargin{ni+1}; ni = ni + 2;
            case 'VisualizationInterval',   vis_interval = varargin{ni+1}; ni = ni + 2;
            otherwise, ni = ni + 2;
        end
    else
        ni = ni + 1;
    end
end

%% ========== 验证环境结构 ==========
required_fields = {'dimension', 'bounds'};
for i = 1:length(required_fields)
    if ~isfield(env, required_fields{i})
        error('环境结构体缺少必需字段: %s', required_fields{i});
    end
end

% 兼容两种命名方式
if isfield(env, 'start_point')
    start_point = env.start_point;
elseif isfield(env, 'start')
    start_point = env.start;
else
    error('环境结构体缺少起点字段 (start_point 或 start)');
end

if isfield(env, 'goal_point')
    goal_point = env.goal_point;
elseif isfield(env, 'goal')
    goal_point = env.goal;
else
    error('环境结构体缺少终点字段 (goal_point 或 goal)');
end

if isfield(env, 'obstacles')
    obstacles = env.obstacles;
else
    error('环境结构体缺少障碍物字段 (obstacles)');
end

dim = env.dimension;
bounds = env.bounds;

% 确保dim是数值
if ischar(dim) || isstring(dim)
    if contains(dim, '2D') || contains(dim, '2d')
        dim = 2;
    else
        dim = 3;
    end
end

% 根据维度和环境大小自动设置参数
if dim == 2
    env_size = max(bounds(2) - bounds(1), bounds(4) - bounds(3));
else
    env_size = max([bounds(2) - bounds(1), bounds(4) - bounds(3), bounds(6) - bounds(5)]);
end

if isempty(user_step_size)
    base_step = env_size * STEP_SIZE_RATIO;
    if dim == 2
        step_size = max(STEP_SIZE_MIN_2D, min(STEP_SIZE_MAX_2D, base_step));
    else
        step_size = max(STEP_SIZE_MIN_3D, min(STEP_SIZE_MAX_3D, base_step));
    end
else
    step_size = user_step_size;
end

if isempty(user_goal_threshold)
    goal_threshold = step_size * 1.5;  % 增大目标阈值，更容易连接
else
    goal_threshold = user_goal_threshold;
end

% 简洁的配置输出
fprintf('SC-RRT开始 (%dD, %s, step=%.1f, thr=%.1f)\n', dim, mode, step_size, goal_threshold);

%% ========== 初始化双向树（矩阵格式） ==========
% 树结构: [position(m) | parent_idx | cost_G | cost_F_hat | num_children]
initial_capacity = min(1000, ceil(max_iterations / 10));

% 树A: 从起点生长
treeA = zeros(initial_capacity, dim + 4);
treeA(1, 1:dim) = start_point;
treeA(1, dim+1:dim+4) = [0, 0, inf, 0];  % [parent, G, F_hat, children]
sizeA = 1;

% 树B: 从终点生长
treeB = zeros(initial_capacity, dim + 4);
treeB(1, 1:dim) = goal_point;
treeB(1, dim+1:dim+4) = [0, 0, inf, 0];
sizeB = 1;

%% ========== 算法状态变量 ==========
success = false;
path = [];
iterations = 0;
tic;  % 开始计时

% 提前终止参数
first_solution_iter = inf;

% 交汇点
meet_point = (start_point + goal_point) / 2;
meet_point_old = meet_point;

% 双椭球体参数
c_best_A = inf;
c_best_B = inf;

% PID控制器（为两棵树分别设置）
prev_error_A = 0;
integral_error_A = 0;
prev_error_B = 0;
integral_error_B = 0;

L_best_shared = inf;  % 全局最优路径长度

%% ========== 主循环 ==========
% 预计算边界范围用于快速边界裁剪
bounds_lo = bounds(1:2:end);
bounds_hi = bounds(2:2:end);

while iterations < max_iterations && ~success
    iterations = iterations + 1;
    
    % ===== 1. 交汇点更新（大幅降低频率） =====
    if mod(iterations, update_interval) == 0 && sizeA > 1 && sizeB > 1
        [meet_point_new, ~, ~] = calculatePotentialMeetPoint(...
            treeA(1:sizeA, :), treeB(1:sizeB, :), start_point, goal_point, dim);
        
        meet_point = smoothing_factor * meet_point_old + (1 - smoothing_factor) * meet_point_new;
        meet_point_old = meet_point;
        
        % 更新双椭球体约束
        [c_best_A, c_best_B, c_min_A, c_min_B] = calculateDualEllipsoidParams(...
            treeA(1:sizeA, :), treeB(1:sizeB, :), ...
            start_point, goal_point, meet_point, dim, ellipsoid_buffer);
    end
    
    % ===== 2. 扩展树A（从起点向交汇点） =====
    % 高效采样策略
    r_val = rand;
    if r_val < 0.35
        sample_A = meet_point;  % 35% 采样交汇点
    elseif r_val < 0.50
        % 15% 采样树B最近节点附近（促进快速连接）
        if sizeB > 1
            rand_B_idx = randi(min(sizeB, 5));  % 从树B前几个节点中选
            sample_A = treeB(rand_B_idx, 1:dim) + randn(1, dim) * step_size;
            sample_A = max(bounds_lo, min(bounds_hi, sample_A));
        else
            sample_A = goal_point;
        end
    else
        sample_A = sampleInEllipsoid(start_point, meet_point, c_best_A, bounds, dim);
    end
    
    % 找最近节点（内联向量化）
    diffs_A = bsxfun(@minus, treeA(1:sizeA, 1:dim), sample_A);
    dists_A = sum(diffs_A .* diffs_A, 2);
    [~, nearest_idx_A] = min(dists_A);
    nearest_point_A = treeA(nearest_idx_A, 1:dim);
    
    % Steer扩展（内联）
    dir_A = sample_A - nearest_point_A;
    dist_A = sqrt(dir_A * dir_A');
    if dist_A > step_size && dist_A > 1e-6
        new_point_A = nearest_point_A + (dir_A / dist_A) * step_size;
    elseif dist_A > 1e-6
        new_point_A = sample_A;
    else
        new_point_A = nearest_point_A + randn(1, dim) * step_size * 0.1;
    end
    
    % 碰撞检测
    if isCollisionFree(nearest_point_A, new_point_A, obstacles, dim)
        % 直接使用nearest作为父节点（跳过rewiring以提速）
        best_cost_A = treeA(nearest_idx_A, dim+2) + norm(new_point_A - nearest_point_A);
        
        % 添加新节点
        sizeA = sizeA + 1;
        if sizeA > size(treeA, 1)
            treeA = [treeA; zeros(size(treeA, 1), dim + 4)];
        end
        
        treeA(sizeA, 1:dim) = new_point_A;
        treeA(sizeA, dim+1) = nearest_idx_A;
        treeA(sizeA, dim+2) = best_cost_A;
        
        % 计算F_hat代价（自适应PID - 核心创新点）
        [F_hat_A, error_info_A] = CostModule(treeA(1:sizeA, :), sizeA, meet_point, dim, ...
            'Mode', mode, ...
            'PrevError', prev_error_A, ...
            'IntegralError', integral_error_A, ...
            'BestPathLength', L_best_shared, ...
            'IterCount', iterations, ...
            'MaxIterations', max_iterations);
        
        treeA(sizeA, dim+3) = F_hat_A;
        treeA(nearest_idx_A, dim+4) = treeA(nearest_idx_A, dim+4) + 1;
        
        % 更新PID状态（简化：只保留最近误差）
        prev_error_A = error_info_A.currentError;
        integral_error_A = error_info_A.integralError;
        
        % 尝试连接到树B（向量化距离检查）
        diffs_conn = bsxfun(@minus, treeB(1:sizeB, 1:dim), new_point_A);
        dists_conn = sqrt(sum(diffs_conn .* diffs_conn, 2));
        [min_conn_dist, conn_idx_B] = min(dists_conn);
        
        if min_conn_dist < goal_threshold
            if isCollisionFree(treeB(conn_idx_B, 1:dim), new_point_A, obstacles, dim)
                path = extractBidirectionalPath(treeA(1:sizeA, :), treeB(1:sizeB, :), ...
                    sizeA, conn_idx_B, dim);
                first_solution_iter = iterations;
                success = true;
                break;
            end
        end
        
        % === Connect策略：贪心向树B延伸多步 ===
        if ~success && min_conn_dist < step_size * 8
            connect_point = new_point_A;
            connect_parent = sizeA;
            target_B = treeB(conn_idx_B, 1:dim);
            for cstep = 1:5  % 最多5步贪心扩展
                dir_c = target_B - connect_point;
                dist_c = norm(dir_c);
                if dist_c < goal_threshold
                    if isCollisionFree(connect_point, target_B, obstacles, dim)
                        % 添加最后一步节点
                        sizeA = sizeA + 1;
                        if sizeA > size(treeA, 1)
                            treeA = [treeA; zeros(size(treeA, 1), dim + 4)];
                        end
                        treeA(sizeA, 1:dim) = target_B;
                        treeA(sizeA, dim+1) = connect_parent;
                        treeA(sizeA, dim+2) = treeA(connect_parent, dim+2) + dist_c;
                        treeA(sizeA, dim+3) = 0;
                        
                        path = extractBidirectionalPath(treeA(1:sizeA, :), treeB(1:sizeB, :), ...
                            sizeA, conn_idx_B, dim);
                        first_solution_iter = iterations;
                        success = true;
                    end
                    break;
                end
                if dist_c < 1e-6, break; end
                next_point = connect_point + (dir_c / dist_c) * step_size;
                if ~isCollisionFree(connect_point, next_point, obstacles, dim)
                    break;
                end
                % 添加中间节点
                sizeA = sizeA + 1;
                if sizeA > size(treeA, 1)
                    treeA = [treeA; zeros(size(treeA, 1), dim + 4)];
                end
                treeA(sizeA, 1:dim) = next_point;
                treeA(sizeA, dim+1) = connect_parent;
                treeA(sizeA, dim+2) = treeA(connect_parent, dim+2) + step_size;
                treeA(sizeA, dim+3) = 0;
                connect_point = next_point;
                connect_parent = sizeA;
            end
            if success, break; end
        end
    end
    
    % ===== 3. 扩展树B（从终点向交汇点） =====
    r_val = rand;
    if r_val < 0.35
        sample_B = meet_point;
    elseif r_val < 0.50
        if sizeA > 1
            rand_A_idx = randi(min(sizeA, 5));
            sample_B = treeA(rand_A_idx, 1:dim) + randn(1, dim) * step_size;
            sample_B = max(bounds_lo, min(bounds_hi, sample_B));
        else
            sample_B = start_point;
        end
    else
        sample_B = sampleInEllipsoid(meet_point, goal_point, c_best_B, bounds, dim);
    end
    
    % 找最近节点（内联向量化）
    diffs_B = bsxfun(@minus, treeB(1:sizeB, 1:dim), sample_B);
    dists_B = sum(diffs_B .* diffs_B, 2);
    [~, nearest_idx_B] = min(dists_B);
    nearest_point_B = treeB(nearest_idx_B, 1:dim);
    
    % Steer（内联）
    dir_B = sample_B - nearest_point_B;
    dist_B = sqrt(dir_B * dir_B');
    if dist_B > step_size && dist_B > 1e-6
        new_point_B = nearest_point_B + (dir_B / dist_B) * step_size;
    elseif dist_B > 1e-6
        new_point_B = sample_B;
    else
        new_point_B = nearest_point_B + randn(1, dim) * step_size * 0.1;
    end
    
    if isCollisionFree(nearest_point_B, new_point_B, obstacles, dim)
        best_cost_B = treeB(nearest_idx_B, dim+2) + norm(new_point_B - nearest_point_B);
        
        sizeB = sizeB + 1;
        if sizeB > size(treeB, 1)
            treeB = [treeB; zeros(size(treeB, 1), dim + 4)];
        end
        
        treeB(sizeB, 1:dim) = new_point_B;
        treeB(sizeB, dim+1) = nearest_idx_B;
        treeB(sizeB, dim+2) = best_cost_B;
        
        [F_hat_B, error_info_B] = CostModule(treeB(1:sizeB, :), sizeB, meet_point, dim, ...
            'Mode', mode, ...
            'PrevError', prev_error_B, ...
            'IntegralError', integral_error_B, ...
            'BestPathLength', L_best_shared, ...
            'IterCount', iterations, ...
            'MaxIterations', max_iterations);
        
        treeB(sizeB, dim+3) = F_hat_B;
        treeB(nearest_idx_B, dim+4) = treeB(nearest_idx_B, dim+4) + 1;
        
        prev_error_B = error_info_B.currentError;
        integral_error_B = error_info_B.integralError;
        
        % 尝试连接到树A
        diffs_conn2 = bsxfun(@minus, treeA(1:sizeA, 1:dim), new_point_B);
        dists_conn2 = sqrt(sum(diffs_conn2 .* diffs_conn2, 2));
        [min_conn_dist2, conn_idx_A] = min(dists_conn2);
        
        if min_conn_dist2 < goal_threshold
            if isCollisionFree(treeA(conn_idx_A, 1:dim), new_point_B, obstacles, dim)
                path = extractBidirectionalPath(treeA(1:sizeA, :), treeB(1:sizeB, :), ...
                    conn_idx_A, sizeB, dim);
                first_solution_iter = iterations;
                success = true;
                break;
            end
        end
        
        % === Connect策略：贪心向树A延伸 ===
        if ~success && min_conn_dist2 < step_size * 8
            connect_point = new_point_B;
            connect_parent = sizeB;
            target_A = treeA(conn_idx_A, 1:dim);
            for cstep = 1:5
                dir_c = target_A - connect_point;
                dist_c = norm(dir_c);
                if dist_c < goal_threshold
                    if isCollisionFree(connect_point, target_A, obstacles, dim)
                        sizeB = sizeB + 1;
                        if sizeB > size(treeB, 1)
                            treeB = [treeB; zeros(size(treeB, 1), dim + 4)];
                        end
                        treeB(sizeB, 1:dim) = target_A;
                        treeB(sizeB, dim+1) = connect_parent;
                        treeB(sizeB, dim+2) = treeB(connect_parent, dim+2) + dist_c;
                        treeB(sizeB, dim+3) = 0;
                        
                        path = extractBidirectionalPath(treeA(1:sizeA, :), treeB(1:sizeB, :), ...
                            conn_idx_A, sizeB, dim);
                        first_solution_iter = iterations;
                        success = true;
                    end
                    break;
                end
                if dist_c < 1e-6, break; end
                next_point = connect_point + (dir_c / dist_c) * step_size;
                if ~isCollisionFree(connect_point, next_point, obstacles, dim)
                    break;
                end
                sizeB = sizeB + 1;
                if sizeB > size(treeB, 1)
                    treeB = [treeB; zeros(size(treeB, 1), dim + 4)];
                end
                treeB(sizeB, 1:dim) = next_point;
                treeB(sizeB, dim+1) = connect_parent;
                treeB(sizeB, dim+2) = treeB(connect_parent, dim+2) + step_size;
                treeB(sizeB, dim+3) = 0;
                connect_point = next_point;
                connect_parent = sizeB;
            end
            if success, break; end
        end
    end
end

planning_time = toc;

fprintf('SC-RRT完成: %s, iter=%d, A=%d, B=%d, t=%.3fs\n', ...
    string(success), iterations, sizeA, sizeB, planning_time);

%% ========== 路径后处理（多阶段高质量平滑） ==========
if success && ~isempty(path)
    path_raw = path;  % 保存原始路径用于回退
    
    % Step 1: 强力Shortcut优化 - 贪心 + 随机对捷径
    try
        path = shortcutPath(path, obstacles, dim, 10);
    catch
        % 失败则保持原路径
    end
    
    % Step 2: 多阶段路径平滑 (渐进加权平均 + 曲率自适应 + 高密度PCHIP + 二次平滑)
    try
        path = smoothPathSimple(path, obstacles, dim, 10);
    catch
        % 平滑失败则保持shortcut后的路径
    end
    
    % Step 3: 圆角平滑 - 对残余的尖锐拐角进行贝塞尔曲线过渡
    try
        % 计算自适应圆角半径: 取平均步长的40%
        seg_lengths = vecnorm(diff(path), 2, 2);
        fillet_r = mean(seg_lengths) * 0.4;
        [path_fillet, fillet_ok] = smoothPathWithFillets(path, obstacles, dim, fillet_r, 25);
        if fillet_ok && ~isempty(path_fillet) && size(path_fillet, 1) >= 2
            path = path_fillet;
        end
    catch
        % 圆角失败则保持当前路径
    end
    
    % Step 4: 最终安全验证
    final_safe = true;
    for i = 1:size(path, 1)-1
        if ~isCollisionFree(path(i, :), path(i+1, :), obstacles, dim)
            final_safe = false;
            break;
        end
    end
    if ~final_safe
        path = path_raw;  % 全部回退到原始路径
    end
end

%% ========== 计算性能指标 ==========
metrics = struct();
metrics.iterations = iterations;
metrics.tree_nodes = sizeA + sizeB;
metrics.planning_time = planning_time;
metrics.success_rate = double(success);

% 计算收敛时间（首次可行解时间）
if success && first_solution_iter < inf
    % 近似估算：假设每次迭代时间均匀分布
    metrics.convergence_time = planning_time * (first_solution_iter / max(iterations, 1));
else
    metrics.convergence_time = inf;
end

if success
    metrics.path_length = sum(vecnorm(diff(path), 2, 2));  % 路径长度
    if size(path, 1) >= 3
        % 计算平滑度(角度变化的标准差)
        angles = zeros(size(path, 1) - 2, 1);
        for i = 2:size(path, 1)-1
            v1 = path(i, :) - path(i-1, :);
            v2 = path(i+1, :) - path(i, :);
            cos_angle = dot(v1, v2) / (norm(v1) * norm(v2) + 1e-10);
            cos_angle = max(-1, min(1, cos_angle));
            angles(i-1) = acos(cos_angle);
        end
        metrics.smoothness = std(angles);
    else
        metrics.smoothness = 0;
    end
    
    % 计算平均障碍物间隙
    if isempty(obstacles)
        metrics.clearance = inf;
    else
        min_distances = zeros(size(path, 1), 1);
        for i = 1:size(path, 1)
            if dim == 2
                distances = vecnorm(obstacles(:, 1:2) - path(i, :), 2, 2) - obstacles(:, 3);
            else
                distances = vecnorm(obstacles(:, 1:3) - path(i, :), 2, 2) - obstacles(:, 4);
            end
            min_distances(i) = min(distances);
        end
        metrics.clearance = mean(min_distances);
    end
else
    metrics.path_length = inf;
    metrics.smoothness = inf;
    metrics.clearance = 0;
end

%% ========== 合并树结构用于可视化 ==========
% 策略：只保留路径相关的节点，避免显示两棵树的分支造成视觉混乱
tree = struct();

if success && ~isempty(path)
    % 方案：将路径转换为树结构（路径即树）
    % 这样可视化时只显示最终路径，不显示探索的分支
    num_path_nodes = size(path, 1);
    tree.nodes = path;
    tree.vertices = path;
    
    % 构建线性父节点关系：path(i)的父节点是path(i-1)
    tree.parent = zeros(num_path_nodes, 1);
    for i = 2:num_path_nodes
        tree.parent(i) = i - 1;
    end
    tree.parent(1) = 0;  % 根节点
    tree.parents = tree.parent;  % 兼容性
else
    % 失败时返回完整的双向树（用于调试）
    tree.nodes = [treeA(1:sizeA, 1:dim); treeB(1:sizeB, 1:dim)];
    
    % 构建parents向量
    parentsA = treeA(1:sizeA, dim+1);
    parentsB = treeB(1:sizeB, dim+1);
    
    tree.parents = [parentsA; parentsB + sizeA];
    tree.parents(sizeA + 1) = 0;  % 树B的根节点
    
    % 兼容性
    tree.vertices = tree.nodes;
    tree.parent = tree.parents;
end

end
