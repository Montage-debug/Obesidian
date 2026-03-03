function [path, tree, success, metrics] = SC_RRT_Basic(env, max_iterations, varargin)
% SC_RRT_Basic - SC-RRT算法：自适应双向超椭球约束采样路径规划
%
% 功能特点:
%   1. 双向RRT with 非对称双椭球约束采样 (ADCS)
%   2. 动态交汇点转移机制（加权质心法）
%   3. Pareto前沿引导的对树连接目标选择
%   4. 基于搜索状态反馈的在线调节机制 (SSFOR)
%   5. LinUCB上下文老虎机驱动的SSFOR参数自适应 (OnlineParamTuner)
%   6. 节点重连策略(Rewiring) + Connect贪心扩展
%   7. 双椭球交集可达性保障
%   8. 七阶段路径后处理管线
%   9. 完整的性能指标追踪（含在线学习统计）
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
STEP_SIZE_RATIO = 0.010;       % 步长比例系数（加大以加快探索）
STEP_SIZE_MIN_2D = 12;         % 2D环境最小步长
STEP_SIZE_MAX_2D = 80;         % 2D环境最大步长
STEP_SIZE_MIN_3D = 18;         % 3D环境最小步长
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

% ===== SSFOR在线调节机制状态 =====
% PIDSamplingController 控制 gamma(椭球膨胀系数) 和 p_informed(知情采样概率)
pid_state = [];          % PID状态结构体（首次调用时自动初始化）
gamma_A = 4.0;           % 初始膨胀系数（最大探索）
gamma_B = 4.0;
p_informed_A = 0.0;      % 初始知情采样概率（无解时不用知情采样）
p_informed_B = 0.0;
ssfor_update_count = 0;  % SSFOR更新计数

% ===== 在线参数调优器状态（LinUCB上下文老虎机） =====
tuner_state = [];                    % OnlineParamTuner状态（首次调用时自动初始化）
tuner_reward_pending = NaN;          % 待反馈的奖励值（首轮为NaN表示无历史）
prev_L_best_for_reward = inf;        % 上一SSFOR更新周期的最优路径长度
current_cross_tree_ratio = 0.15;     % 当前对树连接采样比例（由OnlineParamTuner在线调节）

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
        
        % ===== SSFOR在线调节：OnlineParamTuner驱动的自适应参数选择 =====
        if strcmp(mode, 'adaptive') || strcmp(mode, 'pid')
            % --- 步骤1：计算上一轮参数配置的奖励信号 ---
            if ~isnan(tuner_reward_pending)
                % 基于路径代价改进率计算奖励 r ∈ [0, 1]
                if isinf(prev_L_best_for_reward) && ~isinf(L_best_shared)
                    tuner_reward_pending = 1.0;   % 首次发现可行解：最高奖励
                elseif ~isinf(prev_L_best_for_reward) && ~isinf(L_best_shared)
                    delta_improve = (prev_L_best_for_reward - L_best_shared) / prev_L_best_for_reward;
                    tuner_reward_pending = max(0, min(1, delta_improve * 20));  % 放大微小改进
                else
                    tuner_reward_pending = 0.01;  % 无解期间的微小探索奖励
                end
            end
            
            % --- 步骤2：构建搜索状态上下文特征 ---
            tuner_context = struct();
            tuner_context.alpha = iterations / max_iterations;
            tuner_context.cost_improvement = 0;
            if ~isinf(prev_L_best_for_reward) && ~isinf(L_best_shared)
                tuner_context.cost_improvement = max(0, ...
                    (prev_L_best_for_reward - L_best_shared) / prev_L_best_for_reward);
            end
            tuner_context.tree_balance = sizeA / max(1, sizeA + sizeB);
            tuner_context.has_solution = double(~isinf(L_best_shared));
            tuner_context.gamma_current = gamma_A;
            tuner_context.p_current = p_informed_A;
            
            % --- 步骤3：LinUCB选择最优SSFOR参数配置 ---
            [tuner_state, tuner_params] = OnlineParamTuner(...
                tuner_state, tuner_context, tuner_reward_pending);
            
            % --- 步骤4：用学习到的参数驱动SSFOR-PID控制器 ---
            [pid_state, gamma_A, p_informed_A] = PIDSamplingController(...
                pid_state, L_best_shared, ...
                'WindowSize', 50, ...
                'TargetEfficiency', tuner_params.TargetEfficiency, ...
                'Kp', tuner_params.Kp, ...
                'Ki', tuner_params.Ki, ...
                'Kd', tuner_params.Kd);
            
            % 树B共享同一组gamma/p（对称调节）
            gamma_B = gamma_A;
            p_informed_B = p_informed_A;
            current_cross_tree_ratio = tuner_params.CrossTreeRatio;
            
            % 更新奖励追踪状态
            tuner_reward_pending = 0;           % 标记为非NaN（下次进入时会计算实际奖励）
            prev_L_best_for_reward = L_best_shared;
            ssfor_update_count = ssfor_update_count + 1;
        end
    end
    
    % ===== 2. 扩展树A（从起点向交汇点） =====
    % SSFOR驱动的自适应采样策略
    r_val = rand;
    if r_val < p_informed_A && ~isinf(c_best_A)
        % 知情采样：在gamma膨胀的椭球内采样（SSFOR控制的约束域）
        sample_A = sampleInEllipsoid(start_point, meet_point, c_best_A * gamma_A, bounds, dim);
    elseif r_val < p_informed_A + (1 - p_informed_A) * current_cross_tree_ratio
        % Pareto前沿引导的对树连接偏置（比例由OnlineParamTuner在线调节）
        if sizeB > 3 && use_pareto
            % 从树B的Pareto前沿中选取高质量目标节点（多目标：代价+生长度+路径曲折度）
            [pareto_target, ~, ~, ~] = ParetoModule(treeB(1:sizeB, :), meet_point, 0.15, dim);
            sample_A = pareto_target + randn(1, dim) * step_size * 0.3;
            sample_A = max(bounds_lo, min(bounds_hi, sample_A));
        elseif sizeB > 1
            rand_B_idx = max(1, sizeB - randi(min(sizeB, 5)) + 1);
            sample_A = treeB(rand_B_idx, 1:dim) + randn(1, dim) * step_size;
            sample_A = max(bounds_lo, min(bounds_hi, sample_A));
        else
            sample_A = goal_point;
        end
    else
        % 全局均匀采样（保持探索覆盖性）
        sample_A = zeros(1, dim);
        for d_i = 1:dim
            sample_A(d_i) = bounds_lo(d_i) + rand * (bounds_hi(d_i) - bounds_lo(d_i));
        end
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
        % 找邻域内的节点（RRT*思想：不仅选最近的，还选邻域内的最优）
        radius = min(step_size * 2.5, norm(goal_point - start_point) / 20);
        diffs_nb = bsxfun(@minus, treeA(1:sizeA, 1:dim), new_point_A);
        dists_nb = sqrt(sum(diffs_nb .* diffs_nb, 2));
        neighbor_indices = find(dists_nb <= radius);
        
        % 在邻域内选择最优父节点
        best_cost_A = treeA(nearest_idx_A, dim+2) + norm(new_point_A - nearest_point_A);
        best_parent_A = nearest_idx_A;
        
        for nb_idx = neighbor_indices(:)'
            if nb_idx == nearest_idx_A, continue; end
            tentative_cost = treeA(nb_idx, dim+2) + norm(new_point_A - treeA(nb_idx, 1:dim));
            if tentative_cost < best_cost_A && isCollisionFree(treeA(nb_idx, 1:dim), new_point_A, obstacles, dim)
                best_cost_A = tentative_cost;
                best_parent_A = nb_idx;
            end
        end
        
        % 添加新节点
        sizeA = sizeA + 1;
        if sizeA > size(treeA, 1)
            treeA = [treeA; zeros(size(treeA, 1), dim + 4)];
        end
        
        treeA(sizeA, 1:dim) = new_point_A;
        treeA(sizeA, dim+1) = best_parent_A;
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
        treeA(best_parent_A, dim+4) = treeA(best_parent_A, dim+4) + 1;
        
        % 更新PID状态（简化：只保留最近误差）
        prev_error_A = error_info_A.currentError;
        integral_error_A = error_info_A.integralError;
        
        % RRT*重布线：检查新节点是否能为邻域内的节点提供更好的路径
        for nb_idx = neighbor_indices(:)'
            if nb_idx == best_parent_A, continue; end
            new_cost = best_cost_A + norm(treeA(nb_idx, 1:dim) - new_point_A);
            if new_cost < treeA(nb_idx, dim+2) && isCollisionFree(new_point_A, treeA(nb_idx, 1:dim), obstacles, dim)
                treeA(nb_idx, dim+1) = sizeA;
                treeA(nb_idx, dim+2) = new_cost;
                treeA(sizeA, dim+4) = treeA(sizeA, dim+4) + 1;
                treeA(best_parent_A, dim+4) = treeA(best_parent_A, dim+4) - 1;
            end
        end
        
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
        if ~success && min_conn_dist < step_size * 12
            connect_point = new_point_A;
            connect_parent = sizeA;
            target_B = treeB(conn_idx_B, 1:dim);
            for cstep = 1:8  % 最多8步贪心扩展
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
    % SSFOR驱动的自适应采样策略（树B侧）
    r_val = rand;
    if r_val < p_informed_B && ~isinf(c_best_B)
        % 知情采样：在gamma膨胀的椭球内采样（SSFOR控制的约束域）
        sample_B = sampleInEllipsoid(meet_point, goal_point, c_best_B * gamma_B, bounds, dim);
    elseif r_val < p_informed_B + (1 - p_informed_B) * current_cross_tree_ratio
        % Pareto前沿引导的对树连接偏置（比例由OnlineParamTuner在线调节）
        if sizeA > 3 && use_pareto
            % 从树A的Pareto前沿中选取高质量目标节点（多目标：代价+生长度+路径曲折度）
            [pareto_target, ~, ~, ~] = ParetoModule(treeA(1:sizeA, :), meet_point, 0.15, dim);
            sample_B = pareto_target + randn(1, dim) * step_size * 0.3;
            sample_B = max(bounds_lo, min(bounds_hi, sample_B));
        elseif sizeA > 1
            rand_A_idx = max(1, sizeA - randi(min(sizeA, 5)) + 1);
            sample_B = treeA(rand_A_idx, 1:dim) + randn(1, dim) * step_size;
            sample_B = max(bounds_lo, min(bounds_hi, sample_B));
        else
            sample_B = start_point;
        end
    else
        % 全局均匀采样
        sample_B = zeros(1, dim);
        for d_i = 1:dim
            sample_B(d_i) = bounds_lo(d_i) + rand * (bounds_hi(d_i) - bounds_lo(d_i));
        end
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
        % 找邻域内的节点（RRT*思想）
        radius = min(step_size * 2.5, norm(goal_point - start_point) / 20);
        diffs_nb = bsxfun(@minus, treeB(1:sizeB, 1:dim), new_point_B);
        dists_nb = sqrt(sum(diffs_nb .* diffs_nb, 2));
        neighbor_indices = find(dists_nb <= radius);
        
        % 在邻域内选择最优父节点
        best_cost_B = treeB(nearest_idx_B, dim+2) + norm(new_point_B - nearest_point_B);
        best_parent_B = nearest_idx_B;
        
        for nb_idx = neighbor_indices(:)'
            if nb_idx == nearest_idx_B, continue; end
            tentative_cost = treeB(nb_idx, dim+2) + norm(new_point_B - treeB(nb_idx, 1:dim));
            if tentative_cost < best_cost_B && isCollisionFree(treeB(nb_idx, 1:dim), new_point_B, obstacles, dim)
                best_cost_B = tentative_cost;
                best_parent_B = nb_idx;
            end
        end
        
        sizeB = sizeB + 1;
        if sizeB > size(treeB, 1)
            treeB = [treeB; zeros(size(treeB, 1), dim + 4)];
        end
        
        treeB(sizeB, 1:dim) = new_point_B;
        treeB(sizeB, dim+1) = best_parent_B;
        treeB(sizeB, dim+2) = best_cost_B;
        
        [F_hat_B, error_info_B] = CostModule(treeB(1:sizeB, :), sizeB, meet_point, dim, ...
            'Mode', mode, ...
            'PrevError', prev_error_B, ...
            'IntegralError', integral_error_B, ...
            'BestPathLength', L_best_shared, ...
            'IterCount', iterations, ...
            'MaxIterations', max_iterations);
        
        treeB(sizeB, dim+3) = F_hat_B;
        treeB(best_parent_B, dim+4) = treeB(best_parent_B, dim+4) + 1;
        
        prev_error_B = error_info_B.currentError;
        integral_error_B = error_info_B.integralError;
        
        % RRT*重布线：检查新节点是否能为邻域内的节点提供更好的路径
        for nb_idx = neighbor_indices(:)'
            if nb_idx == best_parent_B, continue; end
            new_cost = best_cost_B + norm(treeB(nb_idx, 1:dim) - new_point_B);
            if new_cost < treeB(nb_idx, dim+2) && isCollisionFree(new_point_B, treeB(nb_idx, 1:dim), obstacles, dim)
                treeB(nb_idx, dim+1) = sizeB;
                treeB(nb_idx, dim+2) = new_cost;
                treeB(sizeB, dim+4) = treeB(sizeB, dim+4) + 1;
                treeB(best_parent_B, dim+4) = treeB(best_parent_B, dim+4) - 1;
            end
        end
        
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
        if ~success && min_conn_dist2 < step_size * 12
            connect_point = new_point_B;
            connect_parent = sizeB;
            target_A = treeA(conn_idx_A, 1:dim);
            for cstep = 1:8
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
if ssfor_update_count > 0
    fprintf('  SSFOR: %d次更新, gamma=%.2f, p_informed=%.2f\n', ...
        ssfor_update_count, gamma_A, p_informed_A);
end
if ~isempty(tuner_state) && tuner_state.round > 0
    [~, best_arm_idx] = max(tuner_state.arm_counts);
    tuner_avg_reward = 0;
    if ~isempty(tuner_state.reward_history)
        tuner_avg_reward = mean(tuner_state.reward_history);
    end
    fprintf('  OnlineTuner(LinUCB): %d轮, 最优臂=%d(Kp=%.1f,Ki=%.2f,Kd=%.1f), 平均奖励=%.4f\n', ...
        tuner_state.round, best_arm_idx, ...
        tuner_state.armConfigs(best_arm_idx, 1), ...
        tuner_state.armConfigs(best_arm_idx, 2), ...
        tuner_state.armConfigs(best_arm_idx, 3), ...
        tuner_avg_reward);
end

%% ========== 路径后处理（七阶段高质量平滑与安全保障） ==========
if success && ~isempty(path)
    path_raw = path;  % 保存原始路径用于回退
    
    % Step 0: 移除近共线冗余节点（预清理，减少后续计算量）
    try
        if size(path, 1) > 4
            keep_mask = true(size(path, 1), 1);
            for ci = 2:size(path, 1)-1
                v1 = path(ci, :) - path(ci-1, :);
                v2 = path(ci+1, :) - path(ci, :);
                len1 = norm(v1); len2 = norm(v2);
                if len1 > 1e-8 && len2 > 1e-8
                    cos_a = dot(v1, v2) / (len1 * len2);
                    if cos_a > cosd(3)  % 偏差<3度视为共线
                        keep_mask(ci) = false;
                    end
                end
            end
            path = path(keep_mask, :);
        end
    catch
    end
    
    % Step 1: 强力Shortcut优化 - 贪心 + 随机对捷径（增强迭代）
    try
        path = shortcutPath(path, obstacles, dim, 18);
    catch
    end
    
    % Step 2: 弹性带拉直优化 - 将路径节点拉向局部最优位置（增强迭代）
    try
        path = pullPathToOptimal(path, obstacles, dim, 30);
    catch
    end
    
    % Step 3: 二次Shortcut - 拉直后可能产生新的可跳过段
    try
        path = shortcutPath(path, obstacles, dim, 10);
    catch
    end
    
    % Step 4: 自适应重采样 - 确保点间距均匀，避免PCHIP插值振荡
    try
        seg_lens_r = vecnorm(diff(path), 2, 2);
        max_gap = step_size * 1.5;
        if max(seg_lens_r) > max_gap * 2
            cum_lens_r = [0; cumsum(seg_lens_r)];
            total_len_r = cum_lens_r(end);
            target_n_r = max(size(path,1), ceil(total_len_r / max_gap));
            t_target_r = linspace(0, total_len_r, target_n_r)';
            path_resamp = zeros(target_n_r, dim);
            path_resamp(1,:) = path(1,:);
            path_resamp(end,:) = path(end,:);
            for ri = 2:target_n_r-1
                idx_r = find(cum_lens_r <= t_target_r(ri), 1, 'last');
                idx_r = min(idx_r, size(path,1)-1);
                alpha_r = (t_target_r(ri) - cum_lens_r(idx_r)) / max(seg_lens_r(idx_r), 1e-10);
                alpha_r = max(0, min(1, alpha_r));
                path_resamp(ri,:) = path(idx_r,:)*(1-alpha_r) + path(idx_r+1,:)*alpha_r;
            end
            resamp_ok = true;
            for ri = 1:size(path_resamp,1)-1
                if ~isCollisionFree(path_resamp(ri,:), path_resamp(ri+1,:), obstacles, dim)
                    resamp_ok = false; break;
                end
            end
            if resamp_ok
                path = path_resamp;
            end
        end
    catch
    end
    
    % Step 5: 多阶段路径平滑 (渐进加权平均 + 曲率自适应 + 高密度PCHIP + 二次平滑)
    try
        path = smoothPathSimple(path, obstacles, dim, 15);
    catch
    end
    
    % Step 6: 圆角平滑 - 对残余的尖锐拐角进行贝塞尔曲线过渡
    try
        seg_lengths = vecnorm(diff(path), 2, 2);
        fillet_r = mean(seg_lengths) * 0.4;
        [path_fillet, fillet_ok] = smoothPathWithFillets(path, obstacles, dim, fillet_r, 25);
        if fillet_ok && ~isempty(path_fillet) && size(path_fillet, 1) >= 2
            path = path_fillet;
        end
    catch
    end
    
    % Step 7: 最终安全验证（解析碰撞检测已保证精确性）
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

% ===== 在线学习统计（OnlineParamTuner） =====
if ~isempty(tuner_state) && tuner_state.round > 0
    metrics.online_learning = struct();
    metrics.online_learning.total_rounds = tuner_state.round;
    metrics.online_learning.arm_counts = tuner_state.arm_counts(:)';
    metrics.online_learning.arm_avg_rewards = ...
        (tuner_state.arm_total_reward(:)' ./ max(tuner_state.arm_counts(:)', 1));
    if ~isempty(tuner_state.reward_history)
        metrics.online_learning.avg_reward = mean(tuner_state.reward_history);
    else
        metrics.online_learning.avg_reward = 0;
    end
    [~, metrics.online_learning.best_arm] = max(tuner_state.arm_counts);
    metrics.online_learning.final_params = struct(...
        'Kp', tuner_state.armConfigs(metrics.online_learning.best_arm, 1), ...
        'Ki', tuner_state.armConfigs(metrics.online_learning.best_arm, 2), ...
        'Kd', tuner_state.armConfigs(metrics.online_learning.best_arm, 3), ...
        'TargetEfficiency', tuner_state.armConfigs(metrics.online_learning.best_arm, 4), ...
        'CrossTreeRatio', tuner_state.armConfigs(metrics.online_learning.best_arm, 5));
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
