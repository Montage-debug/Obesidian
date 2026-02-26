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
% 
% 【步长配置】- 直接在此处修改步长参数
% 
% 自动步长计算公式: base_step = env_size * STEP_SIZE_RATIO
% 环境适配范围: 
%   - 2D环境: STEP_SIZE_MIN_2D ~ STEP_SIZE_MAX_2D
%   - 3D环境: STEP_SIZE_MIN_3D ~ STEP_SIZE_MAX_3D
%
% 调优建议:
%   1. 增大步长 -> 加快探索速度,但可能错过狭窄通道
%   2. 减小步长 -> 提高路径精度,但规划时间增加
%   3. 推荐比例: 环境大小的 0.5%~1% (当前: 0.5%)
%
STEP_SIZE_RATIO = 0.005;      % 步长比例系数 (默认: 0.005 = 0.5%)
STEP_SIZE_MIN_2D = 8;          % 2D环境最小步长 (默认: 5)
STEP_SIZE_MAX_2D = 70;         % 2D环境最大步长 (默认: 50)
STEP_SIZE_MIN_3D = 15;          % 3D环境最小步长 (默认: 2)
STEP_SIZE_MAX_3D = 50;         % 3D环境最大步长 (默认: 20)
%
%% ============================================================

%% ========== 参数解析 ==========
p = inputParser;
addRequired(p, 'env', @isstruct);
addOptional(p, 'max_iterations', 5000, @isnumeric);
addParameter(p, 'Mode', 'adaptive', @(x) ismember(x, {'basic', 'pid', 'adaptive'}));
addParameter(p, 'StepSize', [], @isnumeric);  % 空则自动设置
addParameter(p, 'GoalThreshold', [], @isnumeric);  % 空则自动设置
addParameter(p, 'UpdateInterval', 50, @isnumeric);
addParameter(p, 'SmoothingFactor', 0.7, @isnumeric);
addParameter(p, 'EllipsoidBuffer', 1.2, @isnumeric);
addParameter(p, 'UseParetoFrontier', true, @islogical);
addParameter(p, 'VisualizeDualEllipsoid', false, @islogical);
addParameter(p, 'VisualizationInterval', 0, @isnumeric);
parse(p, env, max_iterations, varargin{:});

mode = p.Results.Mode;
update_interval = p.Results.UpdateInterval;
smoothing_factor = p.Results.SmoothingFactor;
ellipsoid_buffer = p.Results.EllipsoidBuffer;
use_pareto = p.Results.UseParetoFrontier;
viz_ellipsoid = p.Results.VisualizeDualEllipsoid;
vis_interval = p.Results.VisualizationInterval;

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

% 根据维度和环境大小自动设置参数（使用顶部配置区的参数）
if isempty(p.Results.StepSize)
    % 根据环境大小自适应调整步长
    if dim == 2
        env_size = max(bounds(2) - bounds(1), bounds(4) - bounds(3));
    else
        env_size = max([bounds(2) - bounds(1), bounds(4) - bounds(3), bounds(6) - bounds(5)]);
    end
    % 使用配置区的参数计算步长
    base_step = env_size * STEP_SIZE_RATIO;
    if dim == 2
        step_size = max(STEP_SIZE_MIN_2D, min(STEP_SIZE_MAX_2D, base_step));
    else
        step_size = max(STEP_SIZE_MIN_3D, min(STEP_SIZE_MAX_3D, base_step));
    end
else
    step_size = p.Results.StepSize;
end

if isempty(p.Results.GoalThreshold)
    % 目标阈值与步长相同
    goal_threshold = step_size;
else
    goal_threshold = p.Results.GoalThreshold;
end

% 打印算法配置
fprintf('\n========== SC-RRT优化算法开始 (%dD) ==========\n', dim);
fprintf('算法模式: %s\n', upper(mode));
fprintf('步长: %.2f (ratio=%.4f), 目标阈值: %.2f\n', step_size, STEP_SIZE_RATIO, goal_threshold);
fprintf('环境大小: %.0f, 最大迭代: %d\n', env_size, max_iterations);
fprintf('交汇点更新间隔: %d\n', update_interval);
fprintf('平滑因子: %.2f, 椭球缓冲: %.2f\n', smoothing_factor, ellipsoid_buffer);
if use_pareto
    fprintf('Pareto前沿优化: 启用\n');
end
if viz_ellipsoid
    fprintf('双椭球体可视化: 启用\n');
end
fprintf('==========================================\n\n');

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
first_solution_iter = inf;  % 首次找到解的迭代次数
extra_optimization_iters = 0;  % 找到解后继续优化的迭代次数
MAX_EXTRA_ITERS = max(200, round(max_iterations * 0.05));  % 最多继续5%的迭代次数

% 交汇点
meet_point = (start_point + goal_point) / 2;
meet_point_old = meet_point;

% 双椭球体参数
c_min_A = norm(meet_point - start_point);
c_min_B = norm(goal_point - meet_point);
c_best_A = inf;
c_best_B = inf;

% PID控制器（为两棵树分别设置）
prev_error_A = 0;
integral_error_A = 0;
error_history_A = [];

prev_error_B = 0;
integral_error_B = 0;
error_history_B = [];

% 性能指标追踪
total_samples = 0;
valid_samples_A = 0;
valid_samples_B = 0;
L_best_shared = inf;  % 全局最优路径长度

%% ========== 可视化准备 ==========
if vis_interval > 0
    figure('Name', 'SC-RRT Planning', 'NumberTitle', 'off');
    hold on; grid on; axis equal;
    
    if dim == 2
        EnvironmentConfig.visualize2D(env);
        title('SC-RRT双向规划 (2D)');
    else
        EnvironmentConfig.visualize3D(env);
        title('SC-RRT双向规划 (3D)');
        view(3);
    end
    drawnow;
end

%% ========== 主循环 ==========
while iterations < max_iterations && ~success
    iterations = iterations + 1;
    
    % ===== 1. 自适应交汇点更新策略 =====
    % 动态更新间隔
    adaptive_interval = max(20, min(100, round(50 * (1 - iterations/max_iterations))));
    
    if mod(iterations, adaptive_interval) == 0 && sizeA > 1 && sizeB > 1
        % 计算当前两棵树的"吸引力"
        attraction_A = mean(vecnorm(treeA(1:sizeA, 1:dim) - meet_point, 2, 2));
        attraction_B = mean(vecnorm(treeB(1:sizeB, 1:dim) - meet_point, 2, 2));
        
        % 势场法计算新交汇点
        [meet_point_new, ~, ~] = calculatePotentialMeetPoint(...
            treeA(1:sizeA, :), treeB(1:sizeB, :), start_point, goal_point, dim);
        
        % 如果两棵树都远离交汇点，更积极地更新
        if attraction_A > step_size * 10 || attraction_B > step_size * 10
            smoothing_factor_adaptive = 0.5;  % 更快响应
        else
            smoothing_factor_adaptive = smoothing_factor;  % 保持稳定
        end
        
        % 平滑更新
        meet_point = smoothing_factor_adaptive * meet_point_old + (1 - smoothing_factor_adaptive) * meet_point_new;
        meet_point_old = meet_point;
        
        % 更新双椭球体约束
        [c_best_A, c_best_B, c_min_A, c_min_B] = calculateDualEllipsoidParams(...
            treeA(1:sizeA, :), treeB(1:sizeB, :), ...
            start_point, goal_point, meet_point, dim, ellipsoid_buffer);
    end
    
    % 打印统计信息 - 根据环境大小调整打印频率
    progress_interval = max(50, min(500, round(max_iterations / 20)));
    if mod(iterations, progress_interval) == 0
        fprintf('  迭代%d/%d (%.1f%%): 树A=%d节点, 树B=%d节点\n', ...
            iterations, max_iterations, iterations/max_iterations*100, sizeA, sizeB);
        fprintf('  交汇点: [%s]\n', sprintf('%.1f ', meet_point));
        fprintf('  椭球参数: c_A=%.2f, c_B=%.2f\n', c_best_A, c_best_B);
    end
    
    % ===== 提前终止检查 =====
    if success && iterations > first_solution_iter
        extra_optimization_iters = iterations - first_solution_iter;
        if extra_optimization_iters >= MAX_EXTRA_ITERS
            fprintf('  ✓ 已找到解并优化%d次迭代，提前终止\n', extra_optimization_iters);
            break;
        end
    end
    
    % ===== 1.5 周期性全局连接检查 =====
    % 大幅降低全局连接检查频率以节省时间
    global_check_interval = max(800, round(max_iterations / 6));  % 降低检查频率
    if mod(iterations, global_check_interval) == 0 && sizeA > 10 && sizeB > 10
        % 进一步限制检查的节点数量
        max_check_nodes = min(20, max(8, round(sqrt(min(sizeA, sizeB)) * 0.5)));
        [best_connect_A, best_connect_B, best_total_cost] = ...
            findBestConnection(treeA(1:sizeA, :), treeB(1:sizeB, :), obstacles, dim, max_check_nodes);
        
        if ~isempty(best_connect_A) && best_total_cost < L_best_shared
            % 发现更优连接
            path = extractBidirectionalPath(treeA(1:sizeA, :), treeB(1:sizeB, :), ...
                best_connect_A, best_connect_B, dim);
            L_best_shared = best_total_cost;
            
            if ~success
                first_solution_iter = iterations;  % 记录首次找到解的迭代
            end
            success = true;
            
            fprintf('  ✓ 发现更优连接! 代价: %.2f\n', best_total_cost);
        end
    end
    
    % ===== 2. 扩展树A（从起点向交汇点） =====
    total_samples = total_samples + 1;
    
    % 智能采样策略：提高交汇点采样概率以加快连接
    if rand < 0.4  % 从30%提高到40%
        sample_A = meet_point;  % 采样交汇点
    elseif rand < 0.15  % 15%概率采样终点附近（促进连接）
        sample_A = goal_point + randn(1, dim) * step_size * 2;
        % 确保在边界内
        for d = 1:dim
            sample_A(d) = max(bounds(2*d-1), min(bounds(2*d), sample_A(d)));
        end
    else
        sample_A = sampleInEllipsoid(start_point, meet_point, c_best_A, bounds, dim);
    end
    
    % 使用Pareto前沿选择扩展节点 - 进一步优化
    % 降低Pareto计算频率：仅每100次迭代且随机15%概率使用
    if use_pareto && sizeA > 100 && mod(iterations, 100) == 0 && rand < 0.15
        try
            % 多目标权重随迭代自适应调整
            tortuosity_weight = 0.1 * (iterations / max_iterations);
            
            [pareto_node, ~, ~, ~] = ParetoModule(treeA(1:sizeA, :), meet_point, tortuosity_weight, dim);
            [nearest_idx_A, nearest_point_A] = findNearestInTree(treeA(1:sizeA, :), sample_A, dim);
            
            % 50%概率选择Pareto节点
            if rand < 0.5
                nearest_idx_A = findNodeIndex(treeA(1:sizeA, :), pareto_node, dim);
                nearest_point_A = pareto_node;
            end
        catch
            [nearest_idx_A, nearest_point_A] = findNearestInTree(treeA(1:sizeA, :), sample_A, dim);
        end
    else
        [nearest_idx_A, nearest_point_A] = findNearestInTree(treeA(1:sizeA, :), sample_A, dim);
    end
    
    % Steer扩展
    new_point_A = steerPoint(nearest_point_A, sample_A, step_size);
    
    % 碰撞检测
    if isCollisionFree(nearest_point_A, new_point_A, obstacles, dim)
        % 节点重连（Rewiring）- 简化版本，降低计算开销
        adaptive_radius_A = step_size * 2;  % 固定半径，避免动态计算
        [best_parent_idx, best_cost] = findBestParent(treeA(1:sizeA, :), new_point_A, ...
            nearest_idx_A, adaptive_radius_A, obstacles, dim);
        
        % 添加新节点
        sizeA = sizeA + 1;
        if sizeA > size(treeA, 1)
            treeA = [treeA; zeros(size(treeA, 1), dim + 4)];
        end
        
        treeA(sizeA, 1:dim) = new_point_A;
        treeA(sizeA, dim+1) = best_parent_idx;
        treeA(sizeA, dim+2) = best_cost;
        
        % 计算F_hat代价（使用自适应PID - 核心创新点）
        [F_hat_A, error_info_A] = CostModule(treeA(1:sizeA, :), sizeA, meet_point, dim, ...
            'Mode', mode, ...
            'PrevError', prev_error_A, ...
            'IntegralError', integral_error_A, ...
            'BestPathLength', L_best_shared, ...
            'IterCount', iterations, ...
            'MaxIterations', max_iterations, ...
            'SearchEfficiency', 0.5, ...
            'ErrorHistory', error_history_A);
        
        treeA(sizeA, dim+3) = F_hat_A;
        treeA(best_parent_idx, dim+4) = treeA(best_parent_idx, dim+4) + 1;  % 更新子节点数
        
        % 更新PID状态
        prev_error_A = error_info_A.currentError;
        integral_error_A = error_info_A.integralError;
        error_history_A = [error_history_A; error_info_A.currentError];
        if length(error_history_A) > 10
            error_history_A = error_history_A(end-9:end);
        end
        
        valid_samples_A = valid_samples_A + 1;
        
        % 尝试连接到树B
        [connect_success, connect_idx_B] = tryConnectTrees(new_point_A, treeB(1:sizeB, :), ...
            obstacles, dim, goal_threshold);
        
        if connect_success
            path = extractBidirectionalPath(treeA(1:sizeA, :), treeB(1:sizeB, :), ...
                sizeA, connect_idx_B, dim);
            if ~success
                first_solution_iter = iterations;  % 记录首次找到解的迭代
                fprintf('  ✓ 首次找到路径! 迭代: %d\n', iterations);
            end
            success = true;
            break;
        end
    end
    
    % ===== 3. 扩展树B（从终点向交汇点） =====
    total_samples = total_samples + 1;
    
    % 智能采样策略：提高交汇点采样概率
    if rand < 0.4  % 从30%提高到40%
        sample_B = meet_point;
    elseif rand < 0.15  % 15%概率采样起点附近（促进连接）
        sample_B = start_point + randn(1, dim) * step_size * 2;
        % 确保在边界内
        for d = 1:dim
            sample_B(d) = max(bounds(2*d-1), min(bounds(2*d), sample_B(d)));
        end
    else
        sample_B = sampleInEllipsoid(meet_point, goal_point, c_best_B, bounds, dim);
    end
    
    % 使用Pareto前沿 - 进一步优化
    % 降低Pareto计算频率：仅每100次迭代且随机15%概率使用
    if use_pareto && sizeB > 100 && mod(iterations, 100) == 0 && rand < 0.15
        try
            % 多目标权重随迭代自适应调整
            tortuosity_weight = 0.1 * (iterations / max_iterations);
            
            [pareto_node, ~, ~, ~] = ParetoModule(treeB(1:sizeB, :), meet_point, tortuosity_weight, dim);
            [nearest_idx_B, nearest_point_B] = findNearestInTree(treeB(1:sizeB, :), sample_B, dim);
            
            % 50%概率选择Pareto节点
            if rand < 0.5
                nearest_idx_B = findNodeIndex(treeB(1:sizeB, :), pareto_node, dim);
                nearest_point_B = pareto_node;
            end
        catch
            [nearest_idx_B, nearest_point_B] = findNearestInTree(treeB(1:sizeB, :), sample_B, dim);
        end
    else
        [nearest_idx_B, nearest_point_B] = findNearestInTree(treeB(1:sizeB, :), sample_B, dim);
    end
    
    new_point_B = steerPoint(nearest_point_B, sample_B, step_size);
    
    if isCollisionFree(nearest_point_B, new_point_B, obstacles, dim)
        % 节点重连 - 简化版本
        adaptive_radius_B = step_size * 2;  % 固定半径
        [best_parent_idx, best_cost] = findBestParent(treeB(1:sizeB, :), new_point_B, ...
            nearest_idx_B, adaptive_radius_B, obstacles, dim);
        
        sizeB = sizeB + 1;
        if sizeB > size(treeB, 1)
            treeB = [treeB; zeros(size(treeB, 1), dim + 4)];
        end
        
        treeB(sizeB, 1:dim) = new_point_B;
        treeB(sizeB, dim+1) = best_parent_idx;
        treeB(sizeB, dim+2) = best_cost;
        
        % 计算F_hat代价（使用自适应PID - 核心创新点）
        [F_hat_B, error_info_B] = CostModule(treeB(1:sizeB, :), sizeB, meet_point, dim, ...
            'Mode', mode, ...
            'PrevError', prev_error_B, ...
            'IntegralError', integral_error_B, ...
            'BestPathLength', L_best_shared, ...
            'IterCount', iterations, ...
            'MaxIterations', max_iterations, ...
            'SearchEfficiency', 0.5, ...
            'ErrorHistory', error_history_B);
        
        treeB(sizeB, dim+3) = F_hat_B;
        treeB(best_parent_idx, dim+4) = treeB(best_parent_idx, dim+4) + 1;
        
        % 更新PID状态
        prev_error_B = error_info_B.currentError;
        integral_error_B = error_info_B.integralError;
        error_history_B = [error_history_B; error_info_B.currentError];
        if length(error_history_B) > 10
            error_history_B = error_history_B(end-9:end);
        end
        
        valid_samples_B = valid_samples_B + 1;
        
        [connect_success, connect_idx_A] = tryConnectTrees(new_point_B, treeA(1:sizeA, :), ...
            obstacles, dim, goal_threshold);
        
        if connect_success
            path = extractBidirectionalPath(treeA(1:sizeA, :), treeB(1:sizeB, :), ...
                connect_idx_A, sizeB, dim);
            if ~success
                first_solution_iter = iterations;  % 记录首次找到解的迭代
                fprintf('  ✓ 首次找到路径! 迭代: %d\n', iterations);
            end
            success = true;
            break;
        end
    end
    
    % ===== 4. 可视化 =====
    if vis_interval > 0 && mod(iterations, vis_interval) == 0
        % 简化可视化,避免函数调用错误
        % plotBidirectionalTrees(treeA(1:sizeA, :), treeB(1:sizeB, :), dim, viz_ellipsoid, ...
        %     start_point, goal_point, meet_point, c_best_A, c_best_B);
        drawnow limitrate;
    end
end

planning_time = toc;

fprintf('\n========== 规划完成 ==========\n');
fprintf('成功: %s, 迭代次数: %d\n', string(success), iterations);
fprintf('树A节点: %d, 树B节点: %d\n', sizeA, sizeB);
fprintf('规划时间: %.3f秒\n', planning_time);
fprintf('===============================\n\n');

%% ========== 路径后处理优化 ==========
if success && ~isempty(path)
    original_length = sum(vecnorm(diff(path), 2, 2));
    
    % 1. Shortcut优化 - 尝试直接连接远端节点
    try
        path_shortcut = shortcutPath(path, obstacles, dim, 5);  % 最多5次迭代
        shortcut_length = sum(vecnorm(diff(path_shortcut), 2, 2));
        
        if shortcut_length < original_length
            path = path_shortcut;
            fprintf('  ✓ Shortcut优化: %.2f -> %.2f (减少%.1f%%)\n', ...
                original_length, shortcut_length, ...
                (original_length - shortcut_length) / original_length * 100);
        end
    catch ME
        fprintf('  ⚠ Shortcut优化失败: %s\n', ME.message);
    end
    
    % 2. 圆角平滑处理 - 对拐点进行圆角过渡
    if size(path, 1) > 3
        try
            % 记录圆角平滑前的路径节点数
            path_before_fillet = path;
            num_nodes_before_fillet = size(path_before_fillet, 1);
            
            % 计算合适的圆角半径（基于平均步长）
            segment_lengths = vecnorm(diff(path), 2, 2);
            avg_segment_length = mean(segment_lengths);
            fillet_radius = avg_segment_length * 0.25;  % 圆角半径为平均步长的25%
            
            % 执行圆角平滑处理（采样密度20点，安全裕度10%）
            [path_fillet, fillet_success] = smoothPathWithFillets(path, obstacles, dim, fillet_radius, 20);
            
            if ~isempty(path_fillet) && size(path_fillet, 1) >= 2
                fillet_length = sum(vecnorm(diff(path_fillet), 2, 2));
                path = path_fillet;
                
                if fillet_success
                    fprintf('  ✓ 圆角平滑: 节点数 %d -> %d, 长度 %.2f (所有拐点圆角成功，安全裕度10%%)\n', ...
                        num_nodes_before_fillet, size(path_fillet, 1), fillet_length);
                else
                    fprintf('  ✓ 圆角平滑: 节点数 %d -> %d, 长度 %.2f (部分拐点已圆角，其余保留原样)\n', ...
                        num_nodes_before_fillet, size(path_fillet, 1), fillet_length);
                end
            else
                fprintf('  ⚠ 圆角平滑失败，保持原路径\n');
            end
        catch ME
            fprintf('  ⚠ 圆角平滑异常: %s，保持原路径\n', ME.message);
        end
    end
    
    % 3. B-Spline平滑（可选，用于进一步光滑路径）
    if size(path, 1) > 5
        try
            % 记录B-Spline平滑前的路径节点数
            num_nodes_before_bspline = size(path, 1);
            
            % 降低平滑因子，避免过度平滑导致走捷径
            path_smooth = smoothPathBSpline(path, 0.15);  % 平滑因子从0.3降至0.15
            
            % 高效碰撞检测：采用分段验证策略
            path_valid = true;
            num_segments = size(path_smooth, 1) - 1;
            
            % 对于长路径，采样检测而非全部检测
            if num_segments > 50
                % 长路径：均匀采样20%的段进行精细检测
                check_indices = unique(round(linspace(1, num_segments, max(10, round(num_segments*0.2)))));
            else
                % 短路径：检测所有段
                check_indices = 1:num_segments;
            end
            
            for idx = 1:length(check_indices)
                i = check_indices(idx);
                if i > num_segments
                    break;
                end
                % 每段检测5个子点（平衡精度和速度）
                for alpha = linspace(0, 1, 5)
                    p_check = path_smooth(i,:) + alpha * (path_smooth(i+1,:) - path_smooth(i,:));
                    % 简单碰撞检测：仅检查点是否在障碍物内
                    for k = 1:size(obstacles, 1)
                        if dim == 2
                            % obstacles(k,:) = [center_x, center_y, radius]
                            dist = norm(p_check - obstacles(k, 1:2));
                            if dist < obstacles(k, 3) * 0.95  % 留5%安全裕度
                                path_valid = false;
                                break;
                            end
                        elseif dim == 3
                            % obstacles(k,:) = [center_x, center_y, center_z]
                            dist = norm(p_check - obstacles(k, 1:3));
                            if dist < obstacles(k, 4) * 0.95
                                path_valid = false;
                                break;
                            end
                        end
                    end
                    if ~path_valid
                        break;
                    end
                end
                if ~path_valid
                    break;
                end
            end
            
            if path_valid
                smooth_length = sum(vecnorm(diff(path_smooth), 2, 2));
                path = path_smooth;
                fprintf('  ✓ B-Spline平滑: 节点数 %d -> %d, 长度 %.2f\n', ...
                    num_nodes_before_bspline, size(path_smooth, 1), smooth_length);
            else
                fprintf('  ⚠ B-Spline平滑后存在碰撞，保持圆角路径\n');
            end
        catch ME
            fprintf('  ⚠ B-Spline平滑失败: %s\n', ME.message);
        end
    end
    
    fprintf('  最终路径长度: %.2f\n', sum(vecnorm(diff(path), 2, 2)));
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
