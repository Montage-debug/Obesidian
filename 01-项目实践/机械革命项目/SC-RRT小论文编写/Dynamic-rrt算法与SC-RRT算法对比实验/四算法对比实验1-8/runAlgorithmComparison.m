% =========================================================================
%                   RRT算法性能对比 - 统一主程序
% =========================================================================
% 【这是唯一的批量对比运行文件】
%
% 功能: 对比多个RRT算法变体的性能
% 支持算法:
%   1. RRT_Basic          - 原生RRT算法
%   2. RRT_Connect_Basic  - RRT-Connect算法 (双向快速连接)
%   3. RRT_Star_Basic     - RRT*算法 (渐进最优)
%   4. Informed_RRT_Star_Basic - Informed RRT*算法 (知情采样)
%   5. SC_RRT_Basic       - SC-RRT算法 (双向+AD CS+SSFOR+Pareto)
%   6. Dynamic_RRT_Basic  - Dynamic-RRT算法 (Informed+Pareto动态规划)
%
% 输出:
%   - Excel性能对比报告
%   - MAT数据文件
%   - 性能对比图表
%   - 各算法的可视化结果图
%
% 使用方法:
%   1. 直接运行 (默认参数): 
%      runAlgorithmComparison
%
%   2. 快速2D对比 (10次运行):
%      runAlgorithmComparison('NumRuns', 10, 'Dimension', '2D')
%
%   3. 仅对比RRT和RRT*:
%      runAlgorithmComparison('NumRuns', 10, 'Algorithms', {'RRT_Basic', 'RRT_Star_Basic'})
%
%   4. 高密度障碍物测试:
%      runAlgorithmComparison('NumRuns', 10, 'NumObstacles2D', 40)
%
% 作者: GitHub Copilot
% 日期: 2025-12-11
% =========================================================================
%
% 【参数说明】
%   'NumRuns'         - 每个算法运行次数 (默认: 1)
%   'Dimension'       - 测试维度 '2D', '3D', 'both' (默认: 'both')
%   'NumObstacles2D'  - 2D环境障碍物数量 (默认: 25)
%   'NumObstacles3D'  - 3D环境障碍物数量 (默认: 30)
%   'MaxIterations2D' - 2D最大迭代次数 (默认: 3000)
%   'MaxIterations3D' - 3D最大迭代次数 (默认: 5000)
%   'SaveResults'     - 是否保存结果 (默认: true)
%   'Visualize'       - 是否可视化 (默认: true)
%   'Algorithms'      - 算法列表 cell array (默认: 全部算法)
%
% 【示例】
%   % 示例1: 快速10次2D对比
%   runAlgorithmComparison('NumRuns', 10, 'Dimension', '2D')
%
%   % 示例2: 只对比RRT和RRT*
%   runAlgorithmComparison('NumRuns', 10, ...
%       'Algorithms', {'RRT_Basic', 'RRT_Star_Basic'})
%
%   % 示例3: 高密度障碍物测试
%   runAlgorithmComparison('NumRuns', 5, 'NumObstacles2D', 40)
%
% =========================================================================


function runAlgorithmComparison(varargin)
    %% ========== 添加算法子目录路径 ==========
    script_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(script_dir, 'sc_rrt'));
    addpath(fullfile(script_dir, 'dynamic_rrt'));
    
    %% ========== 参数解析 ==========
    p = inputParser;
    addParameter(p, 'NumRuns', 1, @isnumeric);              % 每个算法运行次数
    addParameter(p, 'Dimension', '3D', @ischar);          % '2D', '3D', 'both'
    addParameter(p, 'NumObstacles2D', 225, @isnumeric);      % 2D环境障碍物数量 (1500×1500环境)
    addParameter(p, 'NumObstacles3D', 400, @isnumeric);     % 3D环境障碍物数量 (1500³环境)
    addParameter(p, 'MaxIterations2D', 5000, @isnumeric);   % 2D最大迭代次数 (大环境需更多迭代)
    addParameter(p, 'MaxIterations3D', 8000, @isnumeric);   % 3D最大迭代次数 (大环境需更多迭代)
    addParameter(p, 'SaveResults', true, @islogical);       % 是否保存结果
    addParameter(p, 'Visualize', true, @islogical);         % 是否可视化
    addParameter(p, 'Algorithms', {}, @iscell);             % 算法列表（空则使用默认）
    parse(p, varargin{:});
    
    num_runs = p.Results.NumRuns;
    dimension = p.Results.Dimension;
    num_obs_2d = p.Results.NumObstacles2D;
    num_obs_3d = p.Results.NumObstacles3D;
    max_iter_2d = p.Results.MaxIterations2D;
    max_iter_3d = p.Results.MaxIterations3D;
    save_results = p.Results.SaveResults;
    visualize = p.Results.Visualize;
    
    %% ========== 打印标题 ==========
    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║              RRT算法性能对比实验                           ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('运行次数: %d\n', num_runs);
    fprintf('测试维度: %s\n', dimension);
    fprintf('保存结果: %s\n', iif(save_results, '是', '否'));
    fprintf('\n');
    
    %% ========== 创建性能收集器 ==========
    collector = PerformanceCollector();
    
    %% ========== 算法列表 ==========
    if isempty(p.Results.Algorithms)
        algorithms = {'RRT_Basic', 'RRT_Connect_Basic', 'RRT_Star_Basic', 'Informed_RRT_Star_Basic', 'SC_RRT_Basic', 'Dynamic_RRT_Basic'};
    else
        algorithms = p.Results.Algorithms;
    end
    
    %% ========== 2D环境测试 ==========
    if strcmp(dimension, '2D') || strcmp(dimension, 'both')
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
        fprintf('【2D环境测试】\n');
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');
        
        % 存储第一次运行结果用于合并可视化
        vis_data_2d = cell(length(algorithms), 1);
        vis_env_2d = [];
        
        for run = 1:num_runs
            fprintf('════════ 第 %d/%d 次运行 ════════\n', run, num_runs);
            
            % 生成环境
            fprintf('正在生成2D环境 (%d个障碍物)...\n', num_obs_2d);
            env = generate2DEnvironment([0 1500 0 1500], num_obs_2d, 'Visualize', false);
            fprintf('✓ 环境生成完成\n\n');
            if run == 1, vis_env_2d = env; end
            
            % 测试每个算法
            for i = 1:length(algorithms)
                algo_name = algorithms{i};
                fprintf('[%s] 运行中...\n', algo_name);
                
                try
                    [path, tree, success, metrics] = runSingleAlgorithm(algo_name, env, max_iter_2d);
                    
                    if success
                        fprintf('  >> 成功! 路径长度: %.2f, 时间: %.3fs\n', metrics.path_length, metrics.planning_time);
                        collector.addResult(algo_name, env, path, tree, success, metrics);
                        
                        % 存储第一次运行结果用于合并可视化
                        if run == 1
                            vis_data_2d{i} = struct('path', path, 'tree', tree, ...
                                'success', true, 'metrics', metrics, 'algo_name', algo_name);
                        end
                    else
                        fprintf('  ✗ 失败\n');
                        if run == 1
                            vis_data_2d{i} = struct('path', [], 'tree', struct('vertices',[],'parent',[]), ...
                                'success', false, 'metrics', struct(), 'algo_name', algo_name);
                        end
                    end
                catch ME
                    fprintf('  ✗ 错误: %s\n', ME.message);
                    if run == 1
                        vis_data_2d{i} = struct('path', [], 'tree', struct('vertices',[],'parent',[]), ...
                            'success', false, 'metrics', struct(), 'algo_name', algo_name);
                    end
                end
                fprintf('\n');
            end
        end
        
        % ========== 合并可视化: 2D结果放入一个窗口 ==========
        if visualize && ~isempty(vis_env_2d)
            plotCombinedResults(vis_data_2d, vis_env_2d, '2D');
        end
    end
    
    %% ========== 3D环境测试 ==========
    if strcmp(dimension, '3D') || strcmp(dimension, 'both')
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
        fprintf('【3D环境测试】\n');
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');
        
        % 存储第一次运行结果用于合并可视化
        vis_data_3d = cell(length(algorithms), 1);
        vis_env_3d = [];
        
        for run = 1:num_runs
            fprintf('════════ 第 %d/%d 次运行 ════════\n', run, num_runs);
            
            % 生成环境
            fprintf('正在生成3D环境 (%d个障碍物)...\n', num_obs_3d);
            env = generate3DEnvironment([0 1500 0 1500 0 1500], num_obs_3d, 'Visualize', false);
            fprintf('✓ 环境生成完成\n\n');
            if run == 1, vis_env_3d = env; end
            
            % 测试每个算法
            for i = 1:length(algorithms)
                algo_name = algorithms{i};
                fprintf('[%s] 运行中...\n', algo_name);
                
                try
                    [path, tree, success, metrics] = runSingleAlgorithm(algo_name, env, max_iter_3d);
                    
                    if success
                        fprintf('  >> 成功! 路径长度: %.2f, 时间: %.3fs\n', metrics.path_length, metrics.planning_time);
                        collector.addResult(algo_name, env, path, tree, success, metrics);
                        
                        % 存储第一次运行结果用于合并可视化
                        if run == 1
                            vis_data_3d{i} = struct('path', path, 'tree', tree, ...
                                'success', true, 'metrics', metrics, 'algo_name', algo_name);
                        end
                    else
                        fprintf('  ✗ 失败\n');
                        if run == 1
                            vis_data_3d{i} = struct('path', [], 'tree', struct('vertices',[],'parent',[]), ...
                                'success', false, 'metrics', struct(), 'algo_name', algo_name);
                        end
                    end
                catch ME
                    fprintf('  ✗ 错误: %s\n', ME.message);
                    if run == 1
                        vis_data_3d{i} = struct('path', [], 'tree', struct('vertices',[],'parent',[]), ...
                            'success', false, 'metrics', struct(), 'algo_name', algo_name);
                    end
                end
                fprintf('\n');
            end
        end
        
        % ========== 合并可视化: 3D结果放入一个窗口 ==========
        if visualize && ~isempty(vis_env_3d)
            plotCombinedResults(vis_data_3d, vis_env_3d, '3D');
        end
    end
    
    %% ========== 统计分析 ==========
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('【性能统计分析】\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    collector.printSummary();
    
    %% ========== 导出结果 ==========
    if save_results
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
        fprintf('【导出结果】\n');
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
        
        timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss'));
        
        % Excel报告
        excel_file = sprintf('results/performance_comparison_%s.xlsx', timestamp);
        collector.exportToExcel(excel_file);
        
        % MAT数据
        mat_file = sprintf('results/performance_data_%s.mat', timestamp);
        collector.saveToMAT(mat_file);
        
        % 对比图表
        chart_file = sprintf('results/comparison_charts_%s.png', timestamp);
        collector.plotComparison(chart_file);
        
        fprintf('\n');
    end
    
    %% ========== 总结 ==========
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║                  测试完成                                  ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('✓ 算法对比测试已完成!\n');
    fprintf('  总运行次数: %d\n', length(collector.results));
    fprintf('  算法数量: %d\n', length(collector.algorithms));
    if save_results
        fprintf('\n📊 结果文件:\n');
        fprintf('  Excel: %s\n', excel_file);
        fprintf('  MAT:   %s\n', mat_file);
        fprintf('  图表:  %s\n', chart_file);
    end
    fprintf('\n');
end

%% ========== 辅助函数 ==========
function [path, tree, success, metrics] = runSingleAlgorithm(algo_name, env, max_iter)
    % 运行单个算法
    
    switch algo_name
        case 'RRT_Basic'
            tic;
            [path, tree, success] = RRT_Basic(env, max_iter);
            time_elapsed = toc;
            
            % 手动计算metrics
            metrics = struct();
            metrics.iterations = max_iter;
            metrics.tree_nodes = size(tree.vertices, 1);
            metrics.planning_time = time_elapsed;
            
            if success && ~isempty(path)
                metrics.path_length = tree.final_cost;
            else
                metrics.path_length = inf;
            end
            
            metrics.smoothness = calculateSmoothness(path);
            metrics.clearance = NaN;
            
        case 'RRT_Connect_Basic'
            tic;
            [path, tree, success] = RRT_Connect_Basic(env, max_iter);
            time_elapsed = toc;
            
            % 手动计算metrics
            metrics = struct();
            metrics.iterations = max_iter;
            metrics.tree_nodes = size(tree.vertices, 1);
            metrics.planning_time = time_elapsed;
            
            if success && ~isempty(path)
                metrics.path_length = tree.final_cost;
            else
                metrics.path_length = inf;
            end
            
            metrics.smoothness = calculateSmoothness(path);
            metrics.clearance = NaN;
            
        case 'RRT_Star_Basic'
            tic;
            [path, tree, success] = RRT_Star_Basic(env, max_iter);
            time_elapsed = toc;
            
            % 手动计算metrics
            metrics = struct();
            metrics.iterations = max_iter;
            metrics.tree_nodes = size(tree.vertices, 1);
            metrics.planning_time = time_elapsed;
            
            if success && ~isempty(path)
                metrics.path_length = tree.final_cost;
            else
                metrics.path_length = inf;
            end
            
            metrics.smoothness = calculateSmoothness(path);
            metrics.clearance = NaN;
            
        case 'SC_RRT_Basic'
            [path, tree, success, metrics] = SC_RRT_Basic(env, max_iter);
            
        case 'Dynamic_RRT_Basic'
            [path, tree, success, metrics] = Dynamic_RRT_Basic(env, max_iter);
            
        case 'Informed_RRT_Star_Basic'
            [path, tree, success, metrics] = Informed_RRT_Star_Basic(env, max_iter);
            
        otherwise
            error('未知算法: %s', algo_name);
    end
    
    % ========== 增强metrics计算 - 为所有算法添加新指标 ==========
    if success && ~isempty(path)
        dim = env.dimension;
        if ischar(dim) || isstring(dim)
            if contains(dim, '2D') || contains(dim, '2d')
                dim = 2;
            else
                dim = 3;
            end
        end
        
        % 重新计算路径长度（确保准确）
        if ~isfield(metrics, 'path_length') || isnan(metrics.path_length) || isinf(metrics.path_length)
            metrics.path_length = sum(vecnorm(diff(path), 2, 2));
        end
        
        % 新增指标1: 路径效率 (直线距离/实际路径长度)
        if isfield(env, 'start') && isfield(env, 'goal')
            straight_dist = norm(env.goal - env.start);
        elseif isfield(env, 'start_point') && isfield(env, 'goal_point')
            straight_dist = norm(env.goal_point - env.start_point);
        else
            straight_dist = norm(path(end,:) - path(1,:));
        end
        metrics.path_efficiency = straight_dist / (metrics.path_length + eps);
        
        % 新增指标2: 节点利用率
        metrics.node_efficiency = size(path, 1) / metrics.tree_nodes;
        
        % 新增指标3: 平均转角和最大转角
        if size(path, 1) >= 3
            angles = zeros(size(path, 1) - 2, 1);
            for i = 2:size(path, 1)-1
                v1 = path(i, :) - path(i-1, :);
                v2 = path(i+1, :) - path(i, :);
                cos_angle = dot(v1, v2) / (norm(v1) * norm(v2) + 1e-10);
                cos_angle = max(-1, min(1, cos_angle));
                angles(i-1) = acos(cos_angle) * 180 / pi;  % 转为角度
            end
            metrics.avg_turning_angle = mean(angles);
            metrics.max_turning_angle = max(angles);
        else
            metrics.avg_turning_angle = 0;
            metrics.max_turning_angle = 0;
        end
        
        % 新增指标4: 路径密度 (节点/长度)
        metrics.path_density = size(path, 1) / (metrics.path_length + eps);
        
        % 新增指标5: 最小安全间隙和平均间隙
        if isfield(env, 'obstacles') && ~isempty(env.obstacles)
            obstacles = env.obstacles;
            min_clearances = inf(size(path, 1), 1);
            for i = 1:size(path, 1)
                if dim == 2
                    dists = vecnorm(obstacles(:, 1:2) - path(i, :), 2, 2) - obstacles(:, 3);
                else
                    dists = vecnorm(obstacles(:, 1:3) - path(i, :), 2, 2) - obstacles(:, 4);
                end
                min_clearances(i) = min(dists);
            end
            metrics.min_clearance = min(min_clearances);
            metrics.avg_clearance = mean(min_clearances);
        else
            metrics.min_clearance = inf;
            metrics.avg_clearance = inf;
        end
    else
        % 失败时设置默认值
        metrics.path_efficiency = 0;
        metrics.node_efficiency = 0;
        metrics.avg_turning_angle = NaN;
        metrics.max_turning_angle = NaN;
        metrics.path_density = 0;
        metrics.min_clearance = 0;
        metrics.avg_clearance = 0;
    end
end

function smoothness = calculateSmoothness(path)
    % 计算路径平滑度 (角度变化的标准差)
    if isempty(path) || size(path, 1) < 3
        smoothness = NaN;
        return;
    end
    
    angles = [];
    for i = 2:size(path, 1)-1
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        
        cos_angle = dot(v1, v2) / (norm(v1) * norm(v2) + eps);
        cos_angle = max(-1, min(1, cos_angle));
        angle = acos(cos_angle);
        angles = [angles; angle];
    end
    
    smoothness = std(angles);
end

function result = iif(condition, true_val, false_val)
    % 简单的三元运算符实现
    if condition
        result = true_val;
    else
        result = false_val;
    end
end

function plotCombinedResults(vis_data, env, dim_str)
% plotCombinedResults - 将所有算法结果绘制到一个figure的subplot中
%
% 输入:
%   vis_data  - cell array，每个元素包含一个算法的结果
%   env       - 环境结构体
%   dim_str   - '2D' 或 '3D'

    % 算法颜色和显示名称
    algo_colors = struct('RRT_Basic', [0.85 0.33 0.10], ...
                         'RRT_Connect_Basic', [0.93 0.69 0.13], ...
                         'RRT_Star_Basic', [0.47 0.67 0.19], ...
                         'Informed_RRT_Star_Basic', [0.56 0.27 0.52], ...
                         'SC_RRT_Basic', [0.00 0.45 0.74], ...
                         'Dynamic_RRT_Basic', [1.00 0.00 0.00]);
    
    algo_display = struct('RRT_Basic', 'RRT', ...
                          'RRT_Connect_Basic', 'RRT-Connect', ...
                          'RRT_Star_Basic', 'RRT*', ...
                          'Informed_RRT_Star_Basic', 'Informed-RRT*', ...
                          'SC_RRT_Basic', 'SC-RRT', ...
                          'Dynamic_RRT_Basic', 'Dynamic-RRT');
    
    num_alg = length(vis_data);
    valid_count = sum(~cellfun(@isempty, vis_data));
    
    if valid_count <= 4
        nrows = 2; ncols = 2;
    elseif valid_count <= 6
        nrows = 2; ncols = 3;
    else
        nrows = 3; ncols = 3;
    end
    
    is_2d = strcmp(dim_str, '2D');
    
    fig = figure('Name', sprintf('%s 路径规划结果对比', dim_str), ...
                 'Position', [30+30*~is_2d 50+30*~is_2d ncols*480 nrows*420], 'Color', 'w');
    
    plot_idx = 0;
    for idx = 1:num_alg
        if isempty(vis_data{idx}), continue; end
        
        res = vis_data{idx};
        algo_id = res.algo_name;
        
        % 获取显示名称和颜色
        if isfield(algo_display, algo_id)
            display_name = algo_display.(algo_id);
        else
            display_name = strrep(algo_id, '_', '-');
        end
        if isfield(algo_colors, algo_id)
            alg_color = algo_colors.(algo_id);
        else
            alg_color = [0 0 1];
        end
        
        plot_idx = plot_idx + 1;
        subplot(nrows, ncols, plot_idx);
        hold on; axis equal; grid on;
        
        if is_2d
            xlim([env.bounds(1), env.bounds(2)]);
            ylim([env.bounds(3), env.bounds(4)]);
            xlabel('X (mm)', 'FontSize', 9);
            ylabel('Y (mm)', 'FontSize', 9);
            
            % 绘制障碍物
            for k = 1:env.num
                obs = env.obstacles(k, :);
                rectangle('Position', [obs(1)-obs(3), obs(2)-obs(3), 2*obs(3), 2*obs(3)], ...
                         'Curvature', [1 1], 'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'none');
            end
            
            % 绘制起终点
            plot(env.start_point(1), env.start_point(2), 'go', ...
                 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
            plot(env.goal_point(1), env.goal_point(2), 'rs', ...
                 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
            
            if res.success && ~isempty(res.path)
                plot(res.path(:,1), res.path(:,2), 'Color', alg_color, 'LineWidth', 2.5);
                plan_time = getFieldSafe(res.metrics, 'planning_time', NaN);
                path_len = sum(vecnorm(diff(res.path), 2, 2));
                num_nodes = getNodeCountVis(res.tree);
                smooth_val = getFieldSafe(res.metrics, 'smoothness', NaN);
                title(sprintf('%s\nTime:%.3fs | Len:%.1fmm | Nodes:%d | Smooth:%.4f', ...
                    display_name, plan_time, path_len, num_nodes, smooth_val), ...
                    'FontSize', 10, 'FontWeight', 'bold');
            else
                title(sprintf('%s\n(规划失败)', display_name), ...
                    'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
            end
        else  % 3D
            view(45, 30);
            xlim([env.bounds(1), env.bounds(2)]);
            ylim([env.bounds(3), env.bounds(4)]);
            zlim([env.bounds(5), env.bounds(6)]);
            xlabel('X', 'FontSize', 9);
            ylabel('Y', 'FontSize', 9);
            zlabel('Z', 'FontSize', 9);
            
            [Xs, Ys, Zs] = sphere(8);
            for k = 1:env.num
                surf(Xs*env.obstacles(k,4)+env.obstacles(k,1), ...
                     Ys*env.obstacles(k,4)+env.obstacles(k,2), ...
                     Zs*env.obstacles(k,4)+env.obstacles(k,3), ...
                     'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
            end
            
            plot3(env.start_point(1), env.start_point(2), env.start_point(3), 'go', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
            plot3(env.goal_point(1), env.goal_point(2), env.goal_point(3), 'rs', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
            
            if res.success && ~isempty(res.path) && size(res.path, 2) >= 3
                plot3(res.path(:,1), res.path(:,2), res.path(:,3), ...
                      'Color', alg_color, 'LineWidth', 2.5);
                plan_time = getFieldSafe(res.metrics, 'planning_time', NaN);
                path_len = sum(vecnorm(diff(res.path), 2, 2));
                num_nodes = getNodeCountVis(res.tree);
                smooth_val = getFieldSafe(res.metrics, 'smoothness', NaN);
                title(sprintf('%s\nTime:%.3fs | Len:%.1fmm | Nodes:%d | Smooth:%.4f', ...
                    display_name, plan_time, path_len, num_nodes, smooth_val), ...
                    'FontSize', 10, 'FontWeight', 'bold');
            else
                title(sprintf('%s\n(规划失败)', display_name), ...
                    'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
            end
            lighting gouraud;
            camlight('headlight');
        end
        set(gca, 'FontSize', 9);
    end
    
    sgtitle(sprintf('%s 路径规划结果对比', dim_str), 'FontSize', 16, 'FontWeight', 'bold');
    drawnow;
    
    % 保存图片
    save_dir = 'results';
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    saveas(fig, fullfile(save_dir, sprintf('combined_%s_%s.png', dim_str, timestamp)));
    print(fig, fullfile(save_dir, sprintf('combined_%s_%s_hires.png', dim_str, timestamp)), '-dpng', '-r300');
    fprintf('>> %s合并对比图已保存至 %s/\n', dim_str, save_dir);
end

function val = getFieldSafe(s, field_name, default_val)
    if isfield(s, field_name)
        val = s.(field_name);
    else
        val = default_val;
    end
end

function nodeCount = getNodeCountVis(tree)
    if isfield(tree, 'vertices') && ~isempty(tree.vertices)
        nodeCount = size(tree.vertices, 1);
    elseif isfield(tree, 'nodes') && ~isempty(tree.nodes)
        nodeCount = size(tree.nodes, 1);
    else
        nodeCount = 0;
    end
end
