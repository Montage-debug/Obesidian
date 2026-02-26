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
%   4. SC_RRT_Basic       - SC-RRT算法 (可供优化)
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
    %% ========== 添加SC-RRT路径 ==========
    script_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(script_dir, 'sc_rrt'));
    
    %% ========== 参数解析 ==========
    p = inputParser;
    addParameter(p, 'NumRuns', 1, @isnumeric);              % 每个算法运行次数
    addParameter(p, 'Dimension', 'both', @ischar);          % '2D', '3D', 'both'
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
        algorithms = {'RRT_Basic', 'RRT_Connect_Basic', 'RRT_Star_Basic', 'SC_RRT_Basic'};
        % algorithms = {'SC_RRT_Basic'};
    else
        algorithms = p.Results.Algorithms;
    end
    
    %% ========== 2D环境测试 ==========
    if strcmp(dimension, '2D') || strcmp(dimension, 'both')
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
        fprintf('【2D环境测试】\n');
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');
        
        for run = 1:num_runs
            fprintf('════════ 第 %d/%d 次运行 ════════\n', run, num_runs);
            
            % 生成环境
            fprintf('正在生成2D环境 (%d个障碍物)...\n', num_obs_2d);
            env = generate2DEnvironment([0 1500 0 1500], num_obs_2d, 'Visualize', false);
            fprintf('✓ 环境生成完成\n\n');
            
            % 测试每个算法
            for i = 1:length(algorithms)
                algo_name = algorithms{i};
                fprintf('[%s] 运行中...\n', algo_name);
                
                try
                    [path, tree, success, metrics] = runSingleAlgorithm(algo_name, env, max_iter_2d);
                    
                    if success
                        fprintf('  ✓ 成功! 路径长度: %.2f, 时间: %.3fs\n', metrics.path_length, metrics.planning_time);
                        collector.addResult(algo_name, env, path, tree, success, metrics);
                        
                        if visualize && run == 1  % 只可视化第一次运行
                            visualizeAndSaveRRTResult(env, path, tree, success, algo_name, 'results');
                        end
                    else
                        fprintf('  ✗ 失败\n');
                    end
                catch ME
                    fprintf('  ✗ 错误: %s\n', ME.message);
                end
                fprintf('\n');
            end
        end
    end
    
    %% ========== 3D环境测试 ==========
    if strcmp(dimension, '3D') || strcmp(dimension, 'both')
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
        fprintf('【3D环境测试】\n');
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');
        
        for run = 1:num_runs
            fprintf('════════ 第 %d/%d 次运行 ════════\n', run, num_runs);
            
            % 生成环境
            fprintf('正在生成3D环境 (%d个障碍物)...\n', num_obs_3d);
            env = generate3DEnvironment([0 1500 0 1500 0 1500], num_obs_3d, 'Visualize', false);
            fprintf('✓ 环境生成完成\n\n');
            
            % 测试每个算法
            for i = 1:length(algorithms)
                algo_name = algorithms{i};
                fprintf('[%s] 运行中...\n', algo_name);
                
                try
                    [path, tree, success, metrics] = runSingleAlgorithm(algo_name, env, max_iter_3d);
                    
                    if success
                        fprintf('  ✓ 成功! 路径长度: %.2f, 时间: %.3fs\n', metrics.path_length, metrics.planning_time);
                        collector.addResult(algo_name, env, path, tree, success, metrics);
                        
                        if visualize && run == 1  % 只可视化第一次运行
                            visualizeAndSaveRRTResult(env, path, tree, success, algo_name, 'results');
                        end
                    else
                        fprintf('  ✗ 失败\n');
                    end
                catch ME
                    fprintf('  ✗ 错误: %s\n', ME.message);
                end
                fprintf('\n');
            end
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
