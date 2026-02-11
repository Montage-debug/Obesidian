% =========================================================================
%     Dynamic RRT vs SC-RRT 综合对比实验系统
% =========================================================================
% 目的: 对比Dynamic RRT和SC-RRT算法性能，为SCI论文提供实验数据
%
% 对比算法:
%   1. Dynamic RRT (单树, informed subset + Pareto)
%   2. SC-RRT Basic (双树基础版)
%   3. SC-RRT with PID (双树+PID控制)
%   4. SC-RRT Adaptive (双树+PID+Pareto, 完整版)
%
% 评估指标:
%   - 收敛时间 (Convergence Time): 首次找到可行路径的时间
%   - 路径长度 (Path Length): 路径总长度
%   - 路径平滑度 (Path Smoothness): 转角变化率
%   - 搜索效率 (Search Efficiency): 节点数/路径长度比
%   - 成功率 (Success Rate): 多次运行的成功比例
%   - 计算复杂度 (Nodes Count): 树中节点总数
%
% 环境配置: 按照论文标准 (1500×1500, 225障碍物)
% =========================================================================
clear; clc; close all;

%% =================== 实验配置 ===================
experiment_config = struct();

% 测试环境 (使用2D固定障碍物环境保证可复现)
experiment_config.dimension = '2D';
experiment_config.bounds = [0 1500 0 1500];
experiment_config.startPoint = [400 400];
experiment_config.goalPoint = [1100 1100];
experiment_config.numObstacles = 225;
experiment_config.obstacleRadius = 15;
experiment_config.seed = 42;  % 固定种子

% 实验参数
experiment_config.num_runs = 100;  % 每个算法运行次数 (论文标准100次，这里先30次)
experiment_config.max_iterations = 10000;
experiment_config.time_limit = 10;  % 时间限制(秒)

% 保存结果
experiment_config.save_results = true;
experiment_config.save_path = 'comparison_results';

% 可视化设置
experiment_config.show_progress = true;  % 显示进度
experiment_config.plot_results = true;   % 绘制对比图

fprintf('\n========================================\n');
fprintf('Dynamic RRT vs SC-RRT 对比实验\n');
fprintf('========================================\n');
fprintf('环境: %s 固定障碍物\n', experiment_config.dimension);
fprintf('空间: [%d %d] × [%d %d]\n', experiment_config.bounds);
fprintf('障碍物: %d个 (半径%d)\n', experiment_config.numObstacles, experiment_config.obstacleRadius);
fprintf('起点: (%.0f, %.0f)\n', experiment_config.startPoint);
fprintf('终点: (%.0f, %.0f)\n', experiment_config.goalPoint);
fprintf('运行次数: %d\n', experiment_config.num_runs);
fprintf('========================================\n\n');

%% =================== 生成障碍物环境 ===================
fprintf('生成障碍物环境...\n');

% 检查是否有generateObstacles函数（SC-RRT的）
sc_rrt_path = '../SC-RRT独立算法实现/copilot-1.6';
if exist(fullfile(sc_rrt_path, 'generateObstacles.m'), 'file')
    addpath(sc_rrt_path);
    fprintf('使用SC-RRT的generateObstacles函数\n');
    rng(experiment_config.seed);
    obstacles = generateObstacles(experiment_config.dimension, ...
                                 experiment_config.bounds, ...
                                 experiment_config.numObstacles, ...
                                 experiment_config.obstacleRadius, ...
                                 experiment_config.startPoint, ...
                                 experiment_config.goalPoint);
else
    % 使用Dynamic RRT的GenerateObstacles
    fprintf('使用Dynamic RRT的GenerateObstacles函数\n');
    obstacles = GenerateObstacles(experiment_config.dimension, ...
                                 experiment_config.bounds, ...
                                 experiment_config.numObstacles, ...
                                 experiment_config.obstacleRadius, ...
                                 experiment_config.startPoint, ...
                                 experiment_config.goalPoint, ...
                                 experiment_config.seed);
end

fprintf('障碍物生成完成: %d个\n\n', size(obstacles.circles, 1));

%% =================== 定义测试算法 ===================
algorithms = {};

% 1. Dynamic RRT (Interval=4, 论文推荐)
algorithms{1} = struct(...
    'name', 'Dynamic-RRT (Interval=4)', ...
    'short_name', 'DRRT-4', ...
    'type', 'dynamic_rrt', ...
    'params', struct('Interval', 4, 'ParetoProb', 0.1), ...
    'color', [0.85 0.33 0.10], ...  % 橙红色
    'marker', 'o' ...
);

% 2. Dynamic RRT (Interval=8, 更保守)
algorithms{2} = struct(...
    'name', 'Dynamic-RRT (Interval=8)', ...
    'short_name', 'DRRT-8', ...
    'type', 'dynamic_rrt', ...
    'params', struct('Interval', 8, 'ParetoProb', 0.1), ...
    'color', [0.93 0.69 0.13], ...  % 金色
    'marker', 's' ...
);

% 3. SC-RRT Basic (无PID无Pareto)
algorithms{3} = struct(...
    'name', 'SC-RRT Basic', ...
    'short_name', 'SC-Basic', ...
    'type', 'sc_rrt', ...
    'params', struct('Mode', 'basic', 'UseParetoFrontier', false), ...
    'color', [0.47 0.67 0.19], ...  % 绿色
    'marker', '^' ...
);

% 4. SC-RRT with PID (有PID无Pareto)
algorithms{4} = struct(...
    'name', 'SC-RRT with PID', ...
    'short_name', 'SC-PID', ...
    'type', 'sc_rrt', ...
    'params', struct('Mode', 'pid', 'UseParetoFrontier', false), ...
    'color', [0.30 0.75 0.93], ...  % 天蓝色
    'marker', 'd' ...
);

% 5. SC-RRT Adaptive (完整版: PID+Pareto)
algorithms{5} = struct(...
    'name', 'SC-RRT Adaptive', ...
    'short_name', 'SC-Adaptive', ...
    'type', 'sc_rrt', ...
    'params', struct('Mode', 'adaptive', 'UseParetoFrontier', true), ...
    'color', [0.00 0.45 0.74], ...  % 深蓝色
    'marker', 'p' ...
);

num_algorithms = length(algorithms);

fprintf('待测试算法:\n');
for i = 1:num_algorithms
    fprintf('  %d. %s\n', i, algorithms{i}.name);
end
fprintf('\n');

%% =================== 运行对比实验 ===================
results = cell(num_algorithms, 1);

for alg_idx = 1:num_algorithms
    alg = algorithms{alg_idx};
    
    fprintf('========================================\n');
    fprintf('测试算法 %d/%d: %s\n', alg_idx, num_algorithms, alg.name);
    fprintf('========================================\n');
    
    % 初始化结果存储
    results{alg_idx} = struct();
    results{alg_idx}.algorithm = alg;
    results{alg_idx}.convergence_times = nan(experiment_config.num_runs, 1);
    results{alg_idx}.path_lengths = nan(experiment_config.num_runs, 1);
    results{alg_idx}.path_smoothness = nan(experiment_config.num_runs, 1);
    results{alg_idx}.node_counts = nan(experiment_config.num_runs, 1);
    results{alg_idx}.search_efficiency = nan(experiment_config.num_runs, 1);
    results{alg_idx}.success_count = 0;
    
    % 多次运行
    for run = 1:experiment_config.num_runs
        if experiment_config.show_progress
            fprintf('  Run %d/%d ... ', run, experiment_config.num_runs);
        end
        
        % 设置不同的随机种子（但障碍物环境相同）
        rng(experiment_config.seed + run);
        
        try
            if strcmp(alg.type, 'dynamic_rrt')
                % 运行Dynamic RRT
                [tree, path, success, metrics] = DynamicRRT(...
                    experiment_config.startPoint, ...
                    experiment_config.goalPoint, ...
                    experiment_config.bounds, ...
                    obstacles, ...
                    'MaxIterations', experiment_config.max_iterations, ...
                    'Interval', alg.params.Interval, ...
                    'ParetoProb', alg.params.ParetoProb, ...
                    'EnableVisualization', false);
                
                if success
                    results{alg_idx}.convergence_times(run) = metrics.convergenceTime;
                    results{alg_idx}.path_lengths(run) = metrics.pathLength;
                    results{alg_idx}.node_counts(run) = metrics.nodeCount;
                    results{alg_idx}.path_smoothness(run) = calculateSmoothness(path);
                    results{alg_idx}.search_efficiency(run) = metrics.nodeCount / metrics.pathLength;
                    results{alg_idx}.success_count = results{alg_idx}.success_count + 1;
                    
                    if experiment_config.show_progress
                        fprintf('成功 (%.4fs, %.2f)\n', metrics.convergenceTime, metrics.pathLength);
                    end
                else
                    if experiment_config.show_progress
                        fprintf('失败\n');
                    end
                end
                
            elseif strcmp(alg.type, 'sc_rrt')
                % 运行SC-RRT
                % 创建临时图形（SC-RRT需要）
                fig_temp = figure('Visible', 'off');
                
                [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...
                    experiment_config.startPoint, ...
                    experiment_config.goalPoint, ...
                    experiment_config.bounds, ...
                    obstacles, ...
                    fig_temp, '', 0, 0, ...
                    'Mode', alg.params.Mode, ...
                    'MaxIterations', experiment_config.max_iterations, ...
                    'UseParetoFrontier', alg.params.UseParetoFrontier, ...
                    'EnableVisualization', false);
                
                close(fig_temp);
                
                if success && ~isempty(path)
                    % 处理字段名兼容性 - computationTime vs planningTime
                    if isfield(metrics, 'computationTime')
                        planning_time = metrics.computationTime;
                    elseif isfield(metrics, 'planningTime')
                        planning_time = metrics.planningTime;
                    else
                        planning_time = NaN;
                    end
                    
                    % 处理字段名兼容性 - pathLength vs finalPathLength
                    if isfield(metrics, 'pathLength')
                        path_length = metrics.pathLength;
                    elseif isfield(metrics, 'finalPathLength')
                        path_length = metrics.finalPathLength;
                    else
                        path_length = calculatePathLength(path);
                    end
                    
                    results{alg_idx}.convergence_times(run) = planning_time;
                    results{alg_idx}.path_lengths(run) = path_length;
                    results{alg_idx}.node_counts(run) = treeA.count + treeB.count;
                    results{alg_idx}.path_smoothness(run) = calculateSmoothness(path);
                    results{alg_idx}.search_efficiency(run) = (treeA.count + treeB.count) / path_length;
                    results{alg_idx}.success_count = results{alg_idx}.success_count + 1;
                    
                    if experiment_config.show_progress
                        fprintf('成功 (%.4fs, %.2f)\n', planning_time, path_length);
                    end
                else
                    if experiment_config.show_progress
                        fprintf('失败\n');
                    end
                end
            end
            
        catch ME
            fprintf('错误: %s\n', ME.message);
        end
    end
    
    % 计算统计数据
    valid_idx = ~isnan(results{alg_idx}.convergence_times);
    results{alg_idx}.success_rate = results{alg_idx}.success_count / experiment_config.num_runs;
    
    if results{alg_idx}.success_count > 0
        results{alg_idx}.mean_time = mean(results{alg_idx}.convergence_times(valid_idx));
        results{alg_idx}.std_time = std(results{alg_idx}.convergence_times(valid_idx));
        results{alg_idx}.mean_length = mean(results{alg_idx}.path_lengths(valid_idx));
        results{alg_idx}.std_length = std(results{alg_idx}.path_lengths(valid_idx));
        results{alg_idx}.mean_smoothness = mean(results{alg_idx}.path_smoothness(valid_idx));
        results{alg_idx}.std_smoothness = std(results{alg_idx}.path_smoothness(valid_idx));
        results{alg_idx}.mean_nodes = mean(results{alg_idx}.node_counts(valid_idx));
        results{alg_idx}.std_nodes = std(results{alg_idx}.node_counts(valid_idx));
        results{alg_idx}.mean_efficiency = mean(results{alg_idx}.search_efficiency(valid_idx));
    else
        results{alg_idx}.mean_time = NaN;
        results{alg_idx}.std_time = NaN;
        results{alg_idx}.mean_length = NaN;
        results{alg_idx}.std_length = NaN;
        results{alg_idx}.mean_smoothness = NaN;
        results{alg_idx}.std_smoothness = NaN;
        results{alg_idx}.mean_nodes = NaN;
        results{alg_idx}.std_nodes = NaN;
        results{alg_idx}.mean_efficiency = NaN;
    end
    
    fprintf('\n汇总:\n');
    fprintf('  成功率: %.1f%%\n', results{alg_idx}.success_rate * 100);
    if results{alg_idx}.success_count > 0
        fprintf('  收敛时间: %.4f ± %.4f 秒\n', results{alg_idx}.mean_time, results{alg_idx}.std_time);
        fprintf('  路径长度: %.2f ± %.2f\n', results{alg_idx}.mean_length, results{alg_idx}.std_length);
        fprintf('  平滑度指数: %.4f ± %.4f\n', results{alg_idx}.mean_smoothness, results{alg_idx}.std_smoothness);
        fprintf('  节点数: %.1f ± %.1f\n', results{alg_idx}.mean_nodes, results{alg_idx}.std_nodes);
        fprintf('  搜索效率: %.4f\n', results{alg_idx}.mean_efficiency);
    end
    fprintf('\n');
end

%% =================== 生成对比表格 ===================
fprintf('\n========================================\n');
fprintf('对比结果汇总表\n');
fprintf('========================================\n\n');

% 表头
fprintf('%-20s | %-12s | %-18s | %-18s | %-18s | %-15s | %-18s\n', ...
    '算法', '成功率(%)', '收敛时间(s)', '路径长度', '平滑度指数', '节点数', '搜索效率');
fprintf('%s\n', repmat('-', 160, 1));

% 数据行
for i = 1:num_algorithms
    r = results{i};
    if r.success_count > 0
        fprintf('%-20s | %11.1f%% | %8.4f ± %.4f | %8.2f ± %.2f | %8.4f ± %.4f | %7.1f ± %.1f | %.4f\n', ...
            r.algorithm.short_name, ...
            r.success_rate * 100, ...
            r.mean_time, r.std_time, ...
            r.mean_length, r.std_length, ...
            r.mean_smoothness, r.std_smoothness, ...
            r.mean_nodes, r.std_nodes, ...
            r.mean_efficiency);
    else
        fprintf('%-20s | %11.1f%% | %18s | %18s | %18s | %15s | %18s\n', ...
            r.algorithm.short_name, 0, 'N/A', 'N/A', 'N/A', 'N/A', 'N/A');
    end
end

fprintf('\n注: 平滑度指数越小表示路径越平滑\n');
fprintf('    搜索效率 = 节点数/路径长度，越小表示效率越高\n\n');

%% =================== 保存结果 ===================
if experiment_config.save_results
    if ~exist(experiment_config.save_path, 'dir')
        mkdir(experiment_config.save_path);
    end
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    save_file = fullfile(experiment_config.save_path, sprintf('comparison_results_%s.mat', timestamp));
    save(save_file, 'results', 'experiment_config', 'obstacles');
    fprintf('结果已保存到: %s\n\n', save_file);
end

%% =================== 可视化对比 ===================
if experiment_config.plot_results
    plotComparisonResults(results, experiment_config);
end

fprintf('========================================\n');
fprintf('实验完成!\n');
fprintf('========================================\n');

%% =================== 辅助函数 ===================

function smoothness = calculateSmoothness(path)
% 计算路径平滑度 (角度变化率的标准差)
% 平滑度越小，路径越平滑
    if size(path, 1) < 3
        smoothness = 0;
        return;
    end
    
    angles = zeros(size(path, 1) - 2, 1);
    for i = 2:size(path, 1)-1
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        
        % 计算转角
        cos_angle = dot(v1, v2) / (norm(v1) * norm(v2) + 1e-10);
        cos_angle = max(-1, min(1, cos_angle));  % 防止数值误差
        angles(i-1) = acos(cos_angle);
    end
    
    % 使用角度变化的标准差作为平滑度指标
    smoothness = std(angles);
end

function plotComparisonResults(results, config)
% 绘制对比结果图表
    num_alg = length(results);
    
    % 提取数据
    names = cell(num_alg, 1);
    success_rates = zeros(num_alg, 1);
    mean_times = zeros(num_alg, 1);
    std_times = zeros(num_alg, 1);
    mean_lengths = zeros(num_alg, 1);
    std_lengths = zeros(num_alg, 1);
    mean_smoothness = zeros(num_alg, 1);
    mean_nodes = zeros(num_alg, 1);
    colors = zeros(num_alg, 3);
    
    for i = 1:num_alg
        names{i} = results{i}.algorithm.short_name;
        success_rates(i) = results{i}.success_rate;
        mean_times(i) = results{i}.mean_time;
        std_times(i) = results{i}.std_time;
        mean_lengths(i) = results{i}.mean_length;
        std_lengths(i) = results{i}.std_length;
        mean_smoothness(i) = results{i}.mean_smoothness;
        mean_nodes(i) = results{i}.mean_nodes;
        colors(i, :) = results{i}.algorithm.color;
    end
    
    % 创建对比图
    fig = figure('Name', 'Algorithm Comparison', 'Position', [50 50 1400 900]);
    
    % 1. 收敛时间对比
    subplot(2, 3, 1);
    hold on; grid on;
    for i = 1:num_alg
        if ~isnan(mean_times(i))
            bar(i, mean_times(i), 'FaceColor', colors(i, :), 'EdgeColor', 'k');
            errorbar(i, mean_times(i), std_times(i), 'k', 'LineWidth', 1.5);
        end
    end
    set(gca, 'XTick', 1:num_alg, 'XTickLabel', names, 'XTickLabelRotation', 45);
    ylabel('收敛时间 (秒)');
    title('(a) 收敛时间对比');
    set(gca, 'FontSize', 10);
    
    % 2. 路径长度对比
    subplot(2, 3, 2);
    hold on; grid on;
    for i = 1:num_alg
        if ~isnan(mean_lengths(i))
            bar(i, mean_lengths(i), 'FaceColor', colors(i, :), 'EdgeColor', 'k');
            errorbar(i, mean_lengths(i), std_lengths(i), 'k', 'LineWidth', 1.5);
        end
    end
    set(gca, 'XTick', 1:num_alg, 'XTickLabel', names, 'XTickLabelRotation', 45);
    ylabel('路径长度');
    title('(b) 路径长度对比');
    set(gca, 'FontSize', 10);
    
    % 3. 成功率对比
    subplot(2, 3, 3);
    hold on; grid on;
    for i = 1:num_alg
        bar(i, success_rates(i)*100, 'FaceColor', colors(i, :), 'EdgeColor', 'k');
    end
    set(gca, 'XTick', 1:num_alg, 'XTickLabel', names, 'XTickLabelRotation', 45);
    ylabel('成功率 (%)');
    ylim([0 105]);
    title('(c) 成功率对比');
    set(gca, 'FontSize', 10);
    
    % 4. 路径平滑度对比
    subplot(2, 3, 4);
    hold on; grid on;
    for i = 1:num_alg
        if ~isnan(mean_smoothness(i))
            bar(i, mean_smoothness(i), 'FaceColor', colors(i, :), 'EdgeColor', 'k');
        end
    end
    set(gca, 'XTick', 1:num_alg, 'XTickLabel', names, 'XTickLabelRotation', 45);
    ylabel('平滑度指数');
    title('(d) 路径平滑度对比 (越小越好)');
    set(gca, 'FontSize', 10);
    
    % 5. 节点数对比
    subplot(2, 3, 5);
    hold on; grid on;
    for i = 1:num_alg
        if ~isnan(mean_nodes(i))
            bar(i, mean_nodes(i), 'FaceColor', colors(i, :), 'EdgeColor', 'k');
        end
    end
    set(gca, 'XTick', 1:num_alg, 'XTickLabel', names, 'XTickLabelRotation', 45);
    ylabel('节点数');
    title('(e) 计算复杂度对比');
    set(gca, 'FontSize', 10);
    
    % 6. 时间-长度散点图
    subplot(2, 3, 6);
    hold on; grid on;
    for i = 1:num_alg
        if ~isnan(mean_times(i)) && ~isnan(mean_lengths(i))
            plot(mean_times(i), mean_lengths(i), results{i}.algorithm.marker, ...
                 'MarkerSize', 12, 'MarkerFaceColor', colors(i, :), ...
                 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        end
    end
    xlabel('收敛时间 (秒)');
    ylabel('路径长度');
    title('(f) 时间-长度权衡分析');
    legend(names, 'Location', 'best', 'FontSize', 9);
    set(gca, 'FontSize', 10);
    
    % 保存图表
    if config.save_results
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        saveas(fig, fullfile(config.save_path, sprintf('comparison_plot_%s.png', timestamp)));
        saveas(fig, fullfile(config.save_path, sprintf('comparison_plot_%s.fig', timestamp)));
    end
end
