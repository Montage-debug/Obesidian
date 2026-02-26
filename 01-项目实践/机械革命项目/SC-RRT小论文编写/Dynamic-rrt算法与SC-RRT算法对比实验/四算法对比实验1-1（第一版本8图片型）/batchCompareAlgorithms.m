function results = batchCompareAlgorithms(num_runs, max_iterations, varargin)
% batchCompareAlgorithms - 批量对比RRT算法性能
%
% 功能:
%   在统一环境下批量运行多个RRT算法,生成性能对比数据和图表

% 添加sc_rrt子目录到路径
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, 'sc_rrt'));
%
% 输入:
%   num_runs        - 每个算法运行次数 (默认: 10)
%   max_iterations  - 最大迭代次数 (默认: 3000)
%   varargin        - 可选参数 (Name-Value pairs):
%     'Algorithms'    - 算法列表 cell array (默认: {'RRT_Basic', 'RRT_Star_Basic', 'SC_RRT_Basic'})
%     'Dimension'     - '2D' 或 '3D' (默认: '2D')
%     'NumObstacles'  - 障碍物数量 (默认: 20)
%     'Bounds'        - 环境边界 (默认: [0 100 0 100] 或 [0 100 0 100 0 100])
%     'SaveResults'   - 是否保存结果 (默认: true)
%     'Visualize'     - 是否可视化 (默认: false)
%     'OutputDir'     - 输出目录 (默认: 'comparison_results')
%
% 输出:
%   results - 结果结构体,包含:
%     .data           - 原始数据
%     .statistics     - 统计数据
%     .figures        - 图表句柄
%     .files          - 保存的文件路径
%
% 示例:
%   % 基础使用
%   results = batchCompareAlgorithms(10, 3000);
%
%   % 自定义参数
%   results = batchCompareAlgorithms(20, 5000, ...
%       'Algorithms', {'RRT_Basic', 'RRT_Star_Basic'}, ...
%       'NumObstacles', 30, ...
%       'Visualize', true);
%
%   % 3D环境对比
%   results = batchCompareAlgorithms(10, 5000, 'Dimension', '3D');
%
% 作者: GitHub Copilot
% 日期: 2025-12-11

%% 参数解析
if nargin < 1, num_runs = 10; end
if nargin < 2, max_iterations = 3000; end

p = inputParser;
% addParameter(p, 'Algorithms', {'RRT_Basic', 'RRT_Star_Basic', 'SC_RRT_Basic', 'RRT_Connect_Basic'}, @iscell);
addParameter(p, 'Algorithms', { 'SC_RRT_Basic'}, @iscell);
addParameter(p, 'Dimension', '2D', @(x) ismember(x, {'2D', '3D'}));
addParameter(p, 'NumObstacles', 20, @isnumeric);
addParameter(p, 'Bounds', [], @isnumeric);
addParameter(p, 'SaveResults', true, @islogical);
addParameter(p, 'Visualize', false, @islogical);
addParameter(p, 'OutputDir', 'comparison_results', @ischar);
parse(p, varargin{:});

algorithms = p.Results.Algorithms;
dimension = p.Results.Dimension;
num_obstacles = p.Results.NumObstacles;
bounds = p.Results.Bounds;
save_results = p.Results.SaveResults;
visualize = p.Results.Visualize;
output_dir = p.Results.OutputDir;

% 设置默认边界
if isempty(bounds)
    if strcmp(dimension, '2D')
        bounds = [0 1500 0 1500];
    else
        bounds = [0 1500 0 1500 0 1500];
    end
end

% 创建输出目录
if save_results && ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% 打印标题
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║           批量RRT算法性能对比实验                              ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('  维度: %s\n', dimension);
fprintf('  算法数量: %d\n', length(algorithms));
fprintf('  每算法运行次数: %d\n', num_runs);
fprintf('  最大迭代次数: %d\n', max_iterations);
fprintf('  障碍物数量: %d\n', num_obstacles);
fprintf('  环境边界: [%s]\n', num2str(bounds));
fprintf('════════════════════════════════════════════════════════════════\n\n');

%% 初始化数据存储
data = struct();
for i = 1:length(algorithms)
    algo = algorithms{i};
    data.(algo) = struct();
    data.(algo).success_count = 0;
    data.(algo).path_lengths = [];
    data.(algo).planning_times = [];
    data.(algo).convergence_times = [];  % 新增：收敛时间（首次可行解）
    data.(algo).tree_nodes = [];
    data.(algo).smoothness = [];
end

% 创建统一度量记录器（用于论文数据）
unified_recorder = UnifiedMetricsRecorder(dimension);
unified_recorder.environment_config = struct('num_obstacles', num_obstacles, 'bounds', bounds);

%% 生成统一环境 (所有算法在相同环境下测试)
fprintf('【环境生成】\n');
fprintf('正在生成 %d 个%s测试环境...\n', num_runs, dimension);

environments = cell(num_runs, 1);
for run = 1:num_runs
    if strcmp(dimension, '2D')
        environments{run} = generate2DEnvironment(bounds, num_obstacles);
    else
        environments{run} = generate3DEnvironment(bounds, num_obstacles);
    end
end
fprintf('✓ 完成\n\n');

%% 运行算法对比
fprintf('【算法测试】\n');
total_tests = num_runs * length(algorithms);
current_test = 0;

for run = 1:num_runs
    fprintf('──────────────────────────────────────────────────────────────\n');
    fprintf('运行 %d/%d (环境 #%d)\n', run, num_runs, run);
    fprintf('──────────────────────────────────────────────────────────────\n');
    
    env = environments{run};
    
    for i = 1:length(algorithms)
        algo = algorithms{i};
        current_test = current_test + 1;
        
        fprintf('[%d/%d] %s: ', current_test, total_tests, algo);
        
        try
            % 清除上次迭代的metrics变量
            if exist('metrics', 'var')
                clear metrics;
            end
            
            % 运行算法
            tic;
            switch algo
                case 'RRT_Basic'
                    [path, tree, success] = RRT_Basic(env, max_iterations);
                    if success
                        path_length = tree.final_cost;
                    else
                        path_length = inf;
                    end
                    
                case 'RRT_Star_Basic'
                    [path, tree, success] = RRT_Star_Basic(env, max_iterations);
                    if success
                        path_length = tree.final_cost;
                    else
                        path_length = inf;
                    end
                    
                case 'SC_RRT_Basic'
                    [path, tree, success, metrics] = SC_RRT_Basic(env, max_iterations);
                    if success
                        path_length = metrics.path_length;
                    else
                        path_length = inf;
                    end
                    
                case 'RRT_Connect_Basic'
                    % RRT-Connect需要4个参数: env, max_iter, step_size, goal_threshold
                    % 使用默认步长和目标阈值
                    [path, tree, success] = RRT_Connect_Basic(env, max_iterations, [], []);
                    if success
                        path_length = tree.final_cost;
                    else
                        path_length = inf;
                    end
                    
                case 'SC_RRT_Optimized'
                    [path, tree, success, metrics] = SC_RRT_Optimized(env, max_iterations);
                    if success
                        path_length = metrics.path_length;
                    else
                        path_length = inf;
                    end
                    
                otherwise
                    error('未知算法: %s', algo);
            end
            time_elapsed = toc;
            
            % 记录结果
            if success
                data.(algo).success_count = data.(algo).success_count + 1;
                data.(algo).path_lengths = [data.(algo).path_lengths; path_length];
                data.(algo).planning_times = [data.(algo).planning_times; time_elapsed];
                
                % 记录收敛时间（首次可行解时间）
                % 对于SC-RRT，使用其提供的convergence_time；对于基础算法，首次找到解即为收敛
                if exist('metrics', 'var') && isfield(metrics, 'convergence_time')
                    convergence_time = metrics.convergence_time;
                elseif isfield(tree, 'convergence_time')
                    convergence_time = tree.convergence_time;
                else
                    convergence_time = time_elapsed;  % 基础算法首次找到解即为收敛
                end
                data.(algo).convergence_times = [data.(algo).convergence_times; convergence_time];
                
                data.(algo).tree_nodes = [data.(algo).tree_nodes; size(tree.vertices, 1)];
                
                % 计算平滑度
                if ~isempty(path) && size(path, 1) >= 3
                    smoothness = calculatePathSmoothness(path);
                    data.(algo).smoothness = [data.(algo).smoothness; smoothness];
                else
                    data.(algo).smoothness = [data.(algo).smoothness; NaN];
                end
                
                % 使用统一记录器记录（用于论文）
                if exist('metrics', 'var')
                    unified_recorder.recordAlgorithmRun(algo, env, path, tree, success, time_elapsed, metrics);
                else
                    unified_recorder.recordAlgorithmRun(algo, env, path, tree, success, time_elapsed);
                end
                
                fprintf('✓ 成功 (代价=%.2f, 收敛=%.3fs, 总时间=%.3fs)\n', path_length, convergence_time, time_elapsed);
                
                % 可视化第一次成功的结果
                if visualize && data.(algo).success_count == 1
                    figure('Name', sprintf('%s - 示例结果', algo));
                    visualizeAndSaveRRTResult(env, path, tree, success, algo, output_dir);
                end
            else
                data.(algo).path_lengths = [data.(algo).path_lengths; inf];
                data.(algo).planning_times = [data.(algo).planning_times; time_elapsed];
                data.(algo).convergence_times = [data.(algo).convergence_times; inf];
                data.(algo).tree_nodes = [data.(algo).tree_nodes; size(tree.vertices, 1)];
                data.(algo).smoothness = [data.(algo).smoothness; NaN];
                
                % 使用统一记录器记录失败情况
                if exist('metrics', 'var')
                    unified_recorder.recordAlgorithmRun(algo, env, path, tree, success, time_elapsed, metrics);
                else
                    unified_recorder.recordAlgorithmRun(algo, env, path, tree, success, time_elapsed);
                end
                
                fprintf('✗ 失败 (时间=%.3fs)\n', time_elapsed);
            end
            
        catch ME
            fprintf('✗ 错误: %s\n', ME.message);
            data.(algo).path_lengths = [data.(algo).path_lengths; inf];
            data.(algo).planning_times = [data.(algo).planning_times; NaN];
            data.(algo).convergence_times = [data.(algo).convergence_times; NaN];
            data.(algo).tree_nodes = [data.(algo).tree_nodes; NaN];
            data.(algo).smoothness = [data.(algo).smoothness; NaN];
        end
    end
    fprintf('\n');
end

%% 统计分析
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('【统计分析】\n');
fprintf('════════════════════════════════════════════════════════════════\n\n');

statistics = struct();
for i = 1:length(algorithms)
    algo = algorithms{i};
    
    % 计算统计量
    stats = struct();
    stats.success_rate = data.(algo).success_count / num_runs * 100;
    
    valid_lengths = data.(algo).path_lengths(isfinite(data.(algo).path_lengths));
    valid_conv_times = data.(algo).convergence_times(isfinite(data.(algo).convergence_times));
    valid_times = data.(algo).planning_times(~isnan(data.(algo).planning_times));
    valid_nodes = data.(algo).tree_nodes(~isnan(data.(algo).tree_nodes));
    valid_smoothness = data.(algo).smoothness(~isnan(data.(algo).smoothness));
    
    % 路径代价统计
    if ~isempty(valid_lengths)
        stats.path_length_mean = mean(valid_lengths);
        stats.path_length_std = std(valid_lengths);
        stats.path_length_min = min(valid_lengths);
        stats.path_length_max = max(valid_lengths);
    else
        stats.path_length_mean = NaN;
        stats.path_length_std = NaN;
        stats.path_length_min = NaN;
        stats.path_length_max = NaN;
    end
    
    % 收敛时间统计（首次可行解时间）
    if ~isempty(valid_conv_times)
        stats.convergence_time_mean = mean(valid_conv_times);
        stats.convergence_time_std = std(valid_conv_times);
        stats.convergence_time_min = min(valid_conv_times);
        stats.convergence_time_max = max(valid_conv_times);
    else
        stats.convergence_time_mean = NaN;
        stats.convergence_time_std = NaN;
        stats.convergence_time_min = NaN;
        stats.convergence_time_max = NaN;
    end
    
    if ~isempty(valid_times)
        stats.time_mean = mean(valid_times);
        stats.time_std = std(valid_times);
    else
        stats.time_mean = NaN;
        stats.time_std = NaN;
    end
    
    if ~isempty(valid_nodes)
        stats.nodes_mean = mean(valid_nodes);
        stats.nodes_std = std(valid_nodes);
    else
        stats.nodes_mean = NaN;
        stats.nodes_std = NaN;
    end
    
    if ~isempty(valid_smoothness)
        stats.smoothness_mean = mean(valid_smoothness);
        stats.smoothness_std = std(valid_smoothness);
    else
        stats.smoothness_mean = NaN;
        stats.smoothness_std = NaN;
    end
    
    statistics.(algo) = stats;
    
    % 打印统计结果
    fprintf('【%s】\n', algo);
    fprintf('  成功率:     %.1f%% (%d/%d)\n', stats.success_rate, data.(algo).success_count, num_runs);
    fprintf('  路径代价:   %.2f ± %.2f (范围: %.2f ~ %.2f)\n', ...
            stats.path_length_mean, stats.path_length_std, stats.path_length_min, stats.path_length_max);
    fprintf('  收敛时间:   %.3f ± %.3f 秒 (首次可行解)\n', stats.convergence_time_mean, stats.convergence_time_std);
    fprintf('  计算时间:   %.3f ± %.3f 秒 (总时间)\n', stats.time_mean, stats.time_std);
    fprintf('  树节点数:   %.0f ± %.0f\n', stats.nodes_mean, stats.nodes_std);
    fprintf('  平滑度:     %.3f ± %.3f\n', stats.smoothness_mean, stats.smoothness_std);
    fprintf('\n');
end

%% 生成对比图表
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('【生成对比图表】\n');
fprintf('════════════════════════════════════════════════════════════════\n\n');

figures = generateComparisonCharts(algorithms, data, statistics);

%% 保存结果
files = struct();
if save_results
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    % 保存MAT数据
    mat_file = fullfile(output_dir, sprintf('comparison_%s_%s.mat', dimension, timestamp));
    save(mat_file, 'data', 'statistics', 'algorithms', 'num_runs', 'max_iterations', ...
         'dimension', 'num_obstacles', 'bounds');
    fprintf('✓ MAT数据: %s\n', mat_file);
    files.mat = mat_file;
    
    % 保存CSV表格
    csv_file = fullfile(output_dir, sprintf('comparison_%s_%s.csv', dimension, timestamp));
    exportToCSV(csv_file, algorithms, statistics);
    fprintf('✓ CSV表格: %s\n', csv_file);
    files.csv = csv_file;
    
    % 使用统一记录器导出论文专用数据
    unified_mat_file = fullfile(output_dir, sprintf('unified_metrics_%s_%s.mat', dimension, timestamp));
    unified_csv_file = fullfile(output_dir, sprintf('unified_metrics_%s_%s.csv', dimension, timestamp));
    unified_recorder.exportToMAT(unified_mat_file);
    unified_recorder.exportToCSV(unified_csv_file);
    fprintf('✓ 统一度量MAT: %s\n', unified_mat_file);
    fprintf('✓ 统一度量CSV: %s\n', unified_csv_file);
    files.unified_mat = unified_mat_file;
    files.unified_csv = unified_csv_file;
    
    % 打印统一度量摘要（论文格式）
    fprintf('\n');
    unified_recorder.printSummary();
    
    % 保存图表
    for i = 1:length(figures)
        fig_file = fullfile(output_dir, sprintf('chart_%d_%s_%s.png', i, dimension, timestamp));
        saveas(figures(i), fig_file);
        fprintf('✓ 图表%d: %s\n', i, fig_file);
        files.(sprintf('chart%d', i)) = fig_file;
    end
    
    fprintf('\n');
end

%% 返回结果
results = struct();
results.data = data;
results.statistics = statistics;
results.figures = figures;
results.files = files;
results.algorithms = algorithms;
results.num_runs = num_runs;
results.max_iterations = max_iterations;

fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    测试完成                                    ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('✓ 批量对比实验完成!\n');
fprintf('  总测试数: %d\n', total_tests);
fprintf('  成功测试: %d\n', sum(structfun(@(x) x.success_count, data)));
if save_results
    fprintf('  结果已保存到: %s/\n', output_dir);
end
fprintf('\n');

end

%% ========== 辅助函数 ==========

function smoothness = calculatePathSmoothness(path)
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

function figures = generateComparisonCharts(algorithms, data, statistics)
    % 生成对比图表
    
    figures = [];
    
    % 图1: 性能对比柱状图
    fig1 = figure('Name', '性能对比', 'Position', [100 100 1200 400]);
    
    subplot(1, 3, 1);
    success_rates = cellfun(@(a) statistics.(a).success_rate, algorithms);
    bar(success_rates);
    set(gca, 'XTickLabel', algorithms);
    ylabel('成功率 (%)');
    title('成功率对比');
    grid on;
    ylim([0 105]);
    
    subplot(1, 3, 2);
    path_means = cellfun(@(a) statistics.(a).path_length_mean, algorithms);
    path_stds = cellfun(@(a) statistics.(a).path_length_std, algorithms);
    bar(path_means);
    hold on;
    errorbar(1:length(algorithms), path_means, path_stds, 'k.', 'LineWidth', 1.5);
    set(gca, 'XTickLabel', algorithms);
    ylabel('路径长度');
    title('路径长度对比');
    grid on;
    
    subplot(1, 3, 3);
    time_means = cellfun(@(a) statistics.(a).time_mean, algorithms);
    time_stds = cellfun(@(a) statistics.(a).time_std, algorithms);
    bar(time_means);
    hold on;
    errorbar(1:length(algorithms), time_means, time_stds, 'k.', 'LineWidth', 1.5);
    set(gca, 'XTickLabel', algorithms);
    ylabel('规划时间 (秒)');
    title('规划时间对比');
    grid on;
    
    figures = [figures, fig1];
    
    % 图2: 直方图分布
    fig2 = figure('Name', '数据分布', 'Position', [100 100 1200 400]);
    
    subplot(1, 2, 1);
    hold on;
    colors_hist = {'b', 'r', 'g'};
    for i = 1:length(algorithms)
        algo = algorithms{i};
        valid_data = data.(algo).path_lengths(isfinite(data.(algo).path_lengths));
        if ~isempty(valid_data)
            histogram(valid_data, 'FaceColor', colors_hist{i}, 'FaceAlpha', 0.5, 'DisplayName', algo);
        end
    end
    xlabel('路径长度');
    ylabel('频数');
    title('路径长度分布');
    legend('show');
    grid on;
    
    subplot(1, 2, 2);
    hold on;
    for i = 1:length(algorithms)
        algo = algorithms{i};
        valid_data = data.(algo).planning_times(~isnan(data.(algo).planning_times));
        if ~isempty(valid_data)
            histogram(valid_data, 'FaceColor', colors_hist{i}, 'FaceAlpha', 0.5, 'DisplayName', algo);
        end
    end
    xlabel('规划时间 (秒)');
    ylabel('频数');
    title('规划时间分布');
    legend('show');
    grid on;
    
    figures = [figures, fig2];
end

function exportToCSV(filename, algorithms, statistics)
    % 导出统计数据到CSV
    
    fid = fopen(filename, 'w');
    
    % 写表头
    fprintf(fid, 'Algorithm,Success Rate (%%),Path Length Mean,Path Length Std,');
    fprintf(fid, 'Time Mean (s),Time Std (s),Nodes Mean,Nodes Std,Smoothness Mean,Smoothness Std\n');
    
    % 写数据
    for i = 1:length(algorithms)
        algo = algorithms{i};
        stats = statistics.(algo);
        
        fprintf(fid, '%s,%.2f,%.2f,%.2f,%.4f,%.4f,%.0f,%.0f,%.4f,%.4f\n', ...
                algo, stats.success_rate, ...
                stats.path_length_mean, stats.path_length_std, ...
                stats.time_mean, stats.time_std, ...
                stats.nodes_mean, stats.nodes_std, ...
                stats.smoothness_mean, stats.smoothness_std);
    end
    
    fclose(fid);
end
