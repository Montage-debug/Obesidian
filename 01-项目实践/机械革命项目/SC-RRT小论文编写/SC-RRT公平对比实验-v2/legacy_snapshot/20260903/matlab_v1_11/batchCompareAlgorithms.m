function results = batchCompareAlgorithms(num_runs, max_iterations, varargin)
% batchCompareAlgorithms - 批量对比RRT算法性能
%
% 功能:
%   在统一环境下批量运行多个RRT算法,生成性能对比数据和图表

% 添加算法子目录到路径
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, 'sc_rrt'));
addpath(fullfile(script_dir, 'dynamic_rrt'));
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
addParameter(p, 'Algorithms', {'RRT_Basic', 'RRT_Connect_Basic', 'RRT_Star_Basic', 'Informed_RRT_Star_Basic', 'SC_RRT_Basic', 'Dynamic_RRT_Basic'}, @iscell);
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
    data.(algo).path_efficiency = [];
    data.(algo).min_clearance = [];
    data.(algo).avg_clearance = [];
    data.(algo).avg_turning_angle = [];
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
                    
                case 'Dynamic_RRT_Basic'
                    [path, tree, success, metrics] = Dynamic_RRT_Basic(env, max_iterations);
                    if success
                        path_length = metrics.path_length;
                    else
                        path_length = inf;
                    end
                    
                case 'Informed_RRT_Star_Basic'
                    [path, tree, success, metrics] = Informed_RRT_Star_Basic(env, max_iterations);
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
                % 对SC-RRT使用核心树搜索时间（不含路径后处理），保证与其他算法口径一致
                if strcmp(algo, 'SC_RRT_Basic') && exist('metrics','var') && isfield(metrics,'planning_time')
                    record_plan_time = metrics.planning_time;  % 树搜索时间（后处理前的toc）
                else
                    record_plan_time = time_elapsed;
                end
                data.(algo).planning_times = [data.(algo).planning_times; record_plan_time];
                
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
                
                % 计算路径效率 (直线距离 / 实际路径长度)
                straight_dist_val = norm(env.goal_point - env.start_point);
                data.(algo).path_efficiency = [data.(algo).path_efficiency; straight_dist_val / (path_length + eps)];
                
                % 计算安全间隙 (路径点到最近障碍物表面的距离)
                if ~isempty(path) && isfield(env, 'obstacles') && ~isempty(env.obstacles) && isfinite(path_length)
                    obs_mat = env.obstacles;
                    is_env3d = strcmp(env.dimension, '3D');
                    clr_per_pt = inf(size(path, 1), 1);
                    for k = 1:size(path, 1)
                        if is_env3d
                            dists_k = vecnorm(obs_mat(:,1:3) - path(k,:), 2, 2) - obs_mat(:,4);
                        else
                            dists_k = vecnorm(obs_mat(:,1:2) - path(k,:), 2, 2) - obs_mat(:,3);
                        end
                        clr_per_pt(k) = min(dists_k);
                    end
                    data.(algo).min_clearance = [data.(algo).min_clearance; min(clr_per_pt)];
                    data.(algo).avg_clearance = [data.(algo).avg_clearance; mean(clr_per_pt)];
                else
                    data.(algo).min_clearance = [data.(algo).min_clearance; NaN];
                    data.(algo).avg_clearance = [data.(algo).avg_clearance; NaN];
                end
                
                % 计算平均转角 (度)
                if ~isempty(path) && size(path, 1) >= 3
                    ang_vals = zeros(size(path, 1) - 2, 1);
                    for k = 2:size(path, 1)-1
                        v1 = path(k,:) - path(k-1,:);
                        v2 = path(k+1,:) - path(k,:);
                        ca = dot(v1,v2) / (norm(v1)*norm(v2) + 1e-10);
                        ang_vals(k-1) = acos(max(-1, min(1, ca))) * 180 / pi;
                    end
                    data.(algo).avg_turning_angle = [data.(algo).avg_turning_angle; mean(ang_vals)];
                else
                    data.(algo).avg_turning_angle = [data.(algo).avg_turning_angle; NaN];
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
                data.(algo).path_efficiency = [data.(algo).path_efficiency; NaN];
                data.(algo).min_clearance = [data.(algo).min_clearance; NaN];
                data.(algo).avg_clearance = [data.(algo).avg_clearance; NaN];
                data.(algo).avg_turning_angle = [data.(algo).avg_turning_angle; NaN];
                
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
            data.(algo).path_efficiency = [data.(algo).path_efficiency; NaN];
            data.(algo).min_clearance = [data.(algo).min_clearance; NaN];
            data.(algo).avg_clearance = [data.(algo).avg_clearance; NaN];
            data.(algo).avg_turning_angle = [data.(algo).avg_turning_angle; NaN];
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
    
    % 路径效率统计
    valid_eff = data.(algo).path_efficiency;
    valid_eff = valid_eff(isfinite(valid_eff) & ~isnan(valid_eff));
    if ~isempty(valid_eff)
        stats.path_efficiency_mean = mean(valid_eff);
        stats.path_efficiency_std  = std(valid_eff);
    else
        stats.path_efficiency_mean = NaN;
        stats.path_efficiency_std  = NaN;
    end
    
    % 最小安全间隙统计
    valid_mc = data.(algo).min_clearance;
    valid_mc = valid_mc(isfinite(valid_mc) & ~isnan(valid_mc));
    if ~isempty(valid_mc)
        stats.min_clearance_mean = mean(valid_mc);
        stats.min_clearance_std  = std(valid_mc);
    else
        stats.min_clearance_mean = NaN;
        stats.min_clearance_std  = NaN;
    end
    
    % 平均安全间隙统计
    valid_ac = data.(algo).avg_clearance;
    valid_ac = valid_ac(isfinite(valid_ac) & ~isnan(valid_ac));
    if ~isempty(valid_ac)
        stats.avg_clearance_mean = mean(valid_ac);
        stats.avg_clearance_std  = std(valid_ac);
    else
        stats.avg_clearance_mean = NaN;
        stats.avg_clearance_std  = NaN;
    end
    
    % 平均转角统计
    valid_ang = data.(algo).avg_turning_angle;
    valid_ang = valid_ang(~isnan(valid_ang));
    if ~isempty(valid_ang)
        stats.avg_turning_angle_mean = mean(valid_ang);
        stats.avg_turning_angle_std  = std(valid_ang);
    else
        stats.avg_turning_angle_mean = NaN;
        stats.avg_turning_angle_std  = NaN;
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
    fprintf('  平滑度:     %.4f ± %.4f rad\n', stats.smoothness_mean, stats.smoothness_std);
    fprintf('  路径效率:   %.4f ± %.4f\n', stats.path_efficiency_mean, stats.path_efficiency_std);
    fprintf('  最小间隙:   %.3f ± %.3f mm\n', stats.min_clearance_mean, stats.min_clearance_std);
    fprintf('  平均间隙:   %.3f ± %.3f mm\n', stats.avg_clearance_mean, stats.avg_clearance_std);
    fprintf('  平均转角:   %.2f ± %.2f 度\n', stats.avg_turning_angle_mean, stats.avg_turning_angle_std);
    fprintf('\n');
end

%% 生成对比图表（仅当 Visualize=true 时）
figures = [];
if visualize
    figures = generateComparisonCharts(algorithms, data, statistics);
end


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
    exportToCSV(csv_file, algorithms, statistics, data, num_runs, dimension);
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
    
    % 保存图表（仅当 Visualize=true 时）
    if visualize && ~isempty(figures)
        for i = 1:length(figures)
            fig_file = fullfile(output_dir, sprintf('chart_%d_%s_%s.png', i, dimension, timestamp));
            saveas(figures(i), fig_file);
            fprintf('[chart] %d saved: %s\n', i, fig_file);
            files.(sprintf('chart%d', i)) = fig_file;
        end
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
    % 生成SCI论文级性能对比图表
    
    figures = [];
    n = length(algorithms);
    
    % 6算法配色方案（与runAlgorithmComparison保持一致）
    algo_colors = [0.85 0.33 0.10; 0.93 0.69 0.13; 0.47 0.67 0.19;
                   0.56 0.27 0.52; 0.00 0.45 0.74; 1.00 0.00 0.00];
    if n > size(algo_colors, 1)
        algo_colors = [algo_colors; repmat([0.3 0.3 0.3], n - size(algo_colors,1), 1)];
    end
    algo_labels = cellfun(@(x) strrep(x,'_','-'), algorithms, 'UniformOutput', false);
    
    % ── 图1: 6项核心指标柱状图 (2×3) ──────────────────────────────
    fig1 = figure('Name', '算法性能对比-核心指标', 'Position', [80 80 1400 700]);
    
    metrics_cfg = {
        'success_rate',           '',                    '成功率 (%)';
        'path_length_mean',       'path_length_std',     '路径长度 (mm)';
        'time_mean',              'time_std',            '规划时间 (s)';
        'nodes_mean',             'nodes_std',           '树节点数';
        'smoothness_mean',        'smoothness_std',      '平滑度 (rad, ↓)';
        'path_efficiency_mean',   'path_efficiency_std', '路径效率 (↑)'
    };
    chart_titles = {'成功率', '路径长度', '规划时间', '树节点数', '路径平滑度', '路径效率'};
    
    for mi = 1:6
        subplot(2, 3, mi);
        vals = zeros(n, 1);
        stds = zeros(n, 1);
        for j = 1:n
            a = algorithms{j};
            if isfield(statistics, a) && isfield(statistics.(a), metrics_cfg{mi,1})
                vals(j) = statistics.(a).(metrics_cfg{mi,1});
                if ~isempty(metrics_cfg{mi,2}) && isfield(statistics.(a), metrics_cfg{mi,2})
                    stds(j) = statistics.(a).(metrics_cfg{mi,2});
                end
            end
        end
        b = bar(vals, 'FaceColor', 'flat');
        for j = 1:n
            b.CData(j,:) = algo_colors(min(j, size(algo_colors,1)), :);
        end
        if any(stds > 0)
            hold on;
            errorbar(1:n, vals, stds, 'k.', 'LineWidth', 1.5, 'CapSize', 5);
        end
        set(gca, 'XTick', 1:n, 'XTickLabel', algo_labels, 'FontSize', 8);
        xtickangle(30);
        ylabel(metrics_cfg{mi,3}, 'FontSize', 9);
        title(chart_titles{mi}, 'FontSize', 10, 'FontWeight', 'bold');
        grid on; box on;
    end
    sgtitle('算法性能对比 (Mean ± Std)', 'FontSize', 13, 'FontWeight', 'bold');
    figures = [figures, fig1];
    
    % ── 图2: 路径长度与规划时间分布直方图 ──────────────────────────
    fig2 = figure('Name', '数据分布', 'Position', [100 100 1200 450]);
    
    subplot(1, 2, 1);
    hold on;
    for j = 1:n
        raw = data.(algorithms{j}).path_lengths;
        valid_d = raw(isfinite(raw));
        if ~isempty(valid_d)
            histogram(valid_d, 10, 'FaceColor', algo_colors(min(j,size(algo_colors,1)),:), ...
                      'FaceAlpha', 0.55, 'DisplayName', algo_labels{j});
        end
    end
    xlabel('路径长度 (mm)', 'FontSize', 10);
    ylabel('频数', 'FontSize', 10);
    title('路径长度分布', 'FontSize', 11, 'FontWeight', 'bold');
    legend('show', 'FontSize', 8, 'Location', 'best');
    grid on;
    
    subplot(1, 2, 2);
    hold on;
    for j = 1:n
        raw = data.(algorithms{j}).planning_times;
        valid_d = raw(~isnan(raw));
        if ~isempty(valid_d)
            histogram(valid_d, 10, 'FaceColor', algo_colors(min(j,size(algo_colors,1)),:), ...
                      'FaceAlpha', 0.55, 'DisplayName', algo_labels{j});
        end
    end
    xlabel('规划时间 (s)', 'FontSize', 10);
    ylabel('频数', 'FontSize', 10);
    title('规划时间分布', 'FontSize', 11, 'FontWeight', 'bold');
    legend('show', 'FontSize', 8, 'Location', 'best');
    grid on;
    sgtitle('数据分布分析', 'FontSize', 12, 'FontWeight', 'bold');
    figures = [figures, fig2];
    
    % ── 图3: 安全间隙与平均转角对比 ──────────────────────────────
    fig3 = figure('Name', '路径质量-间隙与转角', 'Position', [120 120 900 420]);
    
    subplot(1, 2, 1);
    mc_vals = zeros(n,1); mc_stds = zeros(n,1);
    for j = 1:n
        a = algorithms{j};
        if isfield(statistics, a) && isfield(statistics.(a), 'min_clearance_mean')
            mc_vals(j) = statistics.(a).min_clearance_mean;
            mc_stds(j) = statistics.(a).min_clearance_std;
        end
    end
    bmc = bar(mc_vals, 'FaceColor', 'flat');
    for j = 1:n, bmc.CData(j,:) = algo_colors(min(j,size(algo_colors,1)),:); end
    if any(mc_stds > 0)
        hold on;
        errorbar(1:n, mc_vals, mc_stds, 'k.', 'LineWidth', 1.5, 'CapSize', 5);
    end
    set(gca, 'XTick', 1:n, 'XTickLabel', algo_labels, 'FontSize', 8); xtickangle(30);
    ylabel('最小安全间隙 (mm)', 'FontSize', 9);
    title('路径安全间隙', 'FontSize', 10, 'FontWeight', 'bold'); grid on; box on;
    
    subplot(1, 2, 2);
    at_vals = zeros(n,1); at_stds = zeros(n,1);
    for j = 1:n
        a = algorithms{j};
        if isfield(statistics, a) && isfield(statistics.(a), 'avg_turning_angle_mean')
            at_vals(j) = statistics.(a).avg_turning_angle_mean;
            at_stds(j) = statistics.(a).avg_turning_angle_std;
        end
    end
    bat = bar(at_vals, 'FaceColor', 'flat');
    for j = 1:n, bat.CData(j,:) = algo_colors(min(j,size(algo_colors,1)),:); end
    if any(at_stds > 0)
        hold on;
        errorbar(1:n, at_vals, at_stds, 'k.', 'LineWidth', 1.5, 'CapSize', 5);
    end
    set(gca, 'XTick', 1:n, 'XTickLabel', algo_labels, 'FontSize', 8); xtickangle(30);
    ylabel('平均转角 (度, ↓越好)', 'FontSize', 9);
    title('路径平滑性（转角）', 'FontSize', 10, 'FontWeight', 'bold'); grid on; box on;
    sgtitle('路径质量对比', 'FontSize', 12, 'FontWeight', 'bold');
    figures = [figures, fig3];

    % ── 图4: 关键指标箱线图（论文数据分布可视化）────────────────────
    fig4 = figure('Name', '关键指标箱线图', 'Position', [160 80 1400 620]);
    bp_cfg = {
        'path_lengths',      '路径长度 (mm)',   '路径长度';
        'convergence_times', '收敛时间 (s)',     '收敛时间 (首次解)';
        'smoothness',        '平滑度 (rad ↓)',  '路径平滑度';
        'path_efficiency',   '路径效率 (↑)',    '路径效率';
    };
    for mi = 1:4
        subplot(2, 2, mi);
        all_vals  = [];
        group_ids = [];
        for j = 1:n
            fname = bp_cfg{mi, 1};
            if isfield(data.(algorithms{j}), fname)
                raw   = data.(algorithms{j}).(fname);
                valid = raw(isfinite(raw) & ~isnan(raw));
                if ~isempty(valid)
                    all_vals  = [all_vals;  valid(:)];
                    group_ids = [group_ids; repmat(j, numel(valid), 1)];
                end
            end
        end
        if ~isempty(all_vals)
            ugroups = unique(group_ids);
            lbl     = algo_labels(ugroups);
            try
                boxplot(all_vals, group_ids, 'Labels', lbl, ...
                        'Color', 'k', 'BoxStyle', 'outline', 'Widths', 0.5);
                h_box = findobj(gca, 'Tag', 'Box');
                for jj = 1:length(h_box)
                    ci = max(1, min(size(algo_colors,1), length(h_box)+1-jj));
                    patch(get(h_box(jj),'XData'), get(h_box(jj),'YData'), ...
                          algo_colors(ci,:), 'FaceAlpha', 0.55);
                end
            catch
            end
        end
        set(gca, 'FontSize', 8);
        ylabel(bp_cfg{mi, 2}, 'FontSize', 9);
        title(bp_cfg{mi, 3}, 'FontSize', 10, 'FontWeight', 'bold');
        grid on; box on; xtickangle(20);
    end
    sgtitle('关键指标数据分布（箱线图）', 'FontSize', 12, 'FontWeight', 'bold');
    figures = [figures, fig4];
end

function exportToCSV(filename, algorithms, statistics, data, num_runs, dimension)
    % 导出SCI论文级实验数据到CSV
    % 包含: 表1-摘要统计 (Mean±Std) + 表2-逐次原始数据
    
    % 确保目录存在
    [fpath, ~, ~] = fileparts(filename);
    if ~isempty(fpath) && ~exist(fpath, 'dir')
        mkdir(fpath);
    end
    
    fid = fopen(filename, 'w', 'n', 'UTF-8');
    if fid < 0
        warning('无法创建文件: %s', filename);
        return;
    end
    
    % ── 文件头信息 ───────────────────────────────────────────────
    fprintf(fid, '# SCI论文-路径规划算法性能对比数据\r\n');
    fprintf(fid, '# 维度: %s | 运行次数: %d | 生成时间: %s\r\n', ...
            dimension, num_runs, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '# 算法: %s\r\n', strjoin(algorithms, ', '));
    fprintf(fid, '#\r\n');
    fprintf(fid, '# 指标说明:\r\n');
    fprintf(fid, '#   路径长度: 起终点间路径欧氏距离之和 (mm)\r\n');
    fprintf(fid, '#   收敛时间: 首次获得可行解所用时间 (s)\r\n');
    fprintf(fid, '#   规划时间: 算法总运行时间 (s)\r\n');
    fprintf(fid, '#   树节点数: 探索树的总节点数量\r\n');
    fprintf(fid, '#   平滑度:   路径转角变化标准差 (rad, 越小越平滑)\r\n');
    fprintf(fid, '#   路径效率: 直线距离/路径长度 (越大越高效)\r\n');
    fprintf(fid, '#   最小间隙: 路径离最近障碍物表面的最小距离 (mm)\r\n');
    fprintf(fid, '#   平均间隙: 路径点到最近障碍物表面距离的均值 (mm)\r\n');
    fprintf(fid, '#   平均转角: 路径各拐点平均转向角度 (deg, 越小越平滑)\r\n');
    fprintf(fid, '\r\n');
    
    % ── 表1: SCI论文摘要统计 (Mean±Std) ────────────────────────
    fprintf(fid, '## 表1: 性能统计摘要 (Mean+-Std, N=%d)\r\n', num_runs);
    hdr1 = {'算法','成功率(%)', ...
            '路径长度(mm)','收敛时间(s)','规划时间(s)', ...
            '树节点数','平滑度(rad)','路径效率', ...
            '最小间隙(mm)','平均间隙(mm)','平均转角(deg)'};
    fprintf(fid, '%s\r\n', strjoin(hdr1, ','));
    
    for i = 1:length(algorithms)
        a = algorithms{i};
        if ~isfield(statistics, a), continue; end
        s = statistics.(a);
        row = {a, ...
            sprintf('%.1f', s.success_rate), ...
            fmtMS(s,'path_length_mean',       'path_length_std',       '%.2f'), ...
            fmtMS(s,'convergence_time_mean',  'convergence_time_std',  '%.4f'), ...
            fmtMS(s,'time_mean',              'time_std',              '%.4f'), ...
            fmtMS(s,'nodes_mean',             'nodes_std',             '%.1f'), ...
            fmtMS(s,'smoothness_mean',        'smoothness_std',        '%.4f'), ...
            fmtMS(s,'path_efficiency_mean',   'path_efficiency_std',   '%.4f'), ...
            fmtMS(s,'min_clearance_mean',     'min_clearance_std',     '%.3f'), ...
            fmtMS(s,'avg_clearance_mean',     'avg_clearance_std',     '%.3f'), ...
            fmtMS(s,'avg_turning_angle_mean', 'avg_turning_angle_std', '%.2f')  ...
        };
        fprintf(fid, '%s\r\n', strjoin(row, ','));
    end
    fprintf(fid, '\r\n');
    
    % ── 表2: 逐次运行原始数据 (供箱线图/折线图绘制) ─────────────
    fprintf(fid, '## 表2: 逐次运行原始数据\r\n');
    hdr2 = {'算法','运行序号','成功(1/0)', ...
            '路径长度(mm)','收敛时间(s)','规划时间(s)', ...
            '树节点数','平滑度(rad)','路径效率', ...
            '最小间隙(mm)','平均间隙(mm)','平均转角(deg)'};
    fprintf(fid, '%s\r\n', strjoin(hdr2, ','));
    
    for i = 1:length(algorithms)
        a = algorithms{i};
        if ~isfield(data, a), continue; end
        d = data.(a);
        nr = length(d.path_lengths);
        for r = 1:nr
            ok = isfinite(d.path_lengths(r)) && ~isnan(d.path_lengths(r));
            fprintf(fid, '%s,%d,%d,%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n', ...
                a, r, int32(ok), ...
                numStr(d.path_lengths(r)), ...
                numStr(d.convergence_times(r)), ...
                numStr(d.planning_times(r)), ...
                numStr(d.tree_nodes(r)), ...
                numStr(d.smoothness(r)), ...
                safeField(d,'path_efficiency',   r), ...
                safeField(d,'min_clearance',     r), ...
                safeField(d,'avg_clearance',     r), ...
                safeField(d,'avg_turning_angle', r));
        end
    end
    
    fclose(fid);
    total_rows = sum(cellfun(@(a) length(data.(a).path_lengths), algorithms));
    fprintf('✓ CSV数据已保存: %s\n', filename);
    fprintf('  (表1: %d算法摘要, 表2: %d条原始记录)\n', length(algorithms), total_rows);
end

function s = fmtMS(st, mf, sf, fmt)
    % 格式化 Mean±Std 字符串
    if isfield(st, mf) && ~isnan(st.(mf))
        if isfield(st, sf) && ~isnan(st.(sf))
            s = sprintf([fmt '+-' fmt], st.(mf), st.(sf));
        else
            s = sprintf(fmt, st.(mf));
        end
    else
        s = 'N/A';
    end
end

function s = numStr(v)
    % 将标量数值安全地格式化为字符串
    if isnan(v)
        s = 'NaN';
    elseif isinf(v)
        s = 'Inf';
    else
        s = sprintf('%.4f', v);
    end
end

function s = safeField(d, fname, idx)
    % 安全读取 data 结构体中某字段的第 idx 个值
    if isfield(d, fname) && length(d.(fname)) >= idx
        s = numStr(d.(fname)(idx));
    else
        s = 'NaN';
    end
end
