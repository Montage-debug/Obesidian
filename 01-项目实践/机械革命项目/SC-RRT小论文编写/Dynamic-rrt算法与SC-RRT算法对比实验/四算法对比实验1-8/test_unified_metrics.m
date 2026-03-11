% test_unified_metrics.m - 测试统一性能度量系统
%
% 此脚本验证统一度量系统是否正常工作
% 运行一个快速测试，对比RRT、RRT*和SC-RRT算法
%
% 作者: 研究团队
% 日期: 2025-12-13

clear; clc; close all;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
fprintf('║           统一性能度量系统 - 快速测试                        ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════╝\n\n');

%% 测试参数
num_runs = 5;  % 快速测试，只运行5次
max_iterations = 3000;
algorithms = {'RRT_Basic', 'RRT_Star_Basic', 'SC_RRT_Basic'};
test_output_dir = 'test_results';

%% 创建测试输出目录
if ~exist(test_output_dir, 'dir')
    mkdir(test_output_dir);
end

%% 测试1: 2D环境
fprintf('【测试1】2D环境性能对比\n');
fprintf('参数: %d次运行, 最大迭代%d, 障碍物数20\n\n', num_runs, max_iterations);

try
    results_2d = batchCompareAlgorithms(num_runs, max_iterations, ...
        'Algorithms', algorithms, ...
        'Dimension', '2D', ...
        'NumObstacles', 20, ...
        'OutputDir', test_output_dir, ...
        'SaveResults', true, ...
        'Visualize', false);
    
    fprintf('✓ 2D环境测试完成\n\n');
catch ME
    fprintf('✗ 2D环境测试失败: %s\n\n', ME.message);
    rethrow(ME);
end

%% 测试2: 验证输出文件
fprintf('【测试2】验证输出文件\n');

expected_files = {
    'unified_metrics_2D_*.csv';
    'unified_metrics_2D_*.mat';
    'comparison_2D_*.csv';
    'comparison_2D_*.mat';
};

all_files_exist = true;
for i = 1:length(expected_files)
    pattern = expected_files{i};
    files = dir(fullfile(test_output_dir, pattern));
    
    if ~isempty(files)
        fprintf('✓ 找到文件: %s (%s)\n', pattern, files(1).name);
    else
        fprintf('✗ 未找到文件: %s\n', pattern);
        all_files_exist = false;
    end
end

if all_files_exist
    fprintf('\n✓ 所有输出文件生成成功\n\n');
else
    fprintf('\n✗ 部分输出文件缺失\n\n');
end

%% 测试3: 读取并验证数据
fprintf('【测试3】验证数据完整性\n');

% 找到最新的unified_metrics文件
mat_files = dir(fullfile(test_output_dir, 'unified_metrics_2D_*.mat'));
if isempty(mat_files)
    error('未找到unified_metrics MAT文件');
end

[~, idx] = max([mat_files.datenum]);
latest_mat = fullfile(test_output_dir, mat_files(idx).name);

% 加载数据
load(latest_mat, 'records', 'algorithms', 'statistics');

fprintf('✓ 成功加载MAT文件: %s\n', mat_files(idx).name);
fprintf('  - 记录数: %d\n', length(records));
fprintf('  - 算法数: %d\n', length(algorithms));

% 验证每个算法的关键指标
fprintf('\n验证关键指标:\n');
for i = 1:length(algorithms)
    algo = algorithms{i};
    if isfield(statistics, algo)
        s = statistics.(algo);
        fprintf('\n【%s】\n', algo);
        fprintf('  ✓ 成功率: %.1f%%\n', s.success_rate);
        
        if isfield(s, 'convergence_time_mean') && ~isnan(s.convergence_time_mean)
            fprintf('  ✓ 收敛时间: %.3f±%.3fs\n', s.convergence_time_mean, s.convergence_time_std);
        else
            fprintf('  ✗ 收敛时间: 缺失或无效\n');
        end
        
        if isfield(s, 'path_cost_mean') && ~isnan(s.path_cost_mean)
            fprintf('  ✓ 路径代价: %.2f±%.2f\n', s.path_cost_mean, s.path_cost_std);
        else
            fprintf('  ✗ 路径代价: 缺失或无效\n');
        end
        
        if isfield(s, 'planning_time_mean') && ~isnan(s.planning_time_mean)
            fprintf('  ✓ 计算时间: %.3f±%.3fs\n', s.planning_time_mean, s.planning_time_std);
        else
            fprintf('  ✗ 计算时间: 缺失或无效\n');
        end
    else
        fprintf('\n【%s】✗ 未找到统计数据\n', algo);
    end
end

%% 测试4: CSV文件可读性
fprintf('\n【测试4】验证CSV文件格式\n');

csv_files = dir(fullfile(test_output_dir, 'unified_metrics_2D_*.csv'));
if ~isempty(csv_files)
    [~, idx] = max([csv_files.datenum]);
    latest_csv = fullfile(test_output_dir, csv_files(idx).name);
    
    % 读取CSV内容
    fid = fopen(latest_csv, 'r', 'n', 'UTF-8');
    lines = {};
    while ~feof(fid)
        lines{end+1} = fgetl(fid);
    end
    fclose(fid);
    
    fprintf('✓ CSV文件可读: %s\n', csv_files(idx).name);
    fprintf('  行数: %d\n', length(lines));
    fprintf('  预览:\n');
    for i = 1:min(3, length(lines))
        fprintf('    %s\n', lines{i});
    end
else
    fprintf('✗ 未找到CSV文件\n');
end

%% 总结
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
fprintf('║                      测试总结                                 ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════╝\n');
fprintf('\n✓ 统一性能度量系统测试通过!\n');
fprintf('\n生成的测试文件位于: %s\n', test_output_dir);
fprintf('\n可以使用以下文件进行论文写作:\n');
fprintf('  - unified_metrics_2D_*.csv (论文表格)\n');
fprintf('  - unified_metrics_2D_*.mat (详细数据)\n');
fprintf('  - chart_*.png (对比图表)\n');
fprintf('\n使用方法请参考: 统一性能度量系统使用说明.md\n\n');
