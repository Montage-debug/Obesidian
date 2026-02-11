% =========================================================================
%     Dynamic RRT Interval参数对比实验
% =========================================================================
% 对应论文 Table 2: 不同interval值对收敛时间和路径长度的影响
% 
% 论文结论:
%   - interval从20→4: 一般更快且更短
%   - interval过小(如3): 可能耗时反增、路径不平滑
% =========================================================================
clear; clc; close all;

% =================== 实验配置 ===================
% 使用2D固定障碍物环境（可复现）
config = struct();
config.dimension = '2D';
config.bounds = [0 1500 0 1500];
config.startPoint = [400 400];
config.goalPoint = [1100 1100];
config.numObstacles = 225;
config.obstacleRadius = 15;
config.seed = 42;  % 固定种子保证可复现

% Interval测试值（对应论文）
interval_values = [3, 4, 6, 8, 10, 20];

% 每个interval运行次数
num_runs = 10;  % 论文用100次，这里先用10次快速测试

% 最大迭代次数
max_iterations = 10000;

fprintf('\n========================================\n');
fprintf('Dynamic RRT Interval参数对比实验\n');
fprintf('========================================\n');
fprintf('环境: 2D固定障碍物\n');
fprintf('障碍物数量: %d\n', config.numObstacles);
fprintf('测试interval值: ');
fprintf('%d ', interval_values);
fprintf('\n每个interval运行: %d 次\n', num_runs);
fprintf('========================================\n\n');

% =================== 生成障碍物（所有实验共用） ===================
fprintf('生成障碍物...\n');
obstacles = GenerateObstacles(config.dimension, config.bounds, ...
                              config.numObstacles, config.obstacleRadius, ...
                              config.startPoint, config.goalPoint, config.seed);
fprintf('实际生成: %d 个障碍物\n\n', size(obstacles.circles, 1));

% =================== 运行实验 ===================
results = struct();

for iv = 1:length(interval_values)
    interval = interval_values(iv);
    
    fprintf('========================================\n');
    fprintf('测试 Interval = %d\n', interval);
    fprintf('========================================\n');
    
    convergence_times = zeros(num_runs, 1);
    path_lengths = zeros(num_runs, 1);
    iterations = zeros(num_runs, 1);
    success_count = 0;
    
    for run = 1:num_runs
        fprintf('  Run %d/%d ... ', run, num_runs);
        
        % 运行Dynamic RRT
        [~, path, success, metrics] = DynamicRRT(...
            config.startPoint, config.goalPoint, config.bounds, obstacles, ...
            'MaxIterations', max_iterations, ...
            'Interval', interval, ...
            'ParetoProb', 0.1, ...
            'EnableVisualization', false);  % 关闭可视化加速
        
        if success
            convergence_times(run) = metrics.convergenceTime;
            path_lengths(run) = metrics.pathLength;
            iterations(run) = metrics.iterations;
            success_count = success_count + 1;
            fprintf('成功 (时间: %.4fs, 长度: %.2f)\n', ...
                    metrics.convergenceTime, metrics.pathLength);
        else
            convergence_times(run) = NaN;
            path_lengths(run) = NaN;
            iterations(run) = max_iterations;
            fprintf('失败\n');
        end
    end
    
    % 统计结果（只统计成功的运行）
    if success_count > 0
        valid_times = convergence_times(~isnan(convergence_times));
        valid_lengths = path_lengths(~isnan(path_lengths));
        valid_iters = iterations(~isnan(path_lengths));
        
        results(iv).interval = interval;
        results(iv).success_rate = success_count / num_runs;
        results(iv).mean_time = mean(valid_times);
        results(iv).std_time = std(valid_times);
        results(iv).mean_length = mean(valid_lengths);
        results(iv).std_length = std(valid_lengths);
        results(iv).mean_iterations = mean(valid_iters);
        
        fprintf('\n汇总:\n');
        fprintf('  成功率: %.1f%%\n', results(iv).success_rate * 100);
        fprintf('  平均收敛时间: %.4f ± %.4f 秒\n', ...
                results(iv).mean_time, results(iv).std_time);
        fprintf('  平均路径长度: %.2f ± %.2f\n', ...
                results(iv).mean_length, results(iv).std_length);
        fprintf('  平均迭代次数: %.1f\n', results(iv).mean_iterations);
    else
        results(iv).interval = interval;
        results(iv).success_rate = 0;
        results(iv).mean_time = NaN;
        results(iv).std_time = NaN;
        results(iv).mean_length = NaN;
        results(iv).std_length = NaN;
        results(iv).mean_iterations = NaN;
        fprintf('\n汇总: 全部失败\n');
    end
    fprintf('\n');
end

% =================== 结果对比表格 ===================
fprintf('\n========================================\n');
fprintf('实验结果汇总\n');
fprintf('========================================\n');
fprintf('%-10s %-15s %-20s %-20s %-15s\n', ...
        'Interval', '成功率', '平均时间(秒)', '平均长度', '平均迭代');
fprintf('%-10s %-15s %-20s %-20s %-15s\n', ...
        '--------', '------', '------------', '--------', '--------');

for iv = 1:length(interval_values)
    if results(iv).success_rate > 0
        fprintf('%-10d %-14.1f%% %-20s %-20s %-15.1f\n', ...
                results(iv).interval, ...
                results(iv).success_rate * 100, ...
                sprintf('%.4f ± %.4f', results(iv).mean_time, results(iv).std_time), ...
                sprintf('%.2f ± %.2f', results(iv).mean_length, results(iv).std_length), ...
                results(iv).mean_iterations);
    else
        fprintf('%-10d %-14.1f%% %-20s %-20s %-15s\n', ...
                results(iv).interval, 0, 'N/A', 'N/A', 'N/A');
    end
end

fprintf('\n论文参考值 (100次运行平均):\n');
fprintf('Interval=3:  0.195s, 1136.0\n');
fprintf('Interval=4:  0.034s, 1157.4\n');
fprintf('Interval=6:  0.056s, 1170.1\n');
fprintf('Interval=8:  0.064s, 1187.0\n');
fprintf('Interval=10: 0.072s, 1197.5\n');
fprintf('Interval=20: 0.108s, 1250.7\n');
fprintf('========================================\n');

% =================== 可视化对比 ===================
% 提取有效数据
valid_indices = find([results.success_rate] > 0);
intervals_plot = [results(valid_indices).interval];
times_plot = [results(valid_indices).mean_time];
lengths_plot = [results(valid_indices).mean_length];

% 论文参考数据
paper_intervals = [3, 4, 6, 8, 10, 20];
paper_times = [0.195, 0.034, 0.056, 0.064, 0.072, 0.108];
paper_lengths = [1136.0, 1157.4, 1170.1, 1187.0, 1197.5, 1250.7];

if ~isempty(valid_indices)
    figure('Name', 'Interval参数对比', 'Position', [100 100 1200 500]);
    
    % 收敛时间对比
    subplot(1, 2, 1);
    hold on; grid on;
    plot(intervals_plot, times_plot, 'bo-', 'LineWidth', 2, 'MarkerSize', 8, ...
         'DisplayName', '本次实验');
    plot(paper_intervals, paper_times, 'rs--', 'LineWidth', 2, 'MarkerSize', 8, ...
         'DisplayName', '论文结果');
    xlabel('Interval');
    ylabel('收敛时间 (秒)');
    title('Interval vs 收敛时间');
    legend('Location', 'best');
    set(gca, 'FontSize', 11);
    
    % 路径长度对比
    subplot(1, 2, 2);
    hold on; grid on;
    plot(intervals_plot, lengths_plot, 'bo-', 'LineWidth', 2, 'MarkerSize', 8, ...
         'DisplayName', '本次实验');
    plot(paper_intervals, paper_lengths, 'rs--', 'LineWidth', 2, 'MarkerSize', 8, ...
         'DisplayName', '论文结果');
    xlabel('Interval');
    ylabel('路径长度');
    title('Interval vs 路径长度');
    legend('Location', 'best');
    set(gca, 'FontSize', 11);
end

% =================== 保存结果 ===================
save('interval_comparison_results.mat', 'results', 'config', 'num_runs');
fprintf('\n结果已保存到: interval_comparison_results.mat\n');
